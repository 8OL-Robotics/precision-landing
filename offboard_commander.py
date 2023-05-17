from cmath import inf
import queue
from time import sleep
from transitions.extensions.asyncio import AsyncMachine
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
import asyncio
from controllers.NED_controllers import PID
import time


class OffboardCommander:
    _controller = None
    _estimateQueue = None
    _connection_address = None
    _drone = None

    def __init__(self, connection_address, inputQueue, controller) -> None:
        self._controller = controller
        self._estimateQueue = inputQueue
        self._connection_address = connection_address
        self._drone = System()

        self.machine = AsyncMachine(
            model=self,
            states=[
                "idle",
                "connected",
                "offboard",
                "armed",
                "takeoff",
                "scan_aruco",
                "track_aruco_xy",
                "track_aruco_xyz",
            ],
            transitions=[
                {
                    "trigger": "attempt_connect",
                    "source": "idle",
                    "dest": "connected",
                    "before": "attempt_connection",
                    "after": "arm_drone",
                },
                {
                    "trigger": "arm",
                    "source": "connected",
                    "dest": "armed",
                    "before": "arm_drone",
                },
                {
                    "trigger": "activate_offboard",
                    "source": "armed",
                    "dest": "offboard",
                    "before": "enable_offboard",
                },
                {
                    "trigger": "stage_for_landing_attempt",
                    "source": "offboard",
                    "dest": "track_aruco_xy",
                    "before": "set_stage_xy",
                },
                {
                    "trigger": "follow_pad_xy",
                    "source": "track_aruco_xy",
                    "dest": "track_aruco_xyz",
                    "before": "go_above_aruco",
                },
            ],
            initial="idle",
            ignore_invalid_triggers=True,
            after_state_change="print_state",
        )

    def print_state(self):
        print(f"State changed to {self.state}")

    async def check_if_connected(self):
        async for state in self._drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
                break

    async def attempt_connection(self):
        await self._drone.connect(
            system_address=self._connection_address,
        )
        print("Waiting for drone to connect...")

        await self.check_if_connected()

        async for health in self._drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

    async def arm_drone(self):
        print("-- Arming")
        await self._drone.action.arm()

    async def enable_offboard(self):
        await self._drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        print("-- Starting offboard")
        try:
            await self._drone.offboard.start()
        except OffboardError as error:
            print(
                f"Starting offboard mode failed with error code: {error._result.result}"
            )
            print("-- Disarming")
            await self._drone.action.disarm()
            return

        print("Offboard mode activated")

    async def set_stage_xy(self):
        await self._drone.offboard.set_position_ned(PositionNedYaw(0, 0, -2, 1.57))

    async def aruco_stream_active(self):
        initial = self._estimateQueue.qsize()
        await asyncio.sleep(1)
        final = self._estimateQueue.qsize()

        return not final == initial

    async def go_above_aruco(self):
        # start PID
        controller_x = PID()
        controller_y = PID()

        z_val = -2

        controller_z = PID(P=0.000002)

        time_when_state_last_steady = 0

        OFFSET_X = 50
        OFFSET_Y = 3

        ERROR_MARGIN = 50
        while True:
            estimate = self._estimateQueue.get()

            if (
                abs(estimate[1][0] - OFFSET_X) < ERROR_MARGIN
                and abs(estimate[1][1] - OFFSET_Y) < ERROR_MARGIN
            ):
                time_when_state_last_steady = time.time()

            controller_x.update(estimate[1][0] + OFFSET_X)
            controller_y.update(estimate[1][1] + OFFSET_Y)

            print("x:", estimate[1][0], " ,y:", estimate[1][1], ",z:", z_val)

            if time.time() - time_when_state_last_steady < 1:
                z_val += 0.004

            await self._drone.offboard.set_position_ned(
                PositionNedYaw(controller_y.output, -controller_x.output, z_val, 1.57)
            )

    async def start_fsm(self):
        await self.attempt_connect()
        await self.arm()
        await self.activate_offboard()
        await self.stage_for_landing_attempt()
        await self.follow_pad_xy()

        print(bool)

        sleep(10)
