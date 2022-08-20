import queue
from estimators.aruco_reader import ArucoReader
from offboard_commander import OffboardCommander
from multiprocessing import Process, Queue, log_to_stderr
import os
import sys
import logging
import asyncio

log_to_stderr(logging.DEBUG)


def start_aruco_reader(inputQueue):
    aruco_reader = ArucoReader(inputQueue, mode="simulation")
    aruco_reader.start_reading()


if __name__ == "__main__":

    inputq = Queue()
    inputProcess = Process(target=start_aruco_reader, args=(inputq,))
    inputProcess.start()
    offboard_commander = OffboardCommander(
        connection_address="udp://:14540", inputQueue=inputq, controller=None
    )

    ## start asyncio FSM

    loop = asyncio.get_event_loop()
    loop.run_until_complete(offboard_commander.start_fsm())

    print("Offboard Commander session ended")

    inputProcess.join()
