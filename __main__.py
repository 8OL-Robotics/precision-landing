import queue
from estimators.aruco_reader import ArucoReader
from offboard_commander import OffboardCommander
from multiprocessing import Process, Queue, log_to_stderr
import os
import sys
import logging
import asyncio
import time

log_to_stderr(logging.DEBUG)


def start_aruco_reader(inputQueue):
    aruco_reader = ArucoReader(inputQueue, mode="simulation")
    aruco_reader.start_reading()

def start_commander(commander):
    commander.start_fsm()

if __name__ == "__main__":

    inputq = Queue()
    inputProcess = Process(target=start_aruco_reader, args=(inputq,))
    inputProcess.start()
    
    offboard_commander = OffboardCommander(
        connection_address="udpin:0.0.0.0:14550", inputQueue=inputq, controller=None
    )
    commanderProcess=  Process(target=start_commander, args=(offboard_commander,))
    commanderProcess.start()

    commanderProcess.join()
    
    print("Offboard Commander session ended")

    inputProcess.join()
