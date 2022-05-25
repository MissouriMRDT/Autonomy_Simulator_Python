"""Simple robot controller."""
from controller import Robot, Keyboard, Pen, Display
import sys, time, math
from rovecomm import RoveComm, RoveCommPacket
import socket, cv2, pickle, struct
import numpy as np
from rover import Rover
from threading import Thread

rovecomm_node = RoveComm(11001, ("", 11112))

# Get pointer to the robot.
robot = Robot()
keyboard = Keyboard()
keyboard.enable(64)

# Initialize the rover class, with our rovecomm node
rover = Rover(robot, rovecomm_node)
rovecomm_node.set_callback(1000, rover.drive_callback)

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Start frame streaming thread.
t1 = Thread(target = rover.stream_frame)
t1.setDaemon(True)
t1.start()

while robot.step(timeStep) != -1:
    # Check if we need to trigger watchdog and stop driving
    rover.drive_watchdog_check()

    # send the sensor data to the autonomy program
    rover.send_sensor_update()

    # Stream the current frame (at fixed FPS)
    if not t1.is_alive():
        # Restart thread.
        t1.join()
        t1 = Thread(target = rover.stream_frame)
        t1.setDaemon(True)
        t1.start()
        
# Cleanup.
rover.close()
t1.join()