from rovecomm import RoveComm, RoveCommPacket
import math
import time
import socket
import numpy as np
import pickle
import struct
import cv2
import gzip
import sys

# Quick lambda to return current time in ms
current_milli_time = lambda: int(round(time.time() * 1000))


class Rover:
    def __init__(self, robot, rovecomm_node):
        self.robot = robot
        self.rovecomm_node = rovecomm_node

        # The update rate for telemetry/commands
        self.UPDATE_RATE = 100

        # The max motor speed in radians
        self.MAX_SPEED = 0.6

        # Define the camera FPS
        self.FPS = 30

        # Get left and right wheel motors.
        # The motor sides are currently actually flipped.
        self.rightMotor = self.robot.getDevice("FrontLeftWheel")
        self.rightMiddleMotor = self.robot.getDevice("MiddleLeftWheel")
        self.rightBackMotor = self.robot.getDevice("BackLeftWheel")
        self.leftFrontMotor = self.robot.getDevice("FrontRightWheel")
        self.leftMiddleMotor = self.robot.getDevice("MiddleRightWheel")
        self.leftBackMotor = self.robot.getDevice("BackRightWheel")

        # Disable motor PID control mode.
        self.leftFrontMotor.setPosition(float("inf"))
        self.leftMiddleMotor.setPosition(float("inf"))
        self.leftBackMotor.setPosition(float("inf"))
        self.rightMotor.setPosition(float("inf"))
        self.rightMiddleMotor.setPosition(float("inf"))
        self.rightBackMotor.setPosition(float("inf"))

        # Grab all the devices on the rover and enable them
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(64)

        self.camera = self.robot.getDevice("camera")
        self.camera.enable(64)

        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(64)

        self.compass = self.robot.getDevice("compass")
        self.compass.enable(64)

        self.pen = self.robot.getDevice("pen")
        self.pen.write(True)

        self.depth = self.robot.getDevice("range-finder")
        self.depth.enable(64)

        # Set up some timers to use for watchdogs/telemetry rates
        self.sensor_update_timer = current_milli_time()
        self.watchdog_timer = current_milli_time()
        self.camera_update_timer = current_milli_time()

        # Create a socket used to stream camera data to Autonomy over local network
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        host_ip = "127.0.0.1"
        port = 9999
        socket_address = (host_ip, port)

        # Socket Bind
        self.server_socket.bind(socket_address)

        # Socket Listen
        self.server_socket.listen(5)
        print("LISTENING AT:", socket_address)

        # The client socket (in this case Autonomy running in SIM mode)
        self.client_socket = None

        # The parameters for encoding images
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

    def close(self):
        if self.client_socket != None:
            self.client_socket.shutdown()
            self.client_socket.close()

    def drive_callback(self, packet):
        left, right = packet.data
        self.leftFrontMotor.setVelocity((left / 250) * self.MAX_SPEED)
        self.leftMiddleMotor.setVelocity((left / 250) * self.MAX_SPEED)
        self.leftBackMotor.setVelocity((left / 250) * self.MAX_SPEED)

        self.rightMotor.setVelocity((right / 250) * self.MAX_SPEED)
        self.rightMiddleMotor.setVelocity((right / 250) * self.MAX_SPEED)
        self.rightBackMotor.setVelocity((right / 250) * self.MAX_SPEED)

        self.watchdog_timer = current_milli_time()

    def drive_watchdog_check(self):
        # Very rudimentary watchdog
        if current_milli_time() - self.watchdog_timer > self.UPDATE_RATE * 1.5:
            self.leftFrontMotor.setVelocity(0)
            self.leftMiddleMotor.setVelocity(0)
            self.leftBackMotor.setVelocity(0)

            self.rightMotor.setVelocity(0)
            self.rightMiddleMotor.setVelocity(0)
            self.rightBackMotor.setVelocity(0)
            self.watchdog_timer = current_milli_time()

    def send_sensor_update(self):
        if current_milli_time() - self.sensor_update_timer > self.UPDATE_RATE:
            lat, lon, alt = self.gps.getValues()
            roll, pitch, yaw = self.imu.getRollPitchYaw()

            # Some lambdas to handle sensor data conversion
            conv_gps_to_int = lambda x: 0 if math.isnan(x) else int(x * 1e7)
            conv_imu_to_int = lambda x: 0 if math.isnan(x) else int(x)

            # Convert lat, lon to ints, some NaN trickiness
            lat = conv_gps_to_int(lat)
            lon = conv_gps_to_int(lon)

            # Convert pitch, yaw, roll to degrees (is in radians)
            yaw = -yaw

            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)

            if yaw < 0:
                yaw = 360 + yaw

            # Convert pitch, yaw, roll to ints
            roll = conv_imu_to_int(roll)
            pitch = conv_imu_to_int(pitch)
            yaw = conv_imu_to_int(yaw)

            # Send GPS to Autonomy (and other subscribers)
            packet = RoveCommPacket(int(5100), "l", (lat, lon), "127.0.0.1", 11000)
            self.rovecomm_node.write(packet, False)

            # Send the rover ortientation to Autonomy (and other subscribers)
            packet = RoveCommPacket(int(5101), "h", (pitch, yaw, roll), "127.0.0.1", 11000)
            self.rovecomm_node.write(packet, False)

            # Update the timer
            self.sensor_update_timer = current_milli_time()

    def stream_frame(self):
        if current_milli_time() - self.camera_update_timer > (1000 / self.FPS):
            # stream the camera feed directly to autonomy
            if self.client_socket == None:
                self.client_socket, addr = self.server_socket.accept()
                print("GOT CONNECTION FROM:", addr)
            if self.client_socket:
                # Grab the frame from the camera and convert to numpy array
                frame = self.camera.getImage()
                frame = np.frombuffer(frame, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))

                # Encode the frame before we transmit it, reduces size and speeds up process
                result, frame = cv2.imencode(".jpg", frame, self.encode_param)

                # Pickle the frame, and send it over socket
                a = pickle.dumps(frame)
                # Pack message, specify this is "r"egular image
                message = struct.pack("Q", len(a)) + "r".encode() + a
                self.client_socket.sendall(message)

                # Do the same for the depth frame
                depth_frame = self.depth.getRangeImage()

                # Depth is a float, so convert those to bytes
                depth_frame = struct.pack("%sf" % len(depth_frame), *depth_frame)

                # Compress/Pickle the frame, and send it over socket
                a = gzip.compress(pickle.dumps(depth_frame))
                # Pack message, specify this is "d"epth data
                message = struct.pack("Q", len(a)) + "d".encode() + a
                self.client_socket.sendall(message)

            self.camera_update_timer = current_milli_time()