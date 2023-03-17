from rovecomm import RoveComm, RoveCommPacket
import math
import time
import socket
import numpy as np
import pickle
import struct
import cv2
import gzip
import select
import asyncio

# Quick lambda to return current time in ms
current_milli_time = lambda: int(round(time.time() * 1000))


class Rover:
    def __init__(self, robot, rovecomm_node):
        self.robot = robot
        self.rovecomm_node = rovecomm_node

        # The update rate for telemetry/commands
        self.UPDATE_RATE = 100

        # Define the timestep for polling sensors in virtual world.
        self.timestep = 64

        # The max motor speed in radians
        self.MAX_SPEED = 4

        # Define the camera FPS
        self.FPS = 30

        # Get left and right wheel motors.
        # The motor sides are currently actually flipped.
        self.rightMotor = self.robot.getDevice("wheel2")
        self.rightBackMotor = self.robot.getDevice("wheel4")
        self.leftFrontMotor = self.robot.getDevice("wheel1")
        self.leftBackMotor = self.robot.getDevice("wheel3")

        # Disable motor PID control mode.
        self.leftFrontMotor.setPosition(float("inf"))
        self.leftBackMotor.setPosition(float("inf"))
        self.rightMotor.setPosition(float("inf"))
        self.rightBackMotor.setPosition(float("inf"))

        # Grab all the devices on the rover and enable them
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)

        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)

        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)

        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)

        self.pen = self.robot.getDevice("pen")
        self.pen.write(True)

        self.depth = self.robot.getDevice("range-finder")
        self.depth.enable(self.timestep)

        self.lidar = self.robot.getDevice("lidar")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

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
        self.server_socket.listen(65535)
        print("LISTENING AT:", socket_address)

        # The client socket (in this case Autonomy running in SIM mode)
        self.client_socket = None

        # The parameters for encoding images
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        
        # Thread stopping toggle.
        self.stop = False

    def close(self):
        # Set stopping toggle.
        self.stop = True
        # Close socket connection.
        if self.client_socket != None:
            self.client_socket.shutdown()
            self.client_socket.close()

    def drive_callback(self, packet):
        left, right = packet.data
        self.leftFrontMotor.setVelocity((left / 250) * self.MAX_SPEED)
        self.leftBackMotor.setVelocity((left / 250) * self.MAX_SPEED)

        self.rightMotor.setVelocity((right / 250) * self.MAX_SPEED)
        self.rightBackMotor.setVelocity((right / 250) * self.MAX_SPEED)

        self.watchdog_timer = current_milli_time()

    def drive_watchdog_check(self):
        # Very rudimentary watchdog
        if current_milli_time() - self.watchdog_timer > self.UPDATE_RATE * 1.5:
            self.leftFrontMotor.setVelocity(0)
            self.leftBackMotor.setVelocity(0)

            self.rightMotor.setVelocity(0)
            self.rightBackMotor.setVelocity(0)
            self.watchdog_timer = current_milli_time()

    def send_sensor_update(self):
        if current_milli_time() - self.sensor_update_timer > self.UPDATE_RATE:
            lat, lon, alt = self.gps.getValues()
            print(lat,lon,alt)
            roll, pitch, yaw = self.imu.getRollPitchYaw()

            # Some lambdas to handle sensor data conversion
            # conv_gps_to_int = lambda x: 0 if math.isnan(x) else int(x * 1e7)
            # conv_imu_to_int = lambda x: 0 if math.isnan(x) else int(x)

            # Convert lat, lon to ints, some NaN trickiness
            # lat = conv_gps_to_int(lat)
            # lon = conv_gps_to_int(lon)

            # Convert pitch, yaw, roll to degrees (is in radians)
            yaw = -yaw

            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)
            print(yaw)

            if yaw < 0:
                yaw = 360 + yaw

            # Send GPS to Autonomy (and other subscribers)
            packet = RoveCommPacket(int(5100), "d", (lat, lon), "127.0.0.1", 11000)
            self.rovecomm_node.write(packet, False)

            # Send the rover ortientation to Autonomy (and other subscribers)
            packet = RoveCommPacket(int(5101), "f", (pitch, yaw, roll), "127.0.0.1", 11000)
            self.rovecomm_node.write(packet, False)

            # Update the timer
            self.sensor_update_timer = current_milli_time()

    def stream_frame(self):
        while not self.stop:
            if current_milli_time() - self.camera_update_timer > (1000 / 15):
                # stream the camera feed directly to autonomy
                if len(select.select([self.server_socket], [], [], 0)[0]) > 0:
                    self.client_socket, addr = self.server_socket.accept()
                    print("GOT CONNECTION FROM:", addr)
    
                # If we had a socket connection, at least attempt to stream frames
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
    
                    # Check if there is an available socket to send on
                    try:
                        self.client_socket.sendall(message)
                    except socket.error:
                        self.client_socket = None
                        return
    
                    # Do the same for the depth frame
                    depth_frame = self.depth.getRangeImage()
                    depth_frame = [x * 1000 for x in depth_frame]
    
                    # Depth is a float, so convert those to bytes
                    depth_frame = struct.pack("%sf" % len(depth_frame), *depth_frame)
    
                    # Compress/Pickle the frame, and send it over socket
                    a = gzip.compress(pickle.dumps(depth_frame))
                    # Pack message, specify this is "d"epth data
                    message = struct.pack("Q", len(a)) + "d".encode() + a
                    
                    # Check if there is an available socket to send on
                    try:
                        self.client_socket.sendall(message)
                    except socket.error:
                        self.client_socket = None
                        return
                    
                    # Do the same for the point cloud.
                    point_cloud = self.lidar.getPointCloud(data_type="buffer") # buffer bytearray is faster than list.
                    # Read the buffer byte data into a numpy array for fast processing.
                    point_cloud = np.frombuffer(point_cloud, dtype=np.float32)
                    # Reshape the array to create a point cloud image frame with the size of 1280x720 (same as configured in lidar properties.)
                    point_cloud = point_cloud.reshape((self.lidar.getNumberOfLayers(), self.lidar.getHorizontalResolution(), 5))
                    # Cutoff last value. The values are now x,y,z,pointlayer. The pointlayer is useless, but we need a forth value to emulate what the zed cam returns.
                    point_cloud = point_cloud[:,:,:4]
                    # Create new copy since point_cloud is read-only.
                    pcd = point_cloud.copy()
                    # Remove infinity.
                    pcd[np.isinf(pcd)] = 0
                    pcd[np.isneginf(pcd)] = 0
                    # Convert meters to millimeters.
                    pcd *= 1000
                    # Scale oll ofthe values between 0-255 since we will be encoding to an image for speed. This reduces accuracy to +-40cm.
                    minmax = np.array([pcd.min(), pcd.max()], dtype=np.float32)
                    pcd = np.interp(pcd, (pcd.min(), pcd.max()), (0, 255))
                    # Round floats to int values.
                    pcd = np.rint(pcd)
                    # Convert array to int32.
                    pcd = pcd.astype(np.int32)
                    
                    # Encode the frame before we transmit it, reduces size and speeds up process
                    result, point_cloud_frame = cv2.imencode(".png", pcd, [int(cv2.IMWRITE_PNG_COMPRESSION), 3])
                    # Pickle the frame, and send it over socket
                    a = pickle.dumps(point_cloud_frame)
                    # Pack message, specify this is "p"ointcloud image
                    message = struct.pack("Q", len(a)) + "p".encode() + a
    
                    # Check if there is an available socket to send on
                    try:
                        self.client_socket.sendall(message)
                    except socket.error:
                        self.client_socket = None
                        return
                        
                    
                    # minmax is a float, so convert those to bytes
                    minmax = minmax.tobytes()
                    # Attach the min and max values to the end of the message. This is used by the autonomy code to rescale things back to normal.
                    message = struct.pack("Q", len(minmax)) + "m".encode() + minmax
                    
                    # Check if there is an available socket to send on
                    try:
                        self.client_socket.sendall(message)
                    except socket.error:
                        self.client_socket = None
                        return
                    
    
                self.camera_update_timer = current_milli_time()