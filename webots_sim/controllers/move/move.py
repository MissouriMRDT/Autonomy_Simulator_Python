"""Simple robot controller."""

from controller import Robot, Keyboard, Pen, Display
import sys, time, math
from rovecomm import RoveComm, RoveCommPacket
import socket, cv2, pickle,struct
import numpy as np



current_milli_time = lambda: int(round(time.time() * 1000))
watchdog_timer = current_milli_time()
sensor_update_timer = current_milli_time()
camera_update_timer = current_milli_time()

# The update rate for both inputs and outputs
UPDATE_RATE = 100

# Define the max motor speed in radians.
MAX_SPEED = 0.6

# Define the camera FPS
FPS = 30

# Socket Create
server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

host_ip = "127.0.0.1"
print('HOST IP:',host_ip)
port = 9999
socket_address = (host_ip,port)

# Socket Bind
server_socket.bind(socket_address)

# Socket Listen
server_socket.listen(5)
print("LISTENING AT:",socket_address)


def drive(packet):
    global watchdog_timer
    left, right = packet.data
    #print(f"Left: {left}")
    #print(f"Right: {right}")
    leftMotor.setVelocity((left / 250) * MAX_SPEED)
    leftMotor2.setVelocity((left / 250) * MAX_SPEED)
    leftMotor3.setVelocity((left / 250) * MAX_SPEED)

    rightMotor.setVelocity((right / 250) * MAX_SPEED)
    rightMotor2.setVelocity((right / 250) * MAX_SPEED)
    rightMotor3.setVelocity((right / 250) * MAX_SPEED)
    watchdog_timer = current_milli_time()


current_milli_time = lambda: int(round(time.time() * 1000))

rovecomm_node = RoveComm(11001, ("", 11112))
rovecomm_node.set_callback(1000, drive)

# Get pointer to the robot.
robot = Robot()
keyboard = Keyboard()
keyboard.enable(64)

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Get left and right wheel motors.
# because I am stupid, currently the rover drives backwards, and hence
# the motor sides are actually flipped.

rightMotor = robot.getDevice("FrontLeftWheel")
rightMotor2 = robot.getDevice("MiddleLeftWheel")
rightMotor3 = robot.getDevice("BackLeftWheel")
leftMotor = robot.getDevice("FrontRightWheel")
leftMotor2 = robot.getDevice("MiddleRightWheel")
leftMotor3 = robot.getDevice("BackRightWheel")

gps = robot.getDevice("gps")
gps.enable(64)

camera = robot.getDevice("camera")
camera.enable(64)

imu = robot.getDevice("inertial unit")
imu.enable(64)

compass = robot.getDevice("compass")
compass.enable(64)

pen = robot.getDevice("pen")
pen.write(True)


# Disable motor PID control mode.
leftMotor.setPosition(float("inf"))
leftMotor2.setPosition(float("inf"))
leftMotor3.setPosition(float("inf"))

rightMotor.setPosition(float("inf"))
rightMotor2.setPosition(float("inf"))
rightMotor3.setPosition(float("inf"))

def send_sensor_data():
    global sensor_update_timer
    if current_milli_time() - sensor_update_timer > UPDATE_RATE:
        lat, lon, alt = gps.getValues()
        roll, pitch, yaw = imu.getRollPitchYaw()
        
        # Some lambdas to handle sensor data conversion
        conv_gps_to_int = lambda x: 0 if math.isnan(x) else int(x * 1e7)
        conv_imu_to_int = lambda x: 0 if math.isnan(x) else int(x)

        # Convert lat, lon to ints, some NaN trickiness
        lat = conv_gps_to_int(lat)
        lon = conv_gps_to_int(lon)
        #print(f"lat: {lat * 1e-7}")
        #print(f"lon: {lon * 1e-7}")
        
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

        # Send GPS to Autonomy
        packet = RoveCommPacket(int(5100), "l", (lat, lon), "127.0.0.1", 11000)
        rovecomm_node.write(packet, False)

        # Send the rover ortientation to Autonomy
        packet = RoveCommPacket(int(5101), "h", (pitch, yaw, roll), "127.0.0.1", 11000)
        rovecomm_node.write(packet, False)

        # Update the timer
        sensor_update_timer = current_milli_time()
        
client_socket = None
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

while robot.step(timeStep) != -1:
    # Very rudimentary watchdog
    if current_milli_time() - watchdog_timer > UPDATE_RATE*1.5:
        leftMotor.setVelocity(0)
        leftMotor2.setVelocity(0)
        leftMotor3.setVelocity(0)

        rightMotor.setVelocity(0)
        rightMotor2.setVelocity(0)
        rightMotor3.setVelocity(0)
        watchdog_timer = current_milli_time()

    # send the sensor data to the autonomy program
    send_sensor_data()
    
    if current_milli_time() - camera_update_timer > (1000/30):
        # stream the camera feed directly to autonomy
        if client_socket == None:
            client_socket,addr = server_socket.accept()
            print('GOT CONNECTION FROM:',addr)
        if client_socket:
            frame = camera.getImage()
            frame = np.frombuffer(frame, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
            result, frame = cv2.imencode('.jpg', frame, encode_param)
            a = pickle.dumps(frame)
            message = struct.pack("Q",len(a))+a
            client_socket.sendall(message)
        camera_update_timer = current_milli_time()

   
client_socket.close()


