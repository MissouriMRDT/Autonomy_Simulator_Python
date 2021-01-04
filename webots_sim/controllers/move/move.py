"""Simple robot controller."""

from controller import Robot, Keyboard, Pen
import sys, time, math
from rovecomm import RoveComm, RoveCommPacket


current_milli_time = lambda: int(round(time.time() * 1000))
watchdog_timer = current_milli_time()
sensor_update_timer = current_milli_time()

# The update rate for both inputs and outputs
UPDATE_RATE = 100

# Define the max motor speed in radians.
MAX_SPEED = 0.6


def test(packet):
    print("Hello")


def drive(packet):
    global watchdog_timer
    left, right = packet.data
    print(f"Left: {left}")
    print(f"Right: {right}")
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
        print(f"lat: {lat}")
        print(f"lon: {lon}")
        #print(f"alt: {alt}")
        # Some lambdas to handle sensor data conversion
        conv_gps_to_int = lambda x: 0 if math.isnan(x) else int(x * 1e7)
        conv_imu_to_int = lambda x: 0 if math.isnan(x) else int(x)

        # Convert lat, lon to ints, some NaN trickiness
        lat = conv_gps_to_int(lat)
        lon = conv_gps_to_int(lon)
        print(f"lat: {lat * 1e-7}")
        print(f"lon: {lon * 1e-7}")
        
        # Convert pitch, yaw, roll to degrees (is in radians)
        yaw = -yaw
        
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)
        
        if yaw < 0:
            yaw = 360 + yaw
            
        #print(yaw)
        
        # Convert pitch, yaw, roll to ints
        roll = conv_imu_to_int(roll)
        pitch = conv_imu_to_int(pitch)
        yaw = conv_imu_to_int(yaw)

        # Send GPS to Autonomy
        packet = RoveCommPacket(int(5100), "l", (lat, lon), "129", 11000)
        rovecomm_node.write(packet, False)

        # Send the rover ortientation to Autonomy
        packet = RoveCommPacket(int(5101), "h", (pitch, yaw, roll), "129", 11000)
        rovecomm_node.write(packet, False)

        # Update the timer
        sensor_update_timer = current_milli_time()


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


"""
    key = keyboard.getKey()
    if key == Keyboard.LEFT:
        leftMotor.setVelocity(MAX_SPEED)         
        leftMotor2.setVelocity(MAX_SPEED)
        leftMotor3.setVelocity(MAX_SPEED)

        rightMotor.setVelocity(-MAX_SPEED)
        rightMotor2.setVelocity(-MAX_SPEED)
        rightMotor3.setVelocity(-MAX_SPEED)
        watchdog_timer = current_milli_time()

    elif key == Keyboard.RIGHT:
        leftMotor.setVelocity(-MAX_SPEED)
        leftMotor2.setVelocity(-MAX_SPEED)
        leftMotor3.setVelocity(-MAX_SPEED)

        rightMotor.setVelocity(MAX_SPEED)
        rightMotor2.setVelocity(MAX_SPEED)
        rightMotor3.setVelocity(MAX_SPEED)
        watchdog_timer = current_milli_time()

    elif key == Keyboard.UP:
        leftMotor.setVelocity(-MAX_SPEED)
        leftMotor2.setVelocity(-MAX_SPEED)
        leftMotor3.setVelocity(-MAX_SPEED)

        rightMotor.setVelocity(-MAX_SPEED)
        rightMotor2.setVelocity(-MAX_SPEED)
        rightMotor3.setVelocity(-MAX_SPEED)
        watchdog_timer = current_milli_time()

    elif key == Keyboard.DOWN:
        leftMotor.setVelocity(MAX_SPEED)
        leftMotor2.setVelocity(MAX_SPEED)
        leftMotor3.setVelocity(MAX_SPEED)

        rightMotor.setVelocity(MAX_SPEED)
        rightMotor2.setVelocity(MAX_SPEED)
        rightMotor3.setVelocity(MAX_SPEED)
        watchdog_timer = current_milli_time()
"""
