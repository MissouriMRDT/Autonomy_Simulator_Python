# Autonomy Simulator

This repo contains python code and simulation files used to simulate the MRDT Autonomy software stack in the [Webots](https://cyberbotics.com/) robot simulator software.

## Details

Currently all scripts are written in Python. We utilize RoveComm (the rover UDP/TCP communication protocol) to talk between the Autonomy stack and the simulator.

Currently the simulator mocks the following onboard rover functionality:

- GPS/IMU data (orientation/heading) sent at a rate of 100ms
- Supports receiving drive commands, with a 150 ms watchdog to stop driving
