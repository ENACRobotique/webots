# VRAC Webots examples for Eurobot

![Eurobot 2025 table with robots](Eurobot2025.png)

This repo is intended to show a simple Eurobot simulation with Webots  
The simulation step is 10 ms (defined in world.wbt) and remember the simulated physics are not perfect!  
This should help you debug and test your robot movement and strategy while the mechanical team works on the robot :D  
These examples are in Python to be more concise

The VRACRobot.proto contains:
- Two wheels driven by motor with encoder
- Two odometry wheels with encoders, mounted on a linear rail to assure ground contact
- A lidar
- A camera
- A servo with a vacuum gripper

There is 3 controllers example:

- **GamepadRobot**: controller for VRACRobot.proto  
Gamepad controls: Left axis (direction) RT (+speed), LT (-speed), RB (servo), A (vacuum)  
Motors and encoders data are sent to [Teleplot](https://github.com/nesnes/teleplot)!

- **VRACRobotController**: external controller for VRACRobot.proto  
See section [VRACRobotController](#VRACRobotController)

- **DummyOpponent**: controller for DummyOpponent.proto  
Move a dummy enemy robot to random predefined positions on the table  
Used to check the VRACRobot main strategy will handle random situations  
This dummy robot has no collisions, it is only detected by lidar points by the VRACRobot  
(thanks memristor for this controller)

## VRACRobotController

VRACRobotController.py: an external controller that is used to interface the real robot program with the Webots simulation environment.  

for this example we are using the following programs:

- **raspiboard**: main program that will run on the Raspberry Pi, has lidar connected, communicate through CAN with motorboard

- **motorboard**: embed program that is running on a microcontroller, has motors and odometry connected, communicate through CAN with raspiboard (the program is in Python to simplify this example)

![diagram of simulation with controller](VRACRobotController.png)

Usage:
```sh
# Install packages
sudo apt install python3 can-utils tmux

# Setup virtual CAN network interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip link show

# open world.wbt, reset and start simulation

# start external controller VRACRobotController
cd controllers/VRACRobotController
./start.sh
# to quit tmux session, "Ctrl-b" then "Ctrl-q" or "tmux kill-session"
```

# Thanks

https://github.com/memristor/mep3/tree/main/mep3_simulation

https://github.com/cvra/robot-software/tree/master/webot_sim

https://github.com/robotique-ecam/simulation-world

