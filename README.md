# Robotics Simulation
![](demo/Roger001.gif)
*The simulator program is credited to Roderic Grupen, professor of Computer science and director of the Laboratory for Perceptual Robotics at the University of Massachusetts Amherst.*
*The simulator program was removed on purpose on github. Only project files are displayed.*

## Relevant Skills:

- ### Stereo Localization
The robot uses its stereo vision(two eyes) to appoximate the depth of the vision to localize the red ball.\
![](demo/roger-stereoVision.gif)


- ### PD Control
Spring-Mass-Damper, Critical Damping, Open/Closed-Loop Control, Laplace Transform, etc.\
![](demo/roger-PDcontrol.gif)


- ### Kinematics
Forward and Inverse kinematics\
![](demo/roger-kinematics.gif)


- ### State Machine
Finite State Machine\
The robot transforms into defensive mode based on the status of the finite state machine.\
![](demo/roger-statemachine.gif)

### +
- ### Homogeneous Transform
- ### Dynamics
- ### Controls
- ### Path Planning


---
```
STEPS FOR COMPILING THE SIMULATOR

1. Unzip roger.zip

2. Go to RogerSimulator. run make clean; make

3. Go to RogerClient run make clean; make

4. Copy the generated libraries to RogerProjects. In the RogerProjects Directory you can run:
       cp ../RogerSimulator/lib/dynamics.a lib/
       cp ../RogerClient/lib/SocketComm.a lib/

The above steps need to be done only once.

5. Compile RogerProjects by running make clean; make
This will generate an executable named roger

STEPS FOR RUNNING THE SIMULATOR

1. Open two terminal windows.
2. In the first window, change the current directory to RogerSimulator. In the other two windows, change the directory to RogerProjects.
3. In the first window run: ./simulator EnvironmentNum RobotNum
	,where EnvironmentNum and RobotNum are integer arguments that determine the 
	simulation environment and number of robots respectively.
	EnvironmentNum = 0 : ARENA
	EnvironmentNum = 1 : DEVELOPMENTAL
    This starts up the simulator. However, the simulator will not display until it has connection from RobotNum different rogers.
4. In the second window run: ./roger 127.0.0.1 8000
5. Repeat step 4 in other terminal windows with different port numbers RobotNum 
	times: 
	./roger 127.0.0.1 8001

The port numbers start from 8000 for the first player and increment by 1 for each new
player. This will start the simulator with RobotNum Rogers.
```
