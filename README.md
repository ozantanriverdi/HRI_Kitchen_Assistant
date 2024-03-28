# Kitchen Assistant

This project has been developed as part of the <b>Human-Robot Interaction Practical</b> at LMU. The repository covers an implementation of a kitchen assistant robot functionalities. The kitchen assistant is able to perform the task of adding spice to a dish on user command. To achieve this, the users gives a voice command starting with the keyword 'assistant' and specifies the type and the amount of the desired spice. The robot then detects the location of the desired spice via its computer vision model using april tag. Using the gripper, the spice is brought above the target location below which a weight sensor has been placed. The scale values are read via an Arduino UNO board. Using a PID Controller and a stream of scale values, a real-time feedback loop is provided to the controller component of the robot and the precise amount of spice gets tossed to the target location.

In this documentation, hardware and software requirements to run the project and how the project should be run will documented.


## Table of Contents
- [Hardware Specifications](#hardware-specifications)
- [Software Specifications](#software-specifications)
- [Environment Specification](#environment-specification)
- [Project Structure](#project-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Sources](#sources)
- [Contributors](#contributors)


## Hardware Specifications
1. Robot: [ER myCobot 280 M5stack](https://www.elephantrobotics.com/en/mycobot-en/)
2. Scale: [Mini Scales Unit (HX711)](https://shop.m5stack.com/products/mini-scales-unit-hx711)
3. Camera: Microsoft LifeTime3000
4. Microphone: Default system microphone
5. Microcontroller Board: [Arduino UNO](https://store.arduino.cc/products/arduino-uno-rev3)
6. NVIDIA CUDA GPU


## Software Specifications
1. ROS Noetic
2. Python 3.9
3. Arduino IDE 1.8.19
4. Ubuntu 20.04

## Physical Environment Specification
1. Fasten the robot to the table with its screen looking left.
2. Place the camera so that it has a clear view over the table the robot is going to operate on. Make sure all the tags are visible to the camera.
3. Place the reference tag in view of the camera and adjust the `REFERENCE_ROBOT_DIFF` and the `SPICE_TAG_DIFF` parameters in `scan.py` accordingly.
4. Paste the april tags under the corresponding spice bottles.
5. Make an empty run to see where the target location is, and then put the scale and the target cup on this location.

## Project Structure
- The project is structured in 6 Catkin packages where 5 of them were implemented as part of this project.
- Arduino IDE is used to run the program to read the scale values on the Arduino UNO board.
- Thus, this repository contains scripts for each of these 5 Catkin packages, plus the code which should be run on the Arduino Board.
### Catkin Packages
- `transcriber`: speech-to-text functionalities
- `scanner`: computer vision functionalities
- `input_handler`: combining voice and camera input and passing the processed input to the control component
- `scale`: scale reading functionalities
- `control`: robot movement control functionalities

## Installation


1. Clone the repository:

```shell
$ git clone git@github.com:ozantanriverdi/HRI_Kitchen_Assistant.git
```


2. Navigate to the project directory:
```shell
$ cd HRI_Kitchen_Assistant
```


3. Create and build a catkin workspace:
```shell
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```


4. Clone and build the [mycobot_ros](https://github.com/elephantrobotics/mycobot_ros) repository under this workspace


5. There are 5 Catkin packages implemented in this repository. For each Catkin package, create an empty Catkin package under the Catkin workspace (preferably, with the same names we introduced above):
```shell
$ catkin_create_pkg <name> rospy
```

6. For each of the 5 Catkin packages, create a `scripts` sub-directory under `src`:
```shell
$ cd src
$ mkdir scripts
```

7. Go the repository directory and copy the scripts under each subfolder into `scripts` sub-directoy under the corresponding Catkin packages. Perform this for every Catkin package.

8. Copy the `Data` folder containing the configurations to the Catkin workspace

9. In each `scripts` sub-directory there should be one python file whose name ends with "_node.py", i.e. "scale_node.py". These are where the ROS nodes have been initialized and ROS should be able to run time. Make these executable with:
```shell
$ chmod +x scale_node.py
```

10. Start the Arduino IDE and select the board. Copy and paste the code from `scale_arduino.txt`. Plug the sensor to the Arduino board following the [instructions](https://www.circuito.io/blog/arduino-uno-pinout/) and sensor manuals (see above). Select the correct Port under Tools -> Port and first verify the code and then upload it, using the icons top left. If you select `Serial Monitor` located top right, you should be able to see the scale readings, but make sure to have `Serial Monitor` closed when running the Python scripts because both programs can't access the port at the same time.

11. If you use a different camera, make sure to configure device-specific values so that accurate distance measurements can be performed.

12. Plug in the robot and if during the usage the error `serial.serialutil.SerialException` comes up, you can free the error causing port via:
```shell
$ sudo chmod 666 /dev/ttyACM0
```

13. According to the camera model to be used, change the `CAMERA_PARAMS` in `scan.py`



## Usage

1. Start the Arduino IDE and make sure readings are received via `Serial Monitor`. Under Tools -> Ports, detect which port is listed even if that's not occupied by the Arduino Board. That's the port of the robot and make sure this port specification is correctly configured in `controller_node.py`

2. Start a ROS master
```shell
$ roscore
```

3. In a new terminal, run the launch script of mycobot moveit
```shell
$ roslaunch mycobot_280_moveit mycobot_moveit.launch
```
4.  In a new terminal, run the mycobot node
```shell
$ rosrun mycobot_280_moveit sync_plan.py _port:=<robot_port> _baud:=115200
```
5.  In a new terminal, run the `transcriber` node
```shell
$ rosrun transcriber node.py
```
6.  In a new terminal, run the `scanner` node
```shell
$ rosrun scanner node.py
```
7.  In a new terminal, run the `input_handler` node
```shell
$ rosrun input_handler node.py
```
8.  In a new terminal, run the `scale` node
```shell
$ rosrun scale scale_node.py
```
9.  In a new terminal, run the `controller` node
```shell
$ rosrun controller controller_node.py
```
10. You can now give a command starting with `assistant` keyword
## Sources
- https://docs.elephantrobotics.com/docs/gitbook-en/
- https://github.com/m5stack/M5Unit-Miniscale
- https://wiki.ros.org/ROS/Tutorials
- https://github.com/elephantrobotics/mycobot_ros
- https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
- https://www.circuito.io/blog/arduino-uno-pinout/
- https://youtu.be/wfDJAYTMTdk?si=qOQDs1f801KfPl_Z
- https://www.ni.com/en/shop/labview/pid-theory-explained.html
- https://github.com/Vaibhavs10/insanely-fast-whisper
- https://answers.ros.org/question/304770/moveit-joint-space-constraints-in-python/
- https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

## Contributors
- Oguzhan Cesur
- Sarp Cagin Erdogan
- Ozan Ilgin Tanriverdi