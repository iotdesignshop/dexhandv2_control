# dexhandv2_control
ROS 2 Controller for DexHand V2 Core Functions and Serial Communications

## Overview
This ROS2 package provides the interface between the DexHand V2 SDK and ROS 2. It is used to control one or more DexHand V2 devices connected to a host via USB. 

The package offers two nodes, and mechanics for integrating with the DexHand V2 hardware via ROS2:

- **Native Messaging (via the native_messaging ROS 2 Node)**: This node provides low level access to the DexHand SDK remapping native C++ SDK messaging into ROS2 topics for controlling the hand. This is the fastest, and lowest level interface to the DexHand. However, it does not offer a lot of additional features beyond the exchange of position and status messages between the hardware and software.
- **High Level Control (via the high_level_control ROS 2 Node**): This node abstracts the functions of the hand a bit more and provides access to virtual "Servo" objects which represent the motors and state in the hand. Instead of exchanging messages directly with the firmware, you can set and read data from the Servo objects. It also offers an approximate "joint angle controller" which will map ROS 2 joint_state messages to the hardware. Note that calibration between these angles, simulation, and actual hardware is a bit of a study onto itself, but the node does provide the framework to get up and running, with a way to have the DexHand responding to joint_state messages.

**NOTE: You should not attempt to run both nodes at the same time. The intent of the nodes provided in the package is that one or the other will have access to the DexHand hardware and USB stream. Running both nodes will cause conflicts. Pick one. Play one.**

## Important Things to Know

### Virtual E-Stop and the DexHand Reset Function

As is common to the native Python and C++ SDK's as well, the DexHand V2 hardware always boots into a safe mode in which it is e-stopped at the SDK level. In order to start the message exchange with the hand **a reset command must be issued**. While this can be a cause of confusion "Why isn't the hand moving?", it's really the only safe way to boot the hardware. We apologize if this catches you off guard a couple of times as you get used to controlling the DexHand. Eventually, it will be part of your routine.

**The DexHand always boots in virtual e-stop and will not move until a reset command is issued**


## Common functions and Procedures Related to Both Native Messaging and High Level Control

Some functions of the DexHand are common to both nodes provided by the package. They are related to the enumeration and control functions of the hand, such as the **reset function** noted above.





## Usage of Native Messaging (native_messaging) ROS 2 Node
