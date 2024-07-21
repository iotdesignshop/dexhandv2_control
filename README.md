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

As is common to the native Python and C++ SDK's as well, the DexHand V2 hardware always boots into a safe mode in which it is e-stopped at the SDK level. In order to start the message exchange with the hand **a reset command must be issued**. Although this can be a cause of confusion: aka, "Why isn't the hand moving?" it's really the only safe way to boot the hardware. We apologize if this catches you off guard a couple of times as you get used to controlling the DexHand. Eventually, it will be part of your routine.

**The DexHand always boots in virtual e-stop and will not move until a reset command is issued**


## Common functions and Procedures Related to Both Native Messaging and High Level Control

Some functions of the DexHand are common to both nodes provided by the package. They are related to the enumeration and control functions of the hand, such as the **reset function** noted above.


### Services

#### Reset Hand
```/dexhandv2/reset```

Because no further processing will occur until the reset function is called, we cover it first. **Note: the DexHand hardware will not begin processing messages and commands until it is reset after boot.** This is a safety function so that the hand does not start to move unexpectedly without intentional control. Once the hand has been reset, it will publish and subscribe to a number of control and status messages which we discuss in the documentation below. 

To reset a hand using the ROS 2 CLI, you can issue the following command:
```
ros2 service call /dexhandv2/reset dexhandv2_control/srv/Reset "{id: <DexHand Serial# ID>}"
```
Where the id is found from the **/dexhandv2/discovered_hands** topic (covered below), or from the documentation that came with your hand. 

Once you call the service, you should see confirmation that the reset was successful as follows:
```
> ros2 service call /dexhandv2/reset dexhandv2_control/srv/Reset "{id: E6616408438E5D29}"

requester: making request: dexhandv2_control.srv.Reset_Request(id='E6616408438E5D29')

response:
dexhandv2_control.srv.Reset_Response(success=True)
```

Once the hand has been reset, additional information about the hand will be available in the topics discussed below.


### Publishers and Topics

#### Hand Enumeration
```/dexhandv2/discovered_hands```

This topic is used to determine which DexHand devices have been discovered by the node. It's primary use case is to uncover the serial numbers of the hands connected to the machine (in case you don't already know them) as most subsequent commands use these ID's to refer to the different hands plugged into the PC when publishing or subscribing to messages. This is the only topic that publishes information prior to the [reset_hand](#reset-hand) service call above. It can be useful if you don't know the ID of your DexHand device(s).

If you run:
```
ros2 topic echo /dexhandv2/discovered_hands
```

You should see output similar to this:
```
hands:
- id: E6616408438E5D29
  port: /dev/ttyACM0
  manufacturer: DexHand
  product: DexHandV2-1C
---
```

The _id:_ field indicates the serial number identifier of the device.

### 








## Usage of Native Messaging (native_messaging) ROS 2 Node
