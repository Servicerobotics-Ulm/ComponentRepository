<!--- This file is generated from the ComponentWebotsRobotArm.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentWebotsRobotArm Component

<img src="model/ComponentWebotsRobotArmComponentDefinition.jpg" alt="ComponentWebotsRobotArm-ComponentImage" width="1000">

*Component Short Description:* 

A robot arm (manipulator) in the webots simulator.

This component needs [OpenRave todo:add link to install instructions]() to be installed first.

### Example program:

Set the joint angels of the [robot arm](https://cyberbotics.com/doc/guide/ure). 

```
CommManipulatorObjects::CommManipulatorTrajectory trajectory;
// do these calls in this order only
// 1. set JOINT_ANGLES
trajectory.setFlag(CommManipulatorObjects::ManipulatorTrajectoryFlag::JOINT_ANGLES);
// 2. set number of points in trajectory
trajectory.set_trajectory_size(1);
// 3. set number of values of each point in the trajectory. must be 6 for the UR5e (6 motors)
trajectory.set_joint_count(6);
// 4. set points:
// time to move to point 0 is 0.3 seconds
trajectory.set_joint_time(0, 0.3);
// point 0, set all 6 motor rotations [radians]
trajectory.set_joint_angle(0, 0, 0.+231);
trajectory.set_joint_angle(0, 1, 0.-215);
trajectory.set_joint_angle(0, 2, 0.-541);
trajectory.set_joint_angle(0, 3, 0.+374);
trajectory.set_joint_angle(0, 4, 0.-121);
trajectory.set_joint_angle(0, 5, 0.+084);
COMP -> sendTrajectoryServiceOut -> send(trajectory);
```

Coordinate systems of robot arm and vacuum gripper (tool center point):
![image](CoordinateSystemRobotArm.png)

Set the pose (position and orientation) of the vacuum gripper in the coordinate system of the robot arm.
The robot arms 'UR5' or 'UR5e' or 'UR10e' with a vacuum gripper of length 0.11m are supported.

The orientation of the vacuum gripper's coordinate system is given by three [Euler Angles](https://web.mit.edu/2.05/www/Handout/HO2.PDF) called azimuth, elevation, roll using zyx convention (1. angle rotates around z-axis, 2. y, 3. x) 
* start with same coordinate system orientation as the robot arm (x/y/z-axis have same direction)
* rotate around blue z-axis by angle azimuth (in webots, you can click on the blue curved arrow and move the mouse to rotate objects)
* rotate around green y-axis by angle elevation (the y-axis could have been rotated by azimuth already)
* rotate around red x-axis by angle roll (the x-axis could have been rotated by azimuth or elevation already)

In below example, the robot arm coordinate system is copied and rotated around x-axis by -PI/2 (-90 degrees), the new y-axis and the vacuum gripper will point downwards, ready to pick things from above.

```
CommManipulatorObjects::CommManipulatorTrajectory trajectory;
trajectory.setFlag(CommManipulatorObjects::ManipulatorTrajectoryFlag::POSE_TCP);
double x, y, z, azimuth, elevation, roll;
x = 0.5; // 0.5m forward relative to robot arm
y = 0.3; // 0.3m left
z = 0.2; // 0.2m up
azimuth = 0.0;
elevation = 0.0;
roll = -M_PI/2; 
trajectory.set_pose_TCP_robot(x, y, z, azimuth, elevation, roll, 1);
COMP->sendTrajectoryServiceOut->send(trajectory);
```


### other similar components:

- [SmartURServer](../SmartURServer)
- [SmartURServerLegacy](../SmartURServerLegacy)
- [SmartGazeboManipulatorServer](../SmartGazeboManipulatorServer)



## Component-Datasheet Properties

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Component-Datasheet Properties</caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Property Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Property Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Property Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">MarketName</td>
<td style="border:1px solid black; padding: 5px;">ComponentWebotsRobotArm</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Supplier</td>
<td style="border:1px solid black; padding: 5px;">Servicerobotics Ulm</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Homepage</td>
<td style="border:1px solid black; padding: 5px;">https://wiki.servicerobotik-ulm.de/directory:collection</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Purpose</td>
<td style="border:1px solid black; padding: 5px;">The robot arm UR5e in the webots simulator.</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

## Component Ports

### trajectorySendServer

*Documentation:*
<p>Reads periodically a movement trajectory: CommManipulatorObjects.CommManipulatorTrajectory
</p>


### posePushServer

*Documentation:*
<p>Writes periodically position etc. : CommManipulatorObjects.CommMobileManipulatorState
</p>


### poseQueryServer

*Documentation:*
<p>Writes on request position etc. : CommManipulatorObjects.CommMobileManipulatorState
</p>


### manipulatorEventServiceOut

*Documentation:*
<p>Sends an event: CommManipulatorObjects:ManipulatorEvent
</p>


### baseStateServiceIn

*Documentation:*
<p>Reads periodically position etc. of an mobile robot: CommBasicObjects.CommBaseState
</p>


### ioQueryServer

*Documentation:*


### ioEventServer

*Documentation:*


### programQuery

*Documentation:*




## Component Parameters: ComponentWebotsRobotArmParams

### Internal Parameter: webots

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>webots</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>robotName</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">"UR5"</td>
<td style="border:1px solid black; padding: 5px;"><p>the webots robot arm name
</p></td>
</tr>
</table>

### ParameterSetInstance: ManipulatorParameter

#### Trigger Instance: MOVE_LINEAR

*Property:* active = **true**

*Documentation:*

#### Trigger Instance: MOVE_CIRCULAR

*Property:* active = **true**

*Documentation:*

#### Trigger Instance: SET_PCS

*Property:* active = **true**

*Documentation:*

#### Trigger Instance: CLEAR_PCS

*Property:* active = **true**

*Documentation:*

#### Trigger Instance: SET_TCP

*Property:* active = **true**

*Documentation:*

#### Trigger Instance: MOVE_PATH

*Property:* active = **true**

*Documentation:*

#### Trigger Instance: LOAD_PROGRAM

*Property:* active = **false**

*Documentation:*

#### Trigger Instance: START_PROGRAM

*Property:* active = **false**

*Documentation:*

### Internal Parameter: base

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>base</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>on_base</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">false</td>
<td style="border:1px solid black; padding: 5px;"><p><ul><li>on_base = true: robot arm is mounted on top of an mobile robot (base = mobile robot)</li>
 <li>on_base = false: robot arm is at a fixed position in the world (base = world)</li></ul>
</p></td>
</tr>
</table>

### Internal Parameter: manipulator

*Documentation:*
<p>the pose of the manipulator coordinate system relative to the base coordinate system
</p>

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>manipulator</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>verbose</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">true</td>
<td style="border:1px solid black; padding: 5px;"><p>print more debug information
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>x</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>x coordinate of the manipulator relative to the base [mm]
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>y</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>see x
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>z</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>see x
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>azimuth</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>rotation around z-axis
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>elevation</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>rotation around y-axis
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>roll</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>rotation around x-axis
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>tcp_velocity</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">1.0</td>
<td style="border:1px solid black; padding: 5px;"><p>velocity of TCP movement (ToolCenterPoint = end point of tool at robot arm) [m/s]
</p></td>
</tr>
</table>

