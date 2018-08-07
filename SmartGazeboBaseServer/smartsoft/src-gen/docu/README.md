<!--- This file is generated from the SmartGazeboBaseServer.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# SmartGazeboBaseServer Component

![SmartGazeboBaseServer-ComponentImage](https://github.com/Servicerobotics-Ulm/ComponentRepository/blob/master/SmartGazeboBaseServer/model/SmartGazeboBaseServerComponentDefinition.jpg)

The SmartGazeboBaseServer can be used to command a robot in a 3D environment using the Gazebo simulator. It offers services for controlling the robot via SmartCDL or SmartJoystickNavigation by receiving v and omega values from SmartCDL or SmartJoystickNavigation. Those values are directly handed over to the Gazebo simulator in case of a differential drive. The communication between the SmartGazeboBaseServer and Gazebo is based on the communication lib of gazebo.

See also: http://servicerobotik-ulm.de

| Metaelement | Documentation |
|-------------|---------------|
| License | LGPL |
| Hardware Requirements | - |
| Purpose | Simulation |



## Service Ports

### LocalizationUpdateServiceIn

Port to receive corrections of base pose to overcome the odometry failure. Accepts a pair of old uncorrected pose and new corrected pose. The deviation between these two poses is applied to correct the current pose of the robot.

### NavVelServiceIn

Send new navigation velocity commands v and omega to the Gazebo Simulator.

### BaseStateServiceOut

Push the base state containing current information about the robot's velocity, pose and raw pose. Should be used when a continuous base pose is required.

### BaseSatateQueryAnsw

Query port to request the base state. Analogous to basePositionPushTimedServer, but a query service. Should be used when the base pose is needed sporadically, for example by a behavior component which explicitly needs to query the base pose from time to time.

### LaserServiceOut

Push latest laser scan. The rate with which the server pushes is the same as in the basePositionPushTimedServer.


## Component Parameters SmartGazeboBaseServerParams

### InternalParameter Settings

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| sendVelTopic | String |  |
| basePoseTopic | String |  |
| baseVelTopic | String |  |
| laserTopic | String |  |

