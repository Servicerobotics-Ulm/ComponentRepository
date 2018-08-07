<!--- This file is generated from the SmartPioneerBaseServer.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# SmartPioneerBaseServer Component

![SmartPioneerBaseServer-ComponentImage](https://github.com/Servicerobotics-Ulm/ComponentRepository/blob/master/SmartPioneerBaseServer/model/SmartPioneerBaseServerComponentDefinition.jpg)

The SmartPioneerBaseServer makes P2OS-based robot platforms available. It handles all the communication with the hardware. It offers several services for controlling the robot, such as sending navigation commands to the base and providing access to the robot's odometry. Position updates can be sent to the component to overcome odometry failures.

Note: This component is used in Tutorials (e.g. Lesson 1).

GPL-License: includes Code from the Player Project.

| Metaelement | Documentation |
|-------------|---------------|
| License | GPL |
| Hardware Requirements | P2OS-based platforms. Currently supported: P3DX, P3DXSH, P3ATSH. Others can simply be added. |
| Purpose | Hardware-Driver |


## Coordination Port CoordinationPort


### States


| MainState Name | MainState Description |
|----------------|-----------------------|

### DynamicWiring


### Parameter

Accept parameters at runtime. See section Parameters.

## Service Ports

### LocalizationUpdate

Port to send corrections of base pose to overcome the odometry failure. Accepts a pair of an old uncorrected pose and a new corrected pose. The deviation between these two poses is applied to correct the current pose of the robot.

### NavVelIn

Send new navigation velocity commands v and omega to hardware base. The values are thresholded by the min and max values specified in the ini file before being sent.
						Note that the base will perform an emergency stop if no new velocity command was sent within the timeout specified in firmware.

### BasePositionOut

Push the base state containing current information about the robot's velocity, pose, raw pose. Should be used when a continuous base pose is required. For example, the SmartLaserLMS200Server uses the latest base pose (received continuously) to stamp the laser scan with the robot's pose at the time the scan was recorded.

### BaseStateQueryServer

Query port to request the base state. Analogous to basePositionPushTimedServer, but a query service. Should be used when the base pose is needed sporadically, for example by a behavior component which explicitly needs to query the base pose from time to time.


## Component Parameters SmartPioneerBaseServerParams

### InternalParameter Robot

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| enable_motors | Boolean | Enable (true) or disable (false) motors at startup. Defines the state of the base' 'motors'-button on startup. |
| enable_sonar | Boolean | Enable (true) or disable (false) sonar at startup. |
| maxVel | Int32 | Set maximum translation velocity of robot [mm/s]. |
| maxVelAcc | Int32 | Set maximum translation acceleration of robot [mm/s^2]. |
| maxVelDecel | Int32 | Set maximum translation deceleration of robot [mm/s^2]. Negative value. |
| maxRotVel | Int32 | Set maximum rotation velocity of robot [deg/s]. |
| maxRotVelAcc | Int32 | Set maximum rotation acceleration of robot [deg/s^2]. |
| maxRotVelDecel | Int32 | Set maximum rotation deceleration of robot [deg/s^2]. Negative value. |
| serialport | String | Device name to access Pioneer Base, e.g. /dev/ttyS0 |
| robotType | String | Type of pioneer platform. Currently supported: p3dx, p3dxsh, p3atsh. |

### ParameterSetInstance BaseParams

#### TriggerInstance BASE_RESET

active = false

Reset the connection to the base at runtime. The serial connection is closed and reopened. All estimated positions are set to zero.

#### TriggerInstance BASE_SONAR

active = false


