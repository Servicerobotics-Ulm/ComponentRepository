<!--- This file is generated from the ComponentPlayerStageSimulator.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentPlayerStageSimulator Component

![ComponentPlayerStageSimulator-ComponentImage](model/ComponentPlayerStageSimulatorComponentDefinition.jpg)

The SmartPlayerStageSimulator simulates a robot in a 2D bitmapped environment using Player/Stage. 
It offers several services for controlling the robot, such as sending navigation commands, providing access to the robot's odometry and laser scans. 
SmartPlayerStage simulator can replace both SmartPioneerBaseServer and SmartLaserLMS200 for simulation purposes.

Tested on Ubuntu 12.04 LTS with player-3.0.2 and Stage-3.2.2, built from sources.

Note: This component is used in Tutorials (e.g. Lesson 1).

GPL-License: includes Code from the Player Project.

| Metaelement | Documentation |
|-------------|---------------|
| License | GPL |
| Hardware Requirements | - |
| Purpose | Simulation |



## Service Ports

### LocalizationUpdateServiceIn

Port to send corrections of base pose to overcome the odometry failure. 
			Accepts a pair of old uncorrected pose and new corrected pose. 
			The deviation between these two poses is applied to correct the current pose of the robot.

### NavigationVelocityServiceIn

Send new navigation velocity commands v and omega to hardware base.

### BaseStateServiceOut

Push the base state containing current information about the robot's velocity, pose and raw pose. 
			Should be used when a continuous base pose is required.

### BaseStateAnswerer

Query port to request the base state. Analogous to basePositionPushTimedServer, but a query service. 
			Should be used when the base pose is needed sporadically, 
			for example by a behavior component which explicitly needs to query the base pose from time to time.

### LaserServiceOut

Push latest laser scan. The rate with which the server pushes is the same as in the basePositionPushTimedServer.

### BatteryEventServiceOut

Battery State event Service Yet to be implemented


## Component Parameters ComponentPlayerStageSimulatorParams

