<!--- This file is generated from the ComponentWebotsMobileRobot.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentWebotsMobileRobot Component

<img src="model/ComponentWebotsMobileRobotComponentDefinition.jpg" alt="ComponentWebotsMobileRobot-ComponentImage" width="1000">

*Component Short Description:* 

A generic driver for robots in the webots simulator with differential or omnidirectional drive.

How a new robot can be added to Webots:
* Add a new Robot, set its controller to '&lt;extern&gt;'.
* The robots name must set the same in Webots and here.

The coordinate system of the robot should be:
* x=front, y=left (x=y=0 is at the turning point of the robot (center between wheels))  
* z=up (z=0 is at floor level)

If a robot is not like this, add a Transformation node to him: 
* Add it into children of the robot 
* set DEF 'CoordinateSystem*Abc_def*' in the Transformation node if the robots name would be *Abc def*
* now move and rotate the Transformation node so its coordinate system is like it should be




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
<td style="border:1px solid black; padding: 5px;">Webots mobile robot</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Supplier</td>
<td style="border:1px solid black; padding: 5px;">Servicerobotics Ulm</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Homepage</td>
<td style="border:1px solid black; padding: 5px;">http://servicerobotik-ulm.de/components</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Purpose</td>
<td style="border:1px solid black; padding: 5px;">Webots mobile robot</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

## Component Ports

### NavigationVelocityServiceIn

*Documentation:*


### LocalizationEventServiceIn

*Documentation:*


### BaseStateQueryServiceAnsw

*Documentation:*


### BaseStateServiceOut

*Documentation:*


### LocalizationUpdateServiceIn

*Documentation:*




## Component Parameters: ComponentWebotsMobileRobotParams

### Internal Parameter: OdometryRandomError

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>OdometryRandomError</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>varianceOfDistancePerMeter</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0.0025</td>
<td style="border:1px solid black; padding: 5px;"><p>e.g. 0.05m * 0.05m / 1m = 0.0025 m (after traveling 1m, distance error has standard deviation of 0.05m)
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>varianceOfHeadingPerRadians</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0.001212</td>
<td style="border:1px solid black; padding: 5px;"><p>e.g. (5°*5°)/360° /180°*pi = 0.001212 (after rotating 360 degrees, heading error has standard deviation of 5 degrees)
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>varianceOfHeadingPerMeter</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0.001218</td>
<td style="border:1px solid black; padding: 5px;"><p>e.g. (2°/180°*pi)^2/1m  = 0.001218 (after traveling 1m, heading error has standard deviation of 2 degrees)
</p></td>
</tr>
</table>

### Internal Parameter: Webots

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>Webots</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>robotName</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">"Robotino 3"</td>
<td style="border:1px solid black; padding: 5px;"><p>name of robot
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>motorName</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">["wheel0_joint", "wheel1_joint", "wheel2_joint"]</td>
<td style="border:1px solid black; padding: 5px;"><p>name of motors of wheels
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>radius</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">[0.063, 0.063, 0.063]</td>
<td style="border:1px solid black; padding: 5px;"><p>radius of wheels in m
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>distanceToRobotCentre</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">[-0.1826, -0.1826, -0.1826]</td>
<td style="border:1px solid black; padding: 5px;"><p>distance of a wheel to the turning point (center) of the robot
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>heading</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">[90.0, 210.0, 330.0]</td>
<td style="border:1px solid black; padding: 5px;"><p>the heading of the wheels, for differential drives set these to 0 [degrees]
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>maxAcceleration</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">[10.0, 6.0, 40.0]</td>
<td style="border:1px solid black; padding: 5px;"><p>the max. acceleration front [m/s], sideways [m/s], rotation [radians/s]
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>keyboardControl</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">true</td>
<td style="border:1px solid black; padding: 5px;"><p>if true, the robot can be moved by arrow or WASD keys (QE sidways), press space key to stop
</p></td>
</tr>
</table>

### Internal Parameter: Robot

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>Robot</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>maxVelX</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">1.0</td>
<td style="border:1px solid black; padding: 5px;"><p>max. speed (forward) in m/s
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>maxVelY</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">1.0</td>
<td style="border:1px solid black; padding: 5px;"><p>max. speed sideways (left) in m/s (only omnidirection drive as in Robotino3)
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>maxRotVel</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">4.0</td>
<td style="border:1px solid black; padding: 5px;"><p>maximum rotation velocity of robot in radians/s
</p></td>
</tr>
</table>

### Internal Parameter: General

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>General</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>verbose</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">false</td>
<td style="border:1px solid black; padding: 5px;"><p>if set to true, more information is printed in the console
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>useLocalizationEvent</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">true</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>writePoseFile</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">false</td>
<td style="border:1px solid black; padding: 5px;"><p>if set to true, the robot position is read/saved in poseFileName
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>poseFileName</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">"/tmp/lastRobotPose.txt"</td>
<td style="border:1px solid black; padding: 5px;"><p>see writePoseFile
</p></td>
</tr>
</table>

