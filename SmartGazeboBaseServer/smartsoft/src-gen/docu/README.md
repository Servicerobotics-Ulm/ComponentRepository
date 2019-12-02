<!--- This file is generated from the SmartGazeboBaseServer.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# SmartGazeboBaseServer Component

<img src="model/SmartGazeboBaseServerComponentDefinition.jpg" alt="SmartGazeboBaseServer-ComponentImage" width="1000">

*Component Short Description:* The SmartGazeboBaseServer can be used to command a robot in a 3D environment using the Gazebo simulator.

## Component Documentation
<p></p>
<p> The SmartGazeboBaseServer can be used to command a robot in a 3D environment using the Gazebo simulator.
 It offers services for controlling the robot via SmartCDL or
 SmartJoystickNavigation by receiving v and omega values from SmartCDL or SmartJoystickNavigation.
 Those values are directly handed over to the Gazebo simulator in case of a differential drive.
 The communication between the SmartGazeboBaseServer and Gazebo is based on the communication lib of gazebo..
</p>
<p> GPL-License: includes Code from the Player Project.
</p>
<p> See also: http://playerstage.sourceforge.net/doc/Player-2.0.0/player/group__driver__amcl.html
</p>
<p></p>

## Component-Datasheet Properties

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Component-Datasheet Properties</caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Property Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Property Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Property Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">SpdxLicense</td>
<td style="border:1px solid black; padding: 5px;">LGPL-2.0-or-later</td>
<td style="border:1px solid black; padding: 5px;">https://spdx.org/licenses/LGPL-2.0-or-later.html</td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">TechnologyReadinessLevel</td>
<td style="border:1px solid black; padding: 5px;">TRL5</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Homepage</td>
<td style="border:1px solid black; padding: 5px;">http://servicerobotik-ulm.de/components</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Supplier</td>
<td style="border:1px solid black; padding: 5px;">Servicerobotics Ulm</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Purpose</td>
<td style="border:1px solid black; padding: 5px;">Simulation</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

## Component Ports

### NavVelServiceIn

*Documentation:*


### LocalizationUpdateServiceIn

*Documentation:*


### BaseSatateQueryAnsw

*Documentation:*


### LaserServiceOut

*Documentation:*


### BaseStateServiceOut

*Documentation:*




## Component Parameters: SmartGazeboBaseServerParams

### Internal Parameter: Settings

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>Settings</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>sendVelTopic</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">"~/robot/vel_cmd"</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>basePoseTopic</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">"~/robot/basePose"</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>baseVelTopic</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">"~/robot::robot::base_footprint"</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>laserTopic</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">"~/robot/robot/base_footprint/base_laser/scan"</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

