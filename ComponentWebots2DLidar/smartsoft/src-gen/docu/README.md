<!--- This file is generated from the ComponentWebots2DLidar.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentWebots2DLidar Component

<img src="model/ComponentWebots2DLidarComponentDefinition.jpg" alt="ComponentWebots2DLidar-ComponentImage" width="1000">

*Component Short Description:* 

ComponentWebots2DLidar connects to an lidar (laser scanner) in the webots simulator, measuring distances with laser rays.
 
The scans are made each Webots timeStep, at the same time (see Time of flight (ToF)). The rays are ordered horizontal from right to left. Distances are in measured in mm.

Note that it is possible to reduce the data size by removing distance values with are not within a min or max distance or by reducing the resolution (angle between two rays) or the opening_angle (angle of all rays in the reduced data set).

How a new lidar can be added to Webots:
* Add a new Robot, set its controller to '&lt;extern&gt;', its coordinate system should be x=front, y=left, z=up of the lidar rays.
* The robots name must set the same in Webots and here.
* Add the lidar in children of the robot, rotate it to match the coordinate system above.

example code:

```cpp
CommBasicObjects::CommMobileLaserScan scan;
if(this->laserServiceInGetUpdate(scan) != Smart::SMART_OK) (some error handling here)
int count = scan.get_scan_size();
for (int i = 0; i < count; ++i) {
	// scan.get_scan_angle is between 0 and 2*pi, 0=front of lidar
	double angle = scan.get_scan_angle(i) / M_PI * 180;
	// angle should be between -180 and +180 degrees, 0=front of lidar
	if( angle > 180 )
		angle -= 360;
	double distance = scan.get_scan_distance(i);
    ...
}
```
  


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
<td style="border:1px solid black; padding: 5px;">ComponentWebots2DLidar</td>
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
<td style="border:1px solid black; padding: 5px;">Example</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

## Component Ports

### BaseStateServiceIn

*Documentation:*


### LaserServiceOut

*Documentation:*


### LaserQueryServiceAnsw

*Documentation:*




## Component Parameters: ComponentWebots2DLidarParams

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
<td style="border:1px solid black; padding: 5px;">"Lidar"</td>
<td style="border:1px solid black; padding: 5px;"><p>the webots lidar must be a children of an extra robot with this name
</p></td>
</tr>
</table>

### Internal Parameter: scanner

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>scanner</b></caption>
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
<td style="border:1px solid black; padding: 5px;"><p>more debug information if set to true
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>on_turret</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">false</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>x</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>position of the lidar relative to the robot in mm
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>y</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>position of the lidar relative to the robot in mm
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>z</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>position of the lidar relative to the robot in mm
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>azimuth</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>position of the lidar relative to the robot in radians
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>elevation</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>position of the lidar relative to the robot in radians
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>roll</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>position of the lidar relative to the robot in radians
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>min_range</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">10</td>
<td style="border:1px solid black; padding: 5px;"><p>if one laser scan distance value is less than this, it is removed from the scan, unit mm. may differ from webots minRange
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>max_range</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">20000</td>
<td style="border:1px solid black; padding: 5px;"><p>if one laser scan distance value is more than this, it is removed from the scan, unit mm. may differ from webots maxRange
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>opening_angle</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">270</td>
<td style="border:1px solid black; padding: 5px;"><p>only the lidar rays within this angle are kept in a scan, unit degrees, may be less than webots fieldOfView
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>resolution</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0.5</td>
<td style="border:1px solid black; padding: 5px;"><p>unit degrees, the angle between two lidar rays, should be set to webots fieldOfView/Pi*180/(horizontalResolution-1)
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>length_unit</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">1</td>
<td style="border:1px solid black; padding: 5px;"><p>the length unit of distances (values are 16 bit only, so length_unit=1 means a max. distance of 65535 mm)
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>frequency</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">50</td>
<td style="border:1px solid black; padding: 5px;"><p>not used yet, a webots lidar updates every WorldInfo.basicTimeStep ms (see webots timeStep)
</p></td>
</tr>
</table>

### Internal Parameter: base_manipulator

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>base_manipulator</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>on_base</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">true</td>
<td style="border:1px solid black; padding: 5px;"><p>if set to false, the baseState is not added to the scan data object
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>x</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>y</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>z</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>base_a</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>steer_a</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>turret_a</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

### Internal Parameter: services

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>services</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>activate_push_newest</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">true</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>active_push_timed</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">true</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

