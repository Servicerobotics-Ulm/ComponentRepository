<!--- This file is generated from the ComponentWebotsMpsDocking.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentWebotsMpsDocking Component

<img src="model/ComponentWebotsMpsDockingComponentDefinition.jpg" alt="ComponentWebotsMpsDocking-ComponentImage" width="1000">

*Component Short Description:* A short description for the ComponentWebotsMpsDocking datasheet


## Component-Datasheet Properties

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Component-Datasheet Properties</caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Property Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Property Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Property Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Supplier</td>
<td style="border:1px solid black; padding: 5px;">No supplier specified</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Homepage</td>
<td style="border:1px solid black; padding: 5px;">http://www.example.com</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;">Purpose</td>
<td style="border:1px solid black; padding: 5px;">Example</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

## Component Ports

### NavigationVelocityServiceOut

*Documentation:*


### BaseStateServiceIn

*Documentation:*


### LaserServiceIn

*Documentation:*


### TrafficLightsServiceOut

*Documentation:*


### RobotDockingEventServiceOut

*Documentation:*




## Component Parameters: ComponentWebotsMpsDocking

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
<td style="border:1px solid black; padding: 5px;">"MpsDocking"</td>
<td style="border:1px solid black; padding: 5px;"><p>the docking robotino should have an Supervisor with this name inside
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>stationName</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">["MPS0", "MPS1", "MPS2", "MPS3"]</td>
<td style="border:1px solid black; padding: 5px;"><p>the DEF names of the docking stations in webots
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>maxDistanceToDockingPoint</b></td>
<td style="border:1px solid black; padding: 5px;">Double</td>
<td style="border:1px solid black; padding: 5px;">2.0</td>
<td style="border:1px solid black; padding: 5px;"><p>dock to stations only if distance to the docking point is less than this value (in meters)
</p></td>
</tr>
</table>

