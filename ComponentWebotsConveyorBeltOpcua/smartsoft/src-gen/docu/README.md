<!--- This file is generated from the ComponentWebotsConveyorBeltOpcua.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentWebotsConveyorBeltOpcua Component

<img src="model/ComponentWebotsConveyorBeltOpcuaComponentDefinition.jpg" alt="ComponentWebotsConveyorBeltOpcua-ComponentImage" width="1000">

*Component Short Description:* 

ComponentWebotsConveyorBeltOpcua transfers a stacking container (box) between an mobile robot and an production station.

This component needs [Open62541CppWrapper](https://github.com/Servicerobotics-Ulm/Open62541CppWrapper) to be installed first.

There is a demo to see this component in action: start SystemWebotsNavMpsDockingOPCUA, go to the window with 'ComponentTCLSequencer' and enter a number.

There are two kinds of production stations:
- an active station with an conveyor belt and an OPC UA server for communication
- an passive station with an roller conveyor

The movement (docking) of the robot to the production station is done by [ComponentWebotsMpsDocking](../ComponentWebotsMpsDocking).

Transfer commands are given by setting the ComponentMode:
- **load** moves the container from an production station to the mobile robot automatically (loads the robot)  
- **unload** moves the container from the mobile robot to the production station automatically
- **manualload** is like **load**, but manually
- **manualunload** is like **unload**, but manually
- **signalerror** shows an error (see **red** light below)

Automatic load/unload:
- if an box should be loaded but there is already one on top of the mobile robot, send event CONVEYER_BELT_LOAD_ERROR_BOX_ADREADY_PRSESENT and stop with an error
- if an box should be unloaded but there was no one, send event CONVEYER_BELT_UNLOAD_ERROR_NO_BOX and stop with an error
- send event CONVEYER_BELT_LOAD_NOT_DONE/CONVEYER_BELT_UNLOAD_NOT_DONE
- turn on conveyor belt motors
- if the box is detected at the target position, send event CONVEYER_BELT_LOAD_DONE/CONVEYER_BELT_UNLOAD_DONE and stop with success
- after too many seconds without success or the production station reported an error, send event CONVEYER_BELT_LOAD_ERROR_NO_BOX_LOADED/CONVEYER_BELT_UNLOAD_ERROR_NO_BOX and stop with an error

Meaning of the signal lights (on the mobile robot or production station):
- **red**: an error occured, stop all until an human fixed the problem and pressed the enter key in the webots graphics window or the state changed (flashing red light=SignalError, constant red light=ManualLoad/ManualUnload)
- **yellow**: warning, conveyor belts are turned on to transfer the stacking container
- **green**: the stacking container is detected



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
<td style="border:1px solid black; padding: 5px;">ComponentWebotsConveyorBeltOpcua</td>
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
<td style="border:1px solid black; padding: 5px;">transfers a stacking container between an mobile robot and an production station by conveyor belts</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

## Component Ports

### TrafficLightsServiceIn

*Documentation:*
<p>Reads peridically to turn off/on red/yellow/green lights on the mobile robot: CommBasicObjects.CommTrafficLights
</p>


### RobotinoConveyerBeltEventOut

*Documentation:*
<p>Sends an event: CommRobotinoObjects::RobotinoConveyerBeltEventType
</p>




## Component Parameters: ComponentWebotsConveyorBeltOpcua

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
<td style="border:1px solid black; padding: 5px;">"RobotinoConveyorBelt"</td>
<td style="border:1px solid black; padding: 5px;"><p>the webots conveyor belt device must be a children of an extra robot with this name
</p></td>
</tr>
</table>

### ParameterSetInstance: RobotinoConveyerParameter

#### Parameter Instance: SetStationID

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Parameter-Instance <b>SetStationID</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>id</b></td>
<td style="border:1px solid black; padding: 5px;">Int16</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"></td>
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
<td style="border:1px solid black; padding: 5px;"><b>belt_time_out_sec</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">10</td>
<td style="border:1px solid black; padding: 5px;"><p>stop with error if load/unload to an active station takes more than this time [seconds]
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>ignore_station_communication_unload_time_sec</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">4</td>
<td style="border:1px solid black; padding: 5px;"><p>stop with error if load/unload to an passive station takes more than this time [seconds]
</p></td>
</tr>
</table>

