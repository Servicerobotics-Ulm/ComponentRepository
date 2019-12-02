<!--- This file is generated from the SmartMapperGridMap.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# SmartMapperGridMap Component

<img src="model/SmartMapperGridMapComponentDefinition.jpg" alt="SmartMapperGridMap-ComponentImage" width="1000">

*Component Short Description:* The SmartMapperGridMap provides mapping services based on occupancy grid maps.

## Component Documentation
<p></p>
<p> The SmartMapperGridMap provides mapping services based on occupancy grid maps.
 Laser scans are taken for building a current and a longterm map.
</p>
<p> The current map represents the latest environment of the robot.
 It can be preoccupied with grids of the longterm map and can be used for path planning e.g. with SmartPlannerBreadthFirstSearch.
 It contains either occupied cells or free cells (binary). An optional obstacle growing can be applied to the current map.
</p>
<p> The longterm map holds cell values from 0 to 255. Values from 0 to 127 denote the traversability where 0 is completely free.
 Values from 128 to 255 are special values: Obstacles are marked with 128,
 cells occupied by obstacle growing with 129 and undeletable grids are marked with 130.
 The cell values can be accumulated over time to represent the environment over a longer period.
</p>
<p> Both grid maps can be saved to XPM and XML and loaded from XML files.
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
<td style="border:1px solid black; padding: 5px;">Mapping</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

## Component Ports

### LaserServiceIn

*Documentation:*


### CurrQueryServer

*Documentation:*


### LtmQueryServer

*Documentation:*


### CurrMapOut

*Documentation:*




## Component Parameters: SmartMapperGridMapParams

### Internal Parameter: CurrentMap

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>CurrentMap</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>interval</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">10</td>
<td style="border:1px solid black; padding: 5px;"><p>Specify the interval for considering laser scans to build the current map. The component will process every nth laser scan. The actual interval for building the current map thus depends on the value of this setting and on the rate of the laser scan for pushing new scans. This can be used to slow down the building of the current map.
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>growing</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">"star16"</td>
<td style="border:1px solid black; padding: 5px;"><p>Obstacle growing mode. Obstacle can be grown in circles with 8 (circle8) or 16 (circle16) (circle40) (circle32)  occupied cells around the obstacle or as a star of 8 (star8) or 16 (star16) occupied cells around the obstacle. Obstacle growing can also be deactivated with value no.
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>xsize</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">20000</td>
<td style="border:1px solid black; padding: 5px;"><p>Initial value for x size of current map. [mm].
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>ysize</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">20000</td>
<td style="border:1px solid black; padding: 5px;"><p>Initial value for y size of current map. [mm].
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>xpos</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">-10000</td>
<td style="border:1px solid black; padding: 5px;"><p>Initial value for x offset of current map relative to world. [mm].
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>ypos</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">-10000</td>
<td style="border:1px solid black; padding: 5px;"><p>Initial value for y offset of current map relative to world. [mm].
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>id</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>Initial id of the current map. Can later be set with parameter CURPARAMETER. The id ?id will be assigned to the current map to synchronize components, for example with SmartPlannerBreadthFirstSearch and SmartCdlServer.
</p></td>
</tr>
</table>

### Internal Parameter: LtmMap

*Documentation:*

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Internal Parameter <b>LtmMap</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>kalman</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">100</td>
<td style="border:1px solid black; padding: 5px;"><p>Adaption rate for kalman filter weighting. [0-255].
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>xsize</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">20000</td>
<td style="border:1px solid black; padding: 5px;"><p>Initial value for x size of longterm map. [mm].
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>ysize</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">20000</td>
<td style="border:1px solid black; padding: 5px;"><p>Initial value for y size of longterm map. [mm]
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>xpos</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">-10000</td>
<td style="border:1px solid black; padding: 5px;"><p>Initial value for x offset of longterm map relative to world. [mm]
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>ypos</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">-10000</td>
<td style="border:1px solid black; padding: 5px;"><p>Initial value for y offset of longterm map relative to world. [mm]
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>id</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">0</td>
<td style="border:1px solid black; padding: 5px;"><p>Initial id of the map. Can later be set with LTMPARAMETER. The id ?id will be assigned to the longterm map for identification.
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
<td style="border:1px solid black; padding: 5px;"><b>cellsize</b></td>
<td style="border:1px solid black; padding: 5px;">UInt32</td>
<td style="border:1px solid black; padding: 5px;">50</td>
<td style="border:1px solid black; padding: 5px;"><p>Specify the size of the cells in the grid maps. [mm].
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>connectLaser</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">true</td>
<td style="border:1px solid black; padding: 5px;"><p>To use mapper as map server only set to false
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>verbose</b></td>
<td style="border:1px solid black; padding: 5px;">Boolean</td>
<td style="border:1px solid black; padding: 5px;">false</td>
<td style="border:1px solid black; padding: 5px;"><p>Print debug messages?
</p></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>mapDataDir</b></td>
<td style="border:1px solid black; padding: 5px;">String</td>
<td style="border:1px solid black; padding: 5px;">"data/maps/"</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

### ParameterSetInstance: MapperParams

#### Trigger Instance: CURPARAMETER

*Property:* active = **false**

*Documentation:*
<p>Drop the current map and create a new map with free cells of dimensions [mm] ?xsize and ?ysize. The map is initialized with an offset [mm] ?xoff and ?yoff relative to world coordinates. The cellsize used for this map as well as the obstacle growing type are specified in the ini-configuration. The id ?id will be assigned to the map to synchronize components, for example with SmartPlannerBreadthFirstSearch and SmartCdlServer.
Using this parameter is only allowed in neutral state.
</p>

#### Trigger Instance: LTMPARAMETER

*Property:* active = **false**

*Documentation:*
<p>Drop the longterm map and create a new map with free cells of dimensions [mm] ?xsize and ?ysize. The map is initialized at offset [mm] ?xoff and ?yoff. The cellsize used for this map is specified in the ini-configuration. The id ?id will be assigned to the map to identify it.
Using this parameter is only allowed in neutral state.
</p>

#### Trigger Instance: LTMINITIALIZE

*Property:* active = **false**

*Documentation:*
<p>Clear the longterm map and initialize the cells with value ?value.
</p>

#### Trigger Instance: CURSAVE

*Property:* active = **false**

*Documentation:*
<p>Save the current map to XML file test-cur-?number.xml.
</p>

#### Trigger Instance: CURLOAD

*Property:* active = **false**

*Documentation:*
<p>Load the current map from XML file test-cur-?number.xml.
Using this parameter is only allowed in neutral state.
</p>

#### Trigger Instance: CURLOADLTM

*Property:* active = **false**

*Documentation:*
<p>Load the current map from longterm map. The long term map is thresholded to meet the binary representation of the current map. See parameter CURLTM for information on the threshold.
Using this parameter is only allowed in neutral state.
</p>

#### Trigger Instance: CURSAVEXPM

*Property:* active = **false**

*Documentation:*
<p>Save the current map to XPM file test-cur-?number.xpm.
</p>

#### Trigger Instance: LTMSAVE

*Property:* active = **false**

*Documentation:*
<p>Save the longterm map to XML file test-ltm-.xml.
</p>

#### Trigger Instance: LTMLOAD

*Property:* active = **false**

*Documentation:*
<p>Load the longterm map from XML file test-ltm-.xml.
Using this parameter is only allowed in neutral state.
</p>

#### Trigger Instance: LTMSAVEXPM

*Property:* active = **false**

*Documentation:*
<p>Save the longterm map to XPM file test-ltm-.xpm.
</p>

#### Trigger Instance: LTMSAVEYAMLPGM

*Property:* active = **false**

*Documentation:*

#### Trigger Instance: LTMSAVEYAMLPPM

*Property:* active = **false**

*Documentation:*

#### Trigger Instance: LTMLOADYAML

*Property:* active = **false**

*Documentation:*

#### Trigger Instance: LTMSAVEIEEESTD

*Property:* active = **false**

*Documentation:*

#### Trigger Instance: LTMLOADIEEESTD

*Property:* active = **false**

*Documentation:*

#### Parameter Instance: CURLTM

*Documentation:*
<p>Configures whether the current map is preoccupied from the longterm map. Possible values for ?preoccupy: ENABLE will cause the component to load values from the longterm map in each cycle to the current map by applying the threshold ?thresh to decide on the occupancy of the cell. If DISABLE, the current map is not preoccupied.
A threshold is applied to preoccupy the binary current map: The cells of the new current map will be marked free if the value from the long term map is smaller than the threshold ?thresh. The cells will be marked as occupied if the long term map value is above the threshold.
</p>

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Parameter-Instance <b>CURLTM</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>preoccupation</b></td>
<td style="border:1px solid black; padding: 5px;">InlineEnumeration</td>
<td style="border:1px solid black; padding: 5px;">DISABLE</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>threshold</b></td>
<td style="border:1px solid black; padding: 5px;">Int32</td>
<td style="border:1px solid black; padding: 5px;">20</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

#### Parameter Instance: CUREMPTY

*Documentation:*
<p>Change the mode ?mode of building the current map: ACCUMULATE will add new occupied cells from the laser scan while EMPTY will clear the map in each cycle before adding occupied cells from the laser scan.
</p>

<table style="border-collapse:collapse;">
<caption><i>Table:</i> Parameter-Instance <b>CUREMPTY</b></caption>
<tr style="background-color:#ccc;">
<th style="border:1px solid black; padding: 5px;"><i>Attribute Name</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Type</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Value</i></th>
<th style="border:1px solid black; padding: 5px;"><i>Attribute Description</i></th>
</tr>
<tr>
<td style="border:1px solid black; padding: 5px;"><b>mapmode</b></td>
<td style="border:1px solid black; padding: 5px;">InlineEnumeration</td>
<td style="border:1px solid black; padding: 5px;">ACCUMULATE</td>
<td style="border:1px solid black; padding: 5px;"></td>
</tr>
</table>

