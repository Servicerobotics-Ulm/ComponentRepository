# SmartMapperGridMap Component

![SmartMapperGridMap-ComponentImage](https://github.com/Servicerobotics-Ulm/ComponentRepository/blob/master/SmartMapperGridMap/model/SmartMapperGridMap.jpg)

The SmartMapperGridMap provides mapping services based on occupancy grid maps. Laser scans are taken for building a current and a longterm map.

The current map represents the latest environment of the robot. It can be preoccupied with grids of the longterm map and can be used for path planning e.g. with SmartPlannerBreadthFirstSearch. It contains either occupied cells or free cells (binary). An optional obstacle growing can be applied to the current map.

The longterm map holds cell values from 0 to 255. Values from 0 to 127 denote the traversability where 0 is completely free. Values from 128 to 255 are special values: Obstacles are marked with 128, cells occupied by obstacle growing with 129 and undeletable grids are marked with 130. The cell values can be accumulated over time to represent the environment over a longer period.

Both grid maps can be saved to XPM and XML and loaded from XML files.

Note: This component is used in Tutorials (e.g. Lesson 1). 

| Metaelement | Documentation |
|-------------|---------------|
| License | LGPL |
| Hardware Requirements | - |
| Purpose | Navigation |


## Coordination Port SlavePort


### States

See States for descriptions of possible states and their meaning.

| MainState Name | MainState Description |
|----------------|-----------------------|
| Neutral | No maps are being built. |
| BuildBothMaps | Both the current and the longterm maps are being built. |
| BuildCurrMap | The current map is being built. |
| BuildLtmMap | The longterm map is being built. |

### DynamicWiring

Slave part of wiring pattern. It is responsible for changing the port connections within the component.

### Parameter

 Accepts parameters to handle the current and longterm map. See Parameters.

						Please note that the following parameters can only be sent when the component is in state neutral:

    					- CURPARAMETER
    					- CURLOAD
    					- CURLOADLTM
    					- LTMPARAMETER
    					- LTMLOAD


## Service Ports

### LaserServiceIn

Port has to be connected to the laser component (e.g. SmartLaserLMS200Server). The received laser scan is used for building the maps.

### CurrMapOut

Pushes the current map to subscribers (e.g. SmartPlannerBreadthFirstSearch) if the current map is being built (component is in state buildbothmaps or buildcurrentmap).
						This port should be used if a map is required continuously. For example, the SmartPlannerBreadthFirstSearch always requires an up-to-date-map for proper path planning. Other components may require the map only occasionally and shall use the query port: curGridMapQueryServer.

### CurrQueryServer

Port to query the current map. If the current map is not being built (component in state neutral or buildcurrentmap), the answer is stamped invalid. It will be stamped valid if the current map is being built (component in state buildbothmaps or buildcurrentmap).
						The query port should be used if a map is required only occasionally. The port currentGridMapServer (a SmartPushNewestServer) should be used in components where it is necessary to continuously receive the latest map.

### LtmQueryServer

Port to query the longterm map. Queries will be replied to in every state.


## Component Parameters SmartMapperGridMapParams

### InternalParameter CurrentMap

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| interval | UInt32 | Specify the interval for considering laser scans to build the current map. The component will process every nth laser scan. The actual interval for building the current map thus depends on the value of this setting and on the rate of the laser scan for pushing new scans. This can be used to slow down the building of the current map. |
| growing | String | Obstacle growing mode. Obstacle can be grown in circles with 8 (circle8) or 16 (circle16) (circle40) (circle32)  occupied cells around the obstacle or as a star of 8 (star8) or 16 (star16) occupied cells around the obstacle. Obstacle growing can also be deactivated with value no. |
| xsize | UInt32 | Initial value for x size of current map. [mm]. |
| ysize | UInt32 | Initial value for y size of current map. [mm]. |
| xpos | Int32 | Initial value for x offset of current map relative to world. [mm]. |
| ypos | Int32 | Initial value for y offset of current map relative to world. [mm]. |
| id | UInt32 | Initial id of the current map. Can later be set with parameter CURPARAMETER. The id ?id will be assigned to the current map to synchronize components, for example with SmartPlannerBreadthFirstSearch and SmartCdlServer. |

### InternalParameter LtmMap

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| kalman | Int32 | Adaption rate for kalman filter weighting. [0-255]. |
| xsize | UInt32 | Initial value for x size of longterm map. [mm]. |
| ysize | UInt32 | Initial value for y size of longterm map. [mm] |
| xpos | Int32 | Initial value for x offset of longterm map relative to world. [mm] |
| ypos | Int32 | Initial value for y offset of longterm map relative to world. [mm] |
| id | UInt32 | Initial id of the map. Can later be set with LTMPARAMETER. The id ?id will be assigned to the longterm map for identification. |

### InternalParameter General

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| cellsize | UInt32 | Specify the size of the cells in the grid maps. [mm]. |
| connectLaser | Boolean | To use mapper as map server only set to false |
| verbose | Boolean | Print debug messages? |
| mapDataDir | String |  |

### ParameterSetInstance MapperParams

#### TriggerInstance CURPARAMETER

active = false

Drop the current map and create a new map with free cells of dimensions [mm] ?xsize and ?ysize. The map is initialized with an offset [mm] ?xoff and ?yoff relative to world coordinates. The cellsize used for this map as well as the obstacle growing type are specified in the ini-configuration. The id ?id will be assigned to the map to synchronize components, for example with SmartPlannerBreadthFirstSearch and SmartCdlServer.
Using this parameter is only allowed in neutral state.

#### TriggerInstance LTMPARAMETER

active = false

Drop the longterm map and create a new map with free cells of dimensions [mm] ?xsize and ?ysize. The map is initialized at offset [mm] ?xoff and ?yoff. The cellsize used for this map is specified in the ini-configuration. The id ?id will be assigned to the map to identify it.
Using this parameter is only allowed in neutral state.

#### TriggerInstance LTMINITIALIZE

active = false

Clear the longterm map and initialize the cells with value ?value.

#### TriggerInstance CURSAVE

active = false

Save the current map to XML file test-cur-?number.xml.

#### TriggerInstance CURLOAD

active = false

Load the current map from XML file test-cur-?number.xml.
Using this parameter is only allowed in neutral state.

#### TriggerInstance CURLOADLTM

active = false

Load the current map from longterm map. The long term map is thresholded to meet the binary representation of the current map. See parameter CURLTM for information on the threshold.
Using this parameter is only allowed in neutral state.

#### TriggerInstance CURSAVEXPM

active = false

Save the current map to XPM file test-cur-?number.xpm.

#### TriggerInstance LTMSAVE

active = false

Save the longterm map to XML file test-ltm-.xml.

#### TriggerInstance LTMLOAD

active = false

Load the longterm map from XML file test-ltm-.xml.
Using this parameter is only allowed in neutral state.

#### TriggerInstance LTMSAVEXPM

active = false

Save the longterm map to XPM file test-ltm-.xpm.

#### TriggerInstance LTMSAVEYAMLPGM

active = false


#### TriggerInstance LTMSAVEYAMLPPM

active = false


#### TriggerInstance LTMLOADYAML

active = false


#### ParameterInstance CURLTM

Configures whether the current map is preoccupied from the longterm map. Possible values for ?preoccupy: ENABLE will cause the component to load values from the longterm map in each cycle to the current map by applying the threshold ?thresh to decide on the occupancy of the cell. If DISABLE, the current map is not preoccupied.
A threshold is applied to preoccupy the binary current map: The cells of the new current map will be marked free if the value from the long term map is smaller than the threshold ?thresh. The cells will be marked as occupied if the long term map value is above the threshold.

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| preoccupation | InlineEnumeration |  |
| threshold | Int32 |  |

#### ParameterInstance CUREMPTY

Change the mode ?mode of building the current map: ACCUMULATE will add new occupied cells from the laser scan while EMPTY will clear the map in each cycle before adding occupied cells from the laser scan.

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| mapmode | InlineEnumeration |  |

