<!--- This file is generated from the ComponentLaserFromRGBDServer.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentLaserFromRGBDServer Component

![ComponentLaserFromRGBDServer-ComponentImage](model/ComponentLaserFromRGBDServerComponentDefinition.jpg)


| Metaelement | Documentation |
|-------------|---------------|
| License |  |
| Hardware Requirements |  |
| Purpose |  |



## Service Ports


## Component Parameters ComponentLaserFromRGBDServerParams

### InternalParameter scanner

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| verbose | Boolean |  |
| on_turret | Boolean |  |
| x | Int32 |  |
| y | Int32 |  |
| z | Int32 |  |
| azimuth | Double |  |
| elevation | Double |  |
| roll | Double |  |

### InternalParameter laser_generator

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| min_range | UInt32 |  |
| max_range | UInt32 |  |
| vertical_fov | Double |  |
| rgbd_source | UInt8 |  |
| angle_resolution | Double |  |
| generationPeriodSec | UInt32 |  |
| generationPeriodMilliSec | UInt32 |  |
| floor_threshold_distance | Double |  |

### InternalParameter base_manipulator

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| on_base | Boolean |  |
| x | Int32 |  |
| y | Int32 |  |
| z | Int32 |  |
| base_a | Double |  |
| steer_a | Double |  |
| turret_a | Double |  |

### InternalParameter services

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| activate_push_newest | Boolean |  |
| active_push_timed | Boolean |  |

