<!--- This file is generated from the ComponentRealSensePersonTracker.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentRealSensePersonTracker Component

![ComponentRealSensePersonTracker-ComponentImage](model/ComponentRealSensePersonTrackerComponentDefinition.jpg)


| Metaelement | Documentation |
|-------------|---------------|
| License |  |
| Hardware Requirements |  |
| Purpose |  |



## Service Ports


## Component Parameters ComponentRealSensePersonTrackerParams

### InternalParameter Settings

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| withDebugWindow | Boolean |  |
| display_depth_image | Boolean |  |

### InternalParameter Tracking

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| RGBDSource | Int8 |  |

### ExtendedTrigger PersonTofollow

active = false

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| id_to_follow | Int16 |  |

### ParameterSetInstance TrackingParam

#### TriggerInstance FOLLOW_RESET

active = false


#### TriggerInstance SET_MAX_COV

active = false


### InternalParameter Realsense_to_kinect

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| x | Double |  |
| y | Double |  |
| z | Double |  |
| azimuth | Double |  |
| elevation | Double |  |
| roll | Double |  |

