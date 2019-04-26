<!--- This file is generated from the ComponentRealSenseV2Server.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentRealSenseV2Server Component

![ComponentRealSenseV2Server-ComponentImage](model/ComponentRealSenseV2ServerComponentDefinition.jpg)


| Metaelement | Documentation |
|-------------|---------------|
| License |  |
| Hardware Requirements |  |
| Purpose |  |



## Service Ports


## Component Parameters ComponentRealSenseV2ServerParams

### InternalParameter settings

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| debug_info | Boolean |  |
| valid_image_time | Double |  |
| undistort_image | Boolean |  |
| pushnewest_rgbd_image | Boolean |  |
| pushnewest_color_image | Boolean |  |
| pushnewest_depth_image | Boolean |  |
| post_processing | Boolean |  |
| device_serial_number | String |  |

### InternalParameter stereo

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| baseline | Float |  |

### InternalParameter sensor_pose

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| x | Double |  |
| y | Double |  |
| z | Double |  |
| azimuth | Double |  |
| elevation | Double |  |
| roll | Double |  |

### InternalParameter RGB_config

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| width | Int32 |  |
| height | Int32 |  |
| framerate | Int32 |  |

### InternalParameter Depth_config

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| width | Int32 |  |
| height | Int32 |  |
| framerate | Int32 |  |

### InternalParameter base

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| on_manipulator | Boolean |  |
| on_ptu | Boolean |  |
| on_base | Boolean |  |
| on_ur | Boolean |  |
| x | Int32 |  |
| y | Int32 |  |
| z | Int32 |  |
| base_a | Double |  |
| steer_a | Double |  |

