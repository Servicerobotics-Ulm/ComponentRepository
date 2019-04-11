<!--- This file is generated from the ComponentKinectV1Server.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentKinectV1Server Component

![ComponentKinectV1Server-ComponentImage](model/ComponentKinectV1ServerComponentDefinition.jpg)


| Metaelement | Documentation |
|-------------|---------------|
| License |  |
| Hardware Requirements |  |
| Purpose |  |



## Service Ports


## Component Parameters ComponentKinectV1ServerParams

### InternalParameter settings

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| debug_info | Boolean |  |
| valid_image_time | Double |  |
| high_resolution | Boolean |  |
| undistort_image | Boolean |  |
| rgb_mode | UInt8 |  |
| depth_mode | UInt8 |  |
| pushnewest_rgbd_image | Boolean |  |
| pushnewest_color_image | Boolean |  |
| pushnewest_depth_image | Boolean |  |

### InternalParameter sensor_pose

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| x | Double |  |
| y | Double |  |
| z | Double |  |
| azimuth | Double |  |
| elevation | Double |  |
| roll | Double |  |

### InternalParameter base

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| on_ptu | Boolean |  |
| on_base | Boolean |  |
| x | Int32 |  |
| y | Int32 |  |
| z | Int32 |  |
| base_a | Double |  |
| steer_a | Double |  |

### InternalParameter hardware_properties

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| min_distance | Double |  |
| max_distance | Double |  |

