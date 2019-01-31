<!--- This file is generated from the SmartJoystickServer.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# SmartJoystickServer Component

![SmartJoystickServer-ComponentImage](model/SmartJoystickServerComponentDefinition.jpg)

The SmartJoystickServer provides access to input commands from a joystick via PushNewest communication pattern. The input commands are represented by x/y-value (as available) and an identifier for the button pressed.

Note: This component is used in Examples. 

| Metaelement | Documentation |
|-------------|---------------|
| License | LGPL |
| Hardware Requirements | Linux supported joystick, e.g. 'Logitech dual action' |
| Purpose | 	Hardware-Driver |



## Service Ports

### JoystickServcieOut

Provides the joystick input as CommJoystick communication object. 


## Component Parameters SmartJoystickServerParameters

### InternalParameter General

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| device | String | Path to joystick device. |
| invert_y1_axis | Boolean | Invert Y axis. |
| invert_y2_axis | Boolean |  |
| invert_x1_axis | Boolean |  |
| invert_x2_axis | Boolean |  |
| interval_min | Double | Specifies the boundary for timeout when reading joystick data. |
| interval_max | Double | Specifies the boundary for timeout when reading joystick data. |
| verbose | Boolean | Display every input command read from joystick device. |

