# SmartJoystickNavigation Component

![SmartJoystickNavigation-ComponentImage](https://github.com/Servicerobotics-Ulm/ComponentRepository/blob/master/SmartJoystickNavigation/model/SmartJoystickNavigationComponentDefinition.jpg)

The SmartJoystickNavigation component takes joystick input commands (CommJoystick) and translates them to v/omega navigation commands (CommNavigationVelocity). This component can be used in combination with SmartJoystickServer to receive input commands and send them to motion execution, e.g. SmartCdlServer, for collision free steering using a joystick.

Note: This component is used in Examples. 

| Metaelement | Documentation |
|-------------|---------------|
| License | LGPL |
| Hardware Requirements | - |
| Purpose | Navigation |


## Coordination Port SlavePort


### States


| MainState Name | MainState Description |
|----------------|-----------------------|

### DynamicWiring

Slave part of wiring pattern. It is responsible for changing the port connections within the component.

### Parameter


## Service Ports

### JoystickServiceIn

The component receives joystick input commands via this ports which are translated to navigation commands. This port is connected e.g. to SmartJoystickServer that provides access to joystick hardware.

### NavVelServiceOut

Typically connected to the robot base (e.g. SmartPioneerBaseServer or SmartCdlServer). This port sends navigation commands v, omega which have been translated from joystick commands.


## Component Parameters SmartJoystickNavigationParameters

### InternalParameter Settings

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| max_velocity | Double | Defines the maximum velocity. This is the velocity [m/s] which will be sent when the joystick axis is at full peak. |
| max_steering | Double | Defines the minimum steering angle. This is the angle [rad] which will be sent as omega when the joystick axis is at full peak. |

