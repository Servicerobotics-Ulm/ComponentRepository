<!--- This file is generated from the ComponentSkillInterface.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentSkillInterface Component

![ComponentSkillInterface-ComponentImage](model/ComponentSkillInterfaceComponentDefinition.jpg)

NOTE: THIS COMPONENT PROJECT ONLY CONTAINS A COMPONENT HULL. IMPLEMENTATION OF THIS COMPONENT IS WORK IN PROGRESS. YOU CAN USE THIS COMPONENT HULL TO FILL IN YOUR OWN IMPLEMENTATION.

TODO: ADD description!

| Metaelement | Documentation |
|-------------|---------------|
| License |  |
| Hardware Requirements |  |
| Purpose |  |



## Service Ports


## Component Parameters ComponentSkillInterfaceParams

### InternalParameter Settings

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| communicationType | InlineEnumeration |  |
| ip | String |  |
| port | UInt32 |  |
| use_socket_timeout | Boolean |  |
| socket_timeout_s | UInt32 | timeout in sec, do not use small values (<2 sec), risk of loosing data, due to client side closed connection! |
| verbose | Boolean |  |

