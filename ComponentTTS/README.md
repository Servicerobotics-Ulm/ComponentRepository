<!--- This file is generated from the ComponentTTS.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentTTS Component

![ComponentTTS-ComponentImage](https://github.com/Servicerobotics-Ulm/ComponentRepository/blob/master/ComponentTTS/model/ComponentTTSComponentDefinition.jpg)

SmartTTS is a component for text to speech (tts) synthesis. SmartTTS pipes speech output messages to stdin of an arbitrary executable. 
It is thus a simple wrapper for all tts applications that accept text via stdin, e.g. mbrola, festival or /bin/cat for debugging. 
It provides send and query services. Speech messages will be appended to a message queue. The queue is processed first in first out. 
Messages of which the discard time is in the past (at the time of processing) are discarded.
On Ubuntu 16.04, the speech-dispatcher package can be used. Configure
the component to use spd-say: set parameter 'player' to '/usr/bin/spd-say -e'.

| Metaelement | Documentation |
|-------------|---------------|
| License | LGPL |
| Hardware Requirements | none |
| Purpose | HMI |



## Service Ports

### SpeechQueryServiceAnsw

Speech messages sent via this port will be queued for processing. Use this service port if you need to know whether the message was spoken or discarded for some reason. The answer includes boolean values: success, discarded, error.

### SpeechSendServiceIn

Speech messages sent via this port will be queued for processing.


## Component Parameters ComponentTTSParams

### InternalParameter Settings

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| player | String |  |

