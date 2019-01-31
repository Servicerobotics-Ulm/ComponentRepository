<!--- This file is generated from the SmartLaserLMS200Server.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# SmartLaserLMS200Server Component

![SmartLaserLMS200Server-ComponentImage](model/SmartLaserLMS200ServerComponentDefinition.jpg)

The SmartLaserLMS200Server makes laser scans from SICK LMS 200 and PLS langer rangers available. Scans can be requested by push newest or query communication.

Note: This component is used in Tutorials (e.g. Lesson 1).

GPL-License: includes Code from the Player Project.

| Metaelement | Documentation |
|-------------|---------------|
| License | GPL |
| Hardware Requirements | SICK LMS 200 and PLS |
| Purpose | Hardware-Driver |


## Coordination Port CoordinationPort


### States


| MainState Name | MainState Description |
|----------------|-----------------------|

### DynamicWiring

Slave part of wiring pattern. It is responsible for changing the port connections within the component.

### Parameter


## Service Ports

### BaseStateIn

Typically connected to the robot base (e.g. SmartPioneerBaseServer) to receive the base state which is included in the laser scan. Whether this port is connected or not can be configured via the ini file. Used to stamp every laser scan with the base state where the scan was recorded, if laser is mounted on a mobile platform. If this port is not connected, the pose where recorded will contain the values configured in the ini file.

### LaserQueryServiceAnsw

 Query to request the latest laser scan. Scan will be stamped invalid if it was not possible to get the base state but the baseClient port is connected.

### LaserScanOut

Push latest laser scan. The rate with which the server pushes depends on the parametrization of the laser scanner, especially baudrate and resolution. A typical rate is 38 Hz (500000 bps, 0.5 degree resolution). Scan will be stamped invalid if it was not possible to get the base state but the baseClient port is connected.


## Component Parameters SmartLaserLMS200ServerParams

### InternalParameter Laser

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| sick_type | String | Type of SICK Scanner. Possible values: LMS and PLS. |
| x | Double | The position of the scanner on the robot. The azimuth is relative to the perspective mounting point (turrent or base) on the robot. Units in [mm] and [rad]. |
| y | Double | The position of the scanner on the robot. The azimuth is relative to the perspective mounting point (turrent or base) on the robot. Units in [mm] and [rad]. |
| z | Double | The position of the scanner on the robot. The azimuth is relative to the perspective mounting point (turrent or base) on the robot. Units in [mm] and [rad]. |
| azimuth | Double | The position of the scanner on the robot. The azimuth is relative to the perspective mounting point (turrent or base) on the robot. Units in [mm] and [rad]. |
| device | String | Serial device of the laser scanner. |
| baudrate | Int32 | Speed for communication [bps]. Possible values: 9600, 38400 and 500000. |
| resolution | Int32 | The angular resolution [0.01 degree]. Possible Values are 25 (0.25 degree), 50 (0.5 degree) and 100 (1 degree). However, in 0.25 degree mode, only 100 degree scans are reported. This value is directly passed to the SICK device. |
| length_unit | Int32 | Length unit of reported distances [mm]. Possible values are 1, 10. Corresponding maximum distances are 8m, 80m. This value is directly passed to the SICK device. |
| verbose | Boolean | Used when debugging the laser interface. |

### InternalParameter Base

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| subscribe | Boolean | Subscribe to base server for position updates (true)? |
| x | Double | Use fixed values x, y, z [mm] for pose of the scanner if not subscribed to base server. See server ports. |
| y | Double | Use fixed values x, y, z [mm] for pose of the scanner if not subscribed to base server. See server ports. |
| z | Double | Use fixed values x, y, z [mm] for pose of the scanner if not subscribed to base server. See server ports. |
| base_a | Double | Use fixed value base_a [rad] for pose of the scanner if not subscribed to base server. See server ports. |

