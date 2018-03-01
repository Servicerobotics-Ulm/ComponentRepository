# SmartGMapping Component

![SmartGMapping-ComponentImage](https://github.com/Servicerobotics-Ulm/ComponentRepository/blob/master/SmartGMapping/model/SmartGMapping.jpg)

SmartGMapping implements GMapping for simultaneous localization and mapping (SLAM). GMapping uses the Rao-Blackwellized particle filer for learning grid maps. It adaptively reduces the number of particles which each carries an individual map of the environment. The robot's pose is estimated by taking the robot's movement and the most recent observation into account.

Triggered by a parameter communication object, the grid map can be saved to file. Based on its localization, SmartGMapping sends position updates to the base server (e.g. SmartPioneerBaseServer).

See also: http://openslam.org/gmapping.html 

| Metaelement | Documentation |
|-------------|---------------|
| License | 	LGPL |
| Hardware Requirements | - |
| Purpose | Navigation |



## Service Ports


## Component Parameters SmartGMappingParams

### InternalParameter gfs

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| particles | Int32 | Number of particles to use. |
| angularUpdate | Double | Used for measurement integration. |
| linearUpdate | Double | Used for measurement integration. |
| delta | Double | Map resolution [m]. |
| maxrange | Double | Maximum valid range [m] of laser scanner, e.g. 81m for SICK LMS, 50m for SICK PLS. |
| maxUrange | Double | Closeup range of scanner [m]. |
| sigma | Double | Scan matcher cell sigma [m], used for the greedy search. |
| regscore | Double | Minimum score for regsistering a scan. |
| iterations | Int32 | Number of iterations of scanmatcher. |
| critscore | Double | Internal parameter. |
| maxMove | Double | Maximum move among two scans. This detects some corrupted logs. |
| lstep | Double | Optimization step for translation. |
| astep | Double | Optimization step for rotation. |
| lsigma | Double | Sigma likelihood of one beam. |
| lskip | Int32 | Number of beams to skip in the likelihood computation. |
| kernelSize | Int32 | Size of kernel. The higher the value, the slower the filter, the better it can deal with noise, but the less precise and slower. |
| ogain | Int32 | Gain for smoothing the likelihood. |
| resampleThreshold | Double | Neff based threshold for resampling. |
| srr | Double | Motion model parameter: error in translation as a function of translation. |
| srt | Double | Motion model parameter: error in translation as a function of rotation. |
| str | Double | Motion model parameter: error in rotation as a function of translation. |
| stt | Double | Motion model parameter: error in rotation as a function of rotation. |
| xmin | Double | Initial map parameter. |
| ymin | Double | Initial map parameter. |
| xmax | Double | Initial map parameter. |
| ymax | Double | Initial map parameter. |
| generateMap | Boolean |  |

### InternalParameter settings

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| verbose | Boolean |  |
| initial_pose_x | Double |  |
| initial_pose_y | Double |  |
| initial_pose_azimuth | Double |  |

### ParameterSetInstance GMappingParameter

#### TriggerInstance INITNEWMAP

active = false


#### TriggerInstance SAVEMAP

active = false

Stores the current grid map to <?dirname>/<?mapname>.pgm and meta information to data/<?mapname>.yaml.

