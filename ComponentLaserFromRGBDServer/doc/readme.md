# 2D Laser generation from RGBD camera
* Initialize the laser parameters i.e 
    * angle resolution = 0.5 degrees as default, parameter in .ini file
    * number of laser rays = (horizontal field of view of camera / angle resolution)
    * angle of the first laser ray = -1 * (horizontal field of view of camera / 2.0 )
* Receive **CommRGBDImage** from 3D camera component.
    * Get **CommDepthImage** from **CommRGBDImage**.
* Generate pointcloud in the camera frame using depth image and intrinsic parameters.
* Transform the pointcloud into robot frame using camera sensor pose (from comobj).
* Remove the points correspond to the floor.
    * Filter the pointcloud using pass-through filter on Z coordinate of points (Z<sub>floor_height</sub> < Z<sub>Ri</sub> x < Z<sub>max</sub>x).
* Down project the pointcloud and find the minimum distance for laser each ray.
    * find distance of the each laser ray d<sub>i</sub> with angleand index l<sub>i</sub>.
    * Calculate the angle and distance r<sub>i</sub> of each pointin XY plane of robot. 
        *  &theta;<sub>i</sub> = atan2(y<sub>i</sub>, x<sub>i</sub>)
        *  r<sub>i</sub> = &radic;(x<sub>i</sub><sup>2</sup>, y<sub>i</sub><sup>2</sup>)
    * Find ray index l<sub>i</sub> fromand set ray distance as r<sub>i</sub>.
        *  if(r<sub>i</sub> < d<sub>i</sub>) d<sub>i</sub> = r<sub>i</sub>
* Set the calculated distances di to **CommMobileLaserScan**.

  <img src="cf_rbf_frames.png" alt="showing Robot and Camera frames" width="400"/>
