^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hey5_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2016-09-01)
------------------

0.1.3 (2016-04-25)
------------------
* Fix error on loading stil by Rviz as they seem to be malformed
  Error was
  The STL file 'package://XXXX/meshes/finger_flex_collision.stl' is malformed. According to the binary STL header it should have '108' triangles, but it has too much data for that to be the case.
  Seems to be fixed in upcoming Rviz (not released yet) https://github.com/ros-visualization/rviz/issues/913
* Install LICENSE.txt
* Change URDF license headers to CC BY 4.0.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Sam Pfeiffer

0.1.2 (2015-06-12)
------------------
* hey5_description: Install gazebo folder
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.1.1 (2015-06-12)
------------------
* Add hey5 gazebo simulation plugin
* Add gazebo references with black material
* Contributors: Bence Magyar, Luca Marchionni

0.1.0 (2015-04-22)
------------------
* Add hand_grasping_frame
* Contributors: Bence Magyar

0.0.6 (2015-01-15)
------------------
* Update hand inertial properties
  - Inertia tensors for all links except palm are approximately four
  orders of magnitude larger to prevent the Gazebo simulation from
  becoming unstable.
  - Palm mass is lower to take into account that actuator mass is
  specified separately.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.5 (2014-12-23)
------------------
* License model as CC-BY 4.0, add attribution text
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.4 (2014-12-12)
------------------
* Remove hi-res finger meshes
* Minor dimension tweaks to harmonize with docs, CAD
* Update hand transmissions, fully open config
  - Add some overlap between abduction and flexion to more closely mimic
  hardware
  - Spread fingers more on fully open configuration
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.3 (2014-11-13)
------------------
* Remove reference to non-existing file
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.2 (2014-11-13)
------------------
* Use transmissions in hand model
* Remove usage of mimic joints
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.1 (2014-11-05)
------------------
* First version of the Hey5 hand model
* Contributors: Adolfo Rodriguez Tsouroukdissian
