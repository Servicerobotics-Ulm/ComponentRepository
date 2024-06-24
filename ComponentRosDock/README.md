### Docking

#### Laserscan Units:
- start_angle: [rad] (-3.14 .. 3.14)
- resolution: [rad]
- distance_min: [m]
- distance_max: [m]

#### How to use (for testing):
needs `ComponentRosBaseFake`

- start `roscore`
- start Naming Service `cd $SMART_ROOT_ACE && ./startSmartSoftNamingService`
- start Base Fake `cd $SMART_ROOT_ACE && ./bin/ComponentRosBaseFake`
- start Docking `cd $SMART_ROOT_ACE && ./bin/ComponentRosDock`
- start Robot Console `cd $SMART_ROOT_ACE && ./bin/SmartRobotConsole`

- use `State Client` (98) in Robot Console to activate **Docking** (when Docking is succeeded, Docking Activity is stopped)
- for another Docking process, use `State Client` (98) in Robot Console to activate **Neutral** before **Docking** can be activated again

#### How to use (for running on Robot)

- make sure the catkin workspace including mojin_seronet is sourced (check `$CMAKE_PREFIX_PATH` and `$ROS_PACKAGE_PATH`)
