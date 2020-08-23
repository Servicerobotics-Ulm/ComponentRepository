### Docking

needs `ComponentRosBaseFake`

#### How to use (for testing):

- start `roscore`
- start Naming Service `cd $SMART_ROOT_ACE && ./startSmartSoftNamingService`
- start Base Fake `cd $SMART_ROOT_ACE && ./bin/ComponentRosBaseFake`
- start Docking `cd $SMART_ROOT_ACE && ./bin/ComponentRosDock`
- start Robot Console `cd $SMART_ROOT_ACE && ./bin/SmartRobotConsole`

- use `State Client` (98) in Robot Console to activate **Docking** (when Docking is succeeded, Docking Activity is stopped)
- for another Docking process, use `State Client` (98) in Robot Console to activate **Neutral** before **Docking** can be activated again
