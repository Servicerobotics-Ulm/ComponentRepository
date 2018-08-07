<!--- This file is generated from the SmartPlannerBreadthFirstSearch.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# SmartPlannerBreadthFirstSearch Component

![SmartPlannerBreadthFirstSearch-ComponentImage](https://github.com/Servicerobotics-Ulm/ComponentRepository/blob/master/SmartPlannerBreadthFirstSearch/model/SmartPlannerBreadthFirstSearchComponentDefinition.jpg)

The SmartPlannerBreadthFirstSearch provides path planning services based on grid maps. It uses a grid map from a map building component (e.g. SmartMapperGridMap) and sends an intermediate waypoint as well as the goalpoint to the motion execution (e.g. SmartCdlServer).

A wave propagation algorithm starting from goal cells backward to the current position of the robot is used to calculate a path. The path planning is further enhanced by a path shortening heuristic: the path is followed starting at the current robot position until a straight line from the robot position to the cell in question on the path interferes with an obstacle. The prior cell is then sent as an intermediate waypoint. The geometric path planning is applied continuously every one second.

Several goal circles and/or goal lines can be specified as goals in the planner. The planner will generate a path to the goal with the shortest distance.

The SmartPlannerBreadthFirstSearch for example can be used with SmartCdlServer which cannot handle local minimas. Goals are then specified in the SmartPlannerBreadthFirstSearch. A CDL_GOAL_REACHED event is fired by the CDL component as soon as the final goal is reached.

Note: This component is used in Tutorials (e.g. Lesson 1).

See also:
Christian Schlegel. Navigation and Execution for Mobile Robots in Dynamic Environments: An Integrated Approach. p. 27-29. Dissertation, Fakultät für Informatik, Universität Ulm, 2004. 

| Metaelement | Documentation |
|-------------|---------------|
| License | LGPL |
| Hardware Requirements | - |
| Purpose | 	Navigation |


## Coordination Port CoordinationPort


### States

See States for descriptions of possible states and their meaning.

| MainState Name | MainState Description |
|----------------|-----------------------|
| Neutral | The path planning execution cycle is stopped. No path planning is performed, no intermediate waypoints and goal points are sent. |
| PathPlanning | The component will continuously (every 1 second) plan a path to the goalpoints. Intermediate waypoints and goals are sent. |

### DynamicWiring

Slave part of wiring pattern. It is responsible for changing the port connections within the component.

### Parameter

Accepts various parameters for managing goals. See Parameters.

## Service Ports

### CurMapClient

The planner will plan paths in grid maps sent to this port. Typically connected to a map building component such as SmartMapperGridMap.

### BaseStateClient

Base states as input for path planning shall be sent to this port. The path is planned based on the current position sent through this port. Typically connected to the robot base, e.g. SmartPioneerBaseServer.

### PlannerGoalServer

If a path was found (event PLANNER_NO_ERROR fired), this port pushes an valid (invalid flag set to false) intermediate waypoint and goal point calculated by the planner to the motion execution. If no path was found, the waypoint and goalpoint are sent with the invalid flag set to true.

### PlannerEventServer

 The event state PLANNER_UNKNOWN can be used for activation if the current state is unknown. Events are sent on every change.

						The port will send the events listed below:

						- PLANNER_NO_PATH: Fired if no path could be found for at least five seconds. Invalid goals (invalid flag=true) are pushed via plannerUpdateNewestServer when no path was found.
						- PLANNER_NO_ERROR: No errors in planner, valid goals (invalid flag=false) are pushed via plannerUpdateNewestServer.
						- PLANNER_UNKNOWN_ERROR: An unknown error occurred.
						- PLANNER_NO_GOAL_AVAILABLE: No goals available.
						- PLANNER_GOAL_NOT_MARKED: Every goal cell is already occupied. The goal cells could not be marked.
						- PLANNER_START_OCCUPIED_OBSTACLE: The start cell (current robot position) is occupied by an obstacle.
						- PLANNER_START_OCCUPIED_GOAL: The robot is standing on the goal cell.
						- PLANNER_WRONG_MAPID: The latest grid map is of another id than the current planner id. The planner will not plan anything and will not send intermediate waypoints and goal points.



## Component Parameters SmartPlannerParams

### InternalParameter Settings

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| no_path_event_timeout | Double |  |

### ParameterSetInstance PlannerParams

#### ParameterInstance ID

Set the goal id. Used to synchronize components, for example with SmartMapperGridMap and SmartCdlServer.

| Attribute Name | Attribute Type | Description |
|----------------|----------------|-------------|
| id | UInt32 |  |

#### TriggerInstance DELETEGOAL

active = false

Delete all specified planner goals.

#### TriggerInstance SETDESTINATIONCIRCLE

active = false

Specify a goal point by setting the coordinates [mm] ?x, ?y and radius ?r for the goal circle.

#### TriggerInstance SETDESTINATIONLINE

active = false

Specify a goal as a line: the planner will plan the shortest path from the current position to a line between the point ?x1,?y1 and ?x2, ?y2.

