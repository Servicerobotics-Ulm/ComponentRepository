<!--- This file is generated from the ComponentSymbolicPlanner.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentSymbolicPlanner Component

![ComponentSymbolicPlanner-ComponentImage](model/ComponentSymbolicPlannerComponentDefinition.jpg)

Provides a symbolic planner service. Works with ff, metric-ff (suggested) and lama.

| Metaelement | Documentation |
|-------------|---------------|
| License | LGPL |
| Hardware Requirements | none |
| Purpose | Planning |



## Service Ports

### SymbolicPlannerQueryServer

Accepts a planning request, hands it over to symbolic planer and returns the result. 
		Takes a domain and problem descriptions (pddl, see metric-ff sources for examples) as input and returns the solution by the symbolic planner.


