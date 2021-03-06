//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//  
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#include "PoseUpdateActivity.hh"
#include "SmartURServerLegacy.hh"

#include <iostream>

#include "UniversalRobotic.hh"


PoseUpdateActivity::PoseUpdateActivity(SmartACE::SmartComponent *comp) 
:	PoseUpdateActivityCore(comp)
{
	std::cout << "constructor PoseUpdateActivity\n";
}
PoseUpdateActivity::~PoseUpdateActivity() 
{
	std::cout << "destructor PoseUpdateActivity\n";
}


void PoseUpdateActivity::on_baseStateServiceIn(const CommBasicObjects::CommBaseState &input)
{
	// upcall triggered from InputPort baseStateServiceIn
	// - use a local mutex here, because this upcal is called asynchroneously from outside of this task
	// - do not use longer blocking calls here since this upcall blocks the InputPort baseStateServiceIn
	// - if you need to implement a long-running procedure, do so within the on_execute() method and in
	//   there, use the method baseStateServiceInGetUpdate(input) to get a copy of the input object
}

int PoseUpdateActivity::on_entry()
{
	CommBasicObjects::CommBasePose default_base_position;
	default_base_position.set_x(COMP->getGlobalState().getBase().getX());
	default_base_position.set_y(COMP->getGlobalState().getBase().getY());
	default_base_position.set_z(COMP->getGlobalState().getBase().getZ());

	// TODO: Set alpha, steer, turret to CommBasePose
	// default_base_position.set_base_alpha(COMP->getGlobalState().getBase().getBase_a());
	// default_base_position.set_steer_alpha(COMP->getGlobalState().getBase().getSteer_a());
	// default_base_position.set_turret_alpha(COMP->getGlobalState().getBase().getTurret_a());

	CommBasicObjects::CommBaseVelocity zero_velocity;
	zero_velocity.setVX(0);
	zero_velocity.setVY(0);
	zero_velocity.setVZ(0);
	zero_velocity.setWX(0);
	zero_velocity.setWY(0);
	zero_velocity.setWZ(0);

	default_baseState.setBasePose(default_base_position);
	default_baseState.set_base_velocity(zero_velocity);
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}
int PoseUpdateActivity::on_execute()
{
	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel
	
	// only push state when component is activated
	COMP->stateSlave->acquire("nonneutral");

	CommManipulatorObjects::CommManipulatorState state;
	CommManipulatorObjects::CommMobileManipulatorState mobileState;

	// get current state from Manipulator
	UR->getCurrentState(state);
	mobileState.set_manipulator_state(state);

	// if baseServer is activated in the ini file get the baseState
	if (COMP->getGlobalState().getBase().getOn_base()) {
		Smart::StatusCode status;
		CommBasicObjects::CommBaseState baseState;


		status = baseStateServiceInGetUpdate(baseState);

		if (status == Smart::SMART_OK) {
			mobileState.set_base_state(baseState);
		} else {
			mobileState.set_base_state(default_baseState);
			mobileState.set_valid(false);
		}
	} else {
		mobileState.set_base_state(default_baseState);
	}

	this->posePushServerPut(mobileState);


	CommBasicObjects::CommDigitalInputEventState ioState;
	std::vector<bool> digitalIO = UR->getDigitalInputValues();

	//pass values to the event server!

	for(unsigned int i=0;i<digitalIO.size();i++){
		ioState.getDigitalInputValuesRef().push_back(digitalIO[i]);
	}
	digitalInputEventServerPut(ioState);

	// only push state when component is activated
	COMP->stateSlave->release("nonneutral");

	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int PoseUpdateActivity::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
