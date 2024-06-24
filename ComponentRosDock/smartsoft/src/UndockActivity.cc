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
#include "UndockActivity.hh"
#include "ComponentRosDock.hh"

#include <iostream>

UndockActivity::UndockActivity(SmartACE::SmartComponent *comp) 
:	UndockActivityCore(comp)
{
	std::cout << "constructor UndockActivity\n";
}
UndockActivity::~UndockActivity() 
{
	std::cout << "destructor UndockActivity\n";
}

void UndockActivity::undock()
{
	std::unique_lock<std::mutex> lck (mtx);

	std_msgs::String undock_goal;
	undock_goal.data = "station_charger";

	std::cout << "publishing undock goal: " << undock_goal << std::endl;
	COMP -> rosPorts -> undock_action_goal.publish(undock_goal);
	undocking = true;
}

void UndockActivity::undock_action_result_cb(const std_msgs::String::ConstPtr &msg)
{
	std::unique_lock<std::mutex> lck (mtx);

	std::string::size_type loc = msg->data.find( "successful", 0 );
	if( loc != std::string::npos )
	{
		std::cout << "undocking succeeded: " << msg->data << std::endl;
		CommNavigationObjects::CommDockingEventState eventState;
		eventState.setNewState(CommNavigationObjects::DockingEventType::UN_DOCKING_DONE);
		COMP->robotDockingEventServiceOut->put(eventState);

	}
	else
	{
		std::cout << "undocking did not succeed: " << msg->data << std::endl;
		CommNavigationObjects::CommDockingEventState eventState;
		eventState.setNewState(CommNavigationObjects::DockingEventType::UN_DOCKING_ERROR);
		COMP->robotDockingEventServiceOut->put(eventState);
	}

	this->stop();
	std::cout << "undocking false" << std::endl;
	undocking = false;
}
void UndockActivity::on_BaseStateServiceIn(const CommBasicObjects::CommBaseState &input)
{
	// upcall triggered from InputPort BaseStateServiceIn
	// - use a local mutex here, because this upcal is called asynchroneously from outside of this task
	// - do not use longer blocking calls here since this upcall blocks the InputPort BaseStateServiceIn
	// - if you need to implement a long-running procedure, do so within the on_execute() method and in
	//   there, use the method baseStateServiceInGetUpdate(input) to get a copy of the input object
}

int UndockActivity::on_entry()
{
	std::cout << "entry UndockActivity\n";

	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}
int UndockActivity::on_execute()
{
	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel
	
	// to get the incoming data, use this methods:
	Smart::StatusCode status;
	CommBasicObjects::CommBaseState baseStateServiceInObject;
	status = this->baseStateServiceInGetUpdate(baseStateServiceInObject);
	if(status != Smart::SMART_OK) {
		std::cerr << status << std::endl;
		// return 0;
	} else {
		//std::cout << "received: " << baseStateServiceInObject << std::endl;
	}

	if (!undocking)
	{
		std::cout << "Undock! " << std::endl;
		undock();
	}
	else
		std::cout << "Waiting for undocking finished." << std::endl;
	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int UndockActivity::on_exit()
{
	std::cout << "exit UndockActivity\n";

	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
