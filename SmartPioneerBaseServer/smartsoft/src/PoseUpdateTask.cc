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
#include "PoseUpdateTask.hh"
#include "SmartPioneerBaseServer.hh"

#include <iostream>

PoseUpdateTask::PoseUpdateTask(SmartACE::SmartComponent *comp) 
:	PoseUpdateTaskCore(comp)
{
	std::cout << "constructor PoseUpdateTask\n";
}
PoseUpdateTask::~PoseUpdateTask() 
{
	std::cout << "destructor PoseUpdateTask\n";
}

void PoseUpdateTask::on_update_from(const RobotTask* robotTask)
{
	mutex.acquire();

	batteryState.setChargeValue(robotTask->getBatteryVoltage());

	CommBasicObjects::CommTimeStamp time_stamp;
	CommBasicObjects::CommBasePose base_position;
	CommBasicObjects::CommBaseVelocity base_velocity;

	time_stamp.set_now(); // Set the timestamp to the current time

	//base_velocity.set_v((xSpeed + ySpeed) / 2);
	base_velocity.setVX(robotTask->getV());
	//base_velocity.set_omega_base(yawSpeed);
	base_velocity.setWZ(robotTask->getOmegaRad());

	// push the objects CommBasePose and CommBaseVelocity into the SmartSoft CommBaseState object
	base_state.set_time_stamp(time_stamp);
	//    base_state.set_base_position(base_position);
	base_state.set_base_position(robotTask->getBasePosition());
	base_state.set_base_raw_position(robotTask->getBaseRawPosition());
	base_state.set_base_velocity(base_velocity);

	mutex.release();
}

int PoseUpdateTask::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}
int PoseUpdateTask::on_execute()
{
	mutex.acquire();

	// send the CommBaseState object to the client
	Smart::StatusCode status = this->basePositionOutPut(base_state);
	if (status != Smart::SMART_OK) {
		std::cerr << "ERROR: failed to push base state (" << status << ")" << std::endl;
	}

	// send battery-state update
	this->batteryEventServerPut(batteryState);

	mutex.release();

	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int PoseUpdateTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
