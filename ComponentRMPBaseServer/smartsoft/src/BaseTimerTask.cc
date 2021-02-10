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
#include "BaseTimerTask.hh"
#include "ComponentRMPBaseServer.hh"

#include <iostream>

BaseTimerTask::BaseTimerTask(SmartACE::SmartComponent *comp) 
:	BaseTimerTaskCore(comp)
{
	std::cout << "constructor BaseTimerTask\n";
}
BaseTimerTask::~BaseTimerTask() 
{
	std::cout << "destructor BaseTimerTask\n";
}



int BaseTimerTask::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}
int BaseTimerTask::on_execute()
{
	COMP->robot.update();
	return 0;
}
int BaseTimerTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
