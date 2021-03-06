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
#include "AmclVizTask.hh"
#include "ComponentVisualization.hh"

#include <iostream>

AmclVizTask::AmclVizTask(SmartACE::SmartComponent *comp) 
:	AmclVizTaskCore(comp)
{
	std::cout << "constructor AmclVizTask\n";
}
AmclVizTask::~AmclVizTask() 
{
	std::cout << "destructor AmclVizTask\n";
}

int AmclVizTask::connectServices(){
	std::cout << "connecting to: " << COMP->connections.amclVisualizationInfoIn.serverName << "; " << COMP->connections.amclVisualizationInfoIn.serviceName << std::endl;
	Smart::StatusCode status = COMP->amclVisualizationInfoIn->connect(COMP->connections.amclVisualizationInfoIn.serverName, COMP->connections.amclVisualizationInfoIn.serviceName);
	while (status != Smart::SMART_OK) {
		usleep(500000);
		status = COMP->amclVisualizationInfoIn->connect(COMP->connections.amclVisualizationInfoIn.serverName, COMP->connections.amclVisualizationInfoIn.serviceName);
	}
	std::cout << COMP->connections.amclVisualizationInfoIn.serverName << "; " << COMP->connections.amclVisualizationInfoIn.serviceName << " connected.\n";
	return 0;
}
int AmclVizTask::disconnectServices(){
	COMP->amclVisualizationInfoIn->disconnect();
	return 0;
}

int AmclVizTask::on_entry()
{
	apv = new AmclParticleVisualization(COMP->getWindow3d(), "apv");
	COMP->amclVisualizationInfoIn->subscribe();
	return (apv != 0) ? 0 : 1;
}
int AmclVizTask::on_execute()
{
	Smart::StatusCode status = COMP->amclVisualizationInfoIn->getUpdateWait(pf_info);
	if (status == Smart::SMART_OK) {
		if(pf_info.getParticlesSize())
		apv->displayAmclInfo(pf_info);
	} else {
		std::cout << "AmclVizTask :: problem in receiving messages : " << Smart::StatusCodeConversion(status)<<std::endl;
	    apv->clear();
	}

	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int AmclVizTask::on_exit()
{
	delete apv;
	COMP->amclVisualizationInfoIn->unsubscribe();
	return 0;
}
