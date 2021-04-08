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
#include "MoveSendHandler.hh"
#include "ComponentWebotsPTUServer.hh"
#include <iostream>

MoveSendHandler::MoveSendHandler(Smart::InputSubject<DomainPTU::CommPTUMoveRequest> *subject, const int &prescaleFactor)
:	MoveSendHandlerCore(subject, prescaleFactor)
{
	std::cout << "constructor MoveSendHandler\n";
}
MoveSendHandler::~MoveSendHandler() 
{
	std::cout << "destructor MoveSendHandler\n";
}

void MoveSendHandler::on_moveSendServer(const DomainPTU::CommPTUMoveRequest &input)
{
	DomainPTU::PTUGoalEventState eventState;

	std::cout << __LINE__ << std::endl;
	if (Smart::SMART_OK == COMP->stateSlave->tryAcquire("nonneutral"))
	{

		std::cout << __LINE__ << std::endl;
		if(Smart::SMART_OK == COMP->stateSlave->tryAcquire("move")) {
			eventState.set_state(DomainPTU::PTUMoveStatus::GOAL_NOT_REACHED);
			 COMP->goalEventServer->put(eventState);
			 std::cout << __LINE__ << std::endl;
		COMP->stateSlave->release("move");
		}
		std::cout << __LINE__ << std::endl;
		COMP->ptuTask->setGoal(input);

		COMP->stateSlave->release("nonneutral");
	} else {
//		std::cout<<"PTU in neutral state --> no move request allowed!"<<std::endl;
	}
}
