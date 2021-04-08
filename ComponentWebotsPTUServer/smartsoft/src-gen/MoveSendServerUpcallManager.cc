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
// Please do not modify this file. It will be re-generated
// running the code generator.
//--------------------------------------------------------------------------
#include "MoveSendServerUpcallManager.hh"

MoveSendServerUpcallManager::MoveSendServerUpcallManager(
	Smart::InputSubject<DomainPTU::CommPTUMoveRequest> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<DomainPTU::CommPTUMoveRequest>(subject, prescaleFactor)
{  }
MoveSendServerUpcallManager::~MoveSendServerUpcallManager()
{  }

void MoveSendServerUpcallManager::notify_upcalls(const DomainPTU::CommPTUMoveRequest &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_moveSendServer(input);
	}
}

void MoveSendServerUpcallManager::attach(MoveSendServerUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void MoveSendServerUpcallManager::detach(MoveSendServerUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}
