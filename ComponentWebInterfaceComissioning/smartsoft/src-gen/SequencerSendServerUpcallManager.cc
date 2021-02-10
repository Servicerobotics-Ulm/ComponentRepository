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
#include "SequencerSendServerUpcallManager.hh"

SequencerSendServerUpcallManager::SequencerSendServerUpcallManager(
	Smart::InputSubject<CommBasicObjects::CommSkillMsg> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<CommBasicObjects::CommSkillMsg>(subject, prescaleFactor)
{  }
SequencerSendServerUpcallManager::~SequencerSendServerUpcallManager()
{  }

void SequencerSendServerUpcallManager::notify_upcalls(const CommBasicObjects::CommSkillMsg &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_sequencerSendServer(input);
	}
}

void SequencerSendServerUpcallManager::attach(SequencerSendServerUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void SequencerSendServerUpcallManager::detach(SequencerSendServerUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}
