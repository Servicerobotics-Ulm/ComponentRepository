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
#ifndef _SEQUENCERSENDSERVER_UPCALL_MANAGER_HH
#define _SEQUENCERSENDSERVER_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "SequencerSendServerUpcallInterface.hh"

/** SequencerSendServerUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort sequencerSendServer and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class SequencerSendServerUpcallManager
:	public Smart::IInputHandler<CommBasicObjects::CommSkillMsg>
{
private:
	// list of associated updalls
	std::list<SequencerSendServerUpcallInterface*> upcalls;

	// call the on_sequencerSendServer of all the attached SequencerSendServerUpcallInterfaces
	void notify_upcalls(const CommBasicObjects::CommSkillMsg &input);
	
protected:
	virtual void handle_input(const CommBasicObjects::CommSkillMsg &input) {
		// relay input-handling to all attached SequencerSendServerUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	SequencerSendServerUpcallManager(
		Smart::InputSubject<CommBasicObjects::CommSkillMsg> *subject,
		const int &prescaleFactor=1
	);
	virtual ~SequencerSendServerUpcallManager();
	
	void attach(SequencerSendServerUpcallInterface *upcall);
	void detach(SequencerSendServerUpcallInterface *upcall);
};

#endif