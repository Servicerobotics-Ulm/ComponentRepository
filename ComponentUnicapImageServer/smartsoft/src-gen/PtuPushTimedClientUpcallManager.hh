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
#ifndef _PTUPUSHTIMEDCLIENT_UPCALL_MANAGER_HH
#define _PTUPUSHTIMEDCLIENT_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "PtuPushTimedClientUpcallInterface.hh"

/** PtuPushTimedClientUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort ptuPushTimedClient and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class PtuPushTimedClientUpcallManager
:	public Smart::IInputHandler<CommBasicObjects::CommDevicePoseState>
{
private:
	// list of associated updalls
	std::list<PtuPushTimedClientUpcallInterface*> upcalls;

	// call the on_ptuPushTimedClient of all the attached PtuPushTimedClientUpcallInterfaces
	void notify_upcalls(const CommBasicObjects::CommDevicePoseState &input);
	
protected:
	virtual void handle_input(const CommBasicObjects::CommDevicePoseState &input) {
		// relay input-handling to all attached PtuPushTimedClientUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	PtuPushTimedClientUpcallManager(
		Smart::InputSubject<CommBasicObjects::CommDevicePoseState> *subject,
		const int &prescaleFactor=1
	);
	virtual ~PtuPushTimedClientUpcallManager();
	
	void attach(PtuPushTimedClientUpcallInterface *upcall);
	void detach(PtuPushTimedClientUpcallInterface *upcall);
};

#endif
