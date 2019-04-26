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
#ifndef _MANIPULATORTIMEDCLIENT_UPCALL_MANAGER_HH
#define _MANIPULATORTIMEDCLIENT_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "ManipulatorTimedClientUpcallInterface.hh"

/** ManipulatorTimedClientUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort manipulatorTimedClient and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class ManipulatorTimedClientUpcallManager
:	public Smart::IInputHandler<CommManipulatorObjects::CommMobileManipulatorState>
{
private:
	// list of associated updalls
	std::list<ManipulatorTimedClientUpcallInterface*> upcalls;

	// call the on_manipulatorTimedClient of all the attached ManipulatorTimedClientUpcallInterfaces
	void notify_upcalls(const CommManipulatorObjects::CommMobileManipulatorState &input);
	
protected:
	virtual void handle_input(const CommManipulatorObjects::CommMobileManipulatorState &input) {
		// relay input-handling to all attached ManipulatorTimedClientUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	ManipulatorTimedClientUpcallManager(
		Smart::InputSubject<CommManipulatorObjects::CommMobileManipulatorState> *subject,
		const int &prescaleFactor=1
	);
	virtual ~ManipulatorTimedClientUpcallManager();
	
	void attach(ManipulatorTimedClientUpcallInterface *upcall);
	void detach(ManipulatorTimedClientUpcallInterface *upcall);
};

#endif