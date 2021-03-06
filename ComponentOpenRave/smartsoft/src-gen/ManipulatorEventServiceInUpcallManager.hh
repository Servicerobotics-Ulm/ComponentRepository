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
#ifndef _MANIPULATOREVENTSERVICEIN_UPCALL_MANAGER_HH
#define _MANIPULATOREVENTSERVICEIN_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "ManipulatorEventServiceInUpcallInterface.hh"

/** ManipulatorEventServiceInUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort ManipulatorEventServiceIn and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class ManipulatorEventServiceInUpcallManager
:	public Smart::IInputHandler<Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult>>
{
private:
	// list of associated updalls
	std::list<ManipulatorEventServiceInUpcallInterface*> upcalls;

	// call the on_ManipulatorEventServiceIn of all the attached ManipulatorEventServiceInUpcallInterfaces
	void notify_upcalls(const Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult> &input);
	
protected:
	virtual void handle_input(const Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult> &input) {
		// relay input-handling to all attached ManipulatorEventServiceInUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	ManipulatorEventServiceInUpcallManager(
		Smart::InputSubject<Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult>> *subject,
		const int &prescaleFactor=1
	);
	virtual ~ManipulatorEventServiceInUpcallManager();
	
	void attach(ManipulatorEventServiceInUpcallInterface *upcall);
	void detach(ManipulatorEventServiceInUpcallInterface *upcall);
};

#endif
