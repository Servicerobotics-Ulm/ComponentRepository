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
#ifndef _BASESTATEPUSHNEWESTCLIENT_UPCALL_MANAGER_HH
#define _BASESTATEPUSHNEWESTCLIENT_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "BaseStatePushNewestClientUpcallInterface.hh"

/** BaseStatePushNewestClientUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort baseStatePushNewestClient and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class BaseStatePushNewestClientUpcallManager
:	public Smart::IInputHandler<CommBasicObjects::CommBaseState>
{
private:
	// list of associated updalls
	std::list<BaseStatePushNewestClientUpcallInterface*> upcalls;

	// call the on_baseStatePushNewestClient of all the attached BaseStatePushNewestClientUpcallInterfaces
	void notify_upcalls(const CommBasicObjects::CommBaseState &input);
	
protected:
	virtual void handle_input(const CommBasicObjects::CommBaseState &input) {
		// relay input-handling to all attached BaseStatePushNewestClientUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	BaseStatePushNewestClientUpcallManager(
		Smart::InputSubject<CommBasicObjects::CommBaseState> *subject,
		const int &prescaleFactor=1
	);
	virtual ~BaseStatePushNewestClientUpcallManager();
	
	void attach(BaseStatePushNewestClientUpcallInterface *upcall);
	void detach(BaseStatePushNewestClientUpcallInterface *upcall);
};

#endif
