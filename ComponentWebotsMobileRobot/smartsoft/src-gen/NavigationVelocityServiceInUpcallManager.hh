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
#ifndef _NAVIGATIONVELOCITYSERVICEIN_UPCALL_MANAGER_HH
#define _NAVIGATIONVELOCITYSERVICEIN_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "NavigationVelocityServiceInUpcallInterface.hh"

/** NavigationVelocityServiceInUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort NavigationVelocityServiceIn and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class NavigationVelocityServiceInUpcallManager
:	public Smart::IInputHandler<CommBasicObjects::CommNavigationVelocity>
{
private:
	// list of associated updalls
	std::list<NavigationVelocityServiceInUpcallInterface*> upcalls;

	// call the on_NavigationVelocityServiceIn of all the attached NavigationVelocityServiceInUpcallInterfaces
	void notify_upcalls(const CommBasicObjects::CommNavigationVelocity &input);
	
protected:
	virtual void handle_input(const CommBasicObjects::CommNavigationVelocity &input) {
		// relay input-handling to all attached NavigationVelocityServiceInUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	NavigationVelocityServiceInUpcallManager(
		Smart::InputSubject<CommBasicObjects::CommNavigationVelocity> *subject,
		const int &prescaleFactor=1
	);
	virtual ~NavigationVelocityServiceInUpcallManager();
	
	void attach(NavigationVelocityServiceInUpcallInterface *upcall);
	void detach(NavigationVelocityServiceInUpcallInterface *upcall);
};

#endif
