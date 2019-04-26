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
#ifndef _LASERCLIENT2_UPCALL_MANAGER_HH
#define _LASERCLIENT2_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "LaserClient2UpcallInterface.hh"

/** LaserClient2UpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort laserClient2 and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class LaserClient2UpcallManager
:	public Smart::IInputHandler<CommBasicObjects::CommMobileLaserScan>
{
private:
	// list of associated updalls
	std::list<LaserClient2UpcallInterface*> upcalls;

	// call the on_laserClient2 of all the attached LaserClient2UpcallInterfaces
	void notify_upcalls(const CommBasicObjects::CommMobileLaserScan &input);
	
protected:
	virtual void handle_input(const CommBasicObjects::CommMobileLaserScan &input) {
		// relay input-handling to all attached LaserClient2UpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	LaserClient2UpcallManager(
		Smart::InputSubject<CommBasicObjects::CommMobileLaserScan> *subject,
		const int &prescaleFactor=1
	);
	virtual ~LaserClient2UpcallManager();
	
	void attach(LaserClient2UpcallInterface *upcall);
	void detach(LaserClient2UpcallInterface *upcall);
};

#endif