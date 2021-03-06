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
#ifndef _RGBD_CLIENT_UPCALL_MANAGER_HH
#define _RGBD_CLIENT_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "Rgbd_clientUpcallInterface.hh"

/** Rgbd_clientUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort rgbd_client and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class Rgbd_clientUpcallManager
:	public Smart::IInputHandler<DomainVision::CommRGBDImage>
{
private:
	// list of associated updalls
	std::list<Rgbd_clientUpcallInterface*> upcalls;

	// call the on_rgbd_client of all the attached Rgbd_clientUpcallInterfaces
	void notify_upcalls(const DomainVision::CommRGBDImage &input);
	
protected:
	virtual void handle_input(const DomainVision::CommRGBDImage &input) {
		// relay input-handling to all attached Rgbd_clientUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	Rgbd_clientUpcallManager(
		Smart::InputSubject<DomainVision::CommRGBDImage> *subject,
		const int &prescaleFactor=1
	);
	virtual ~Rgbd_clientUpcallManager();
	
	void attach(Rgbd_clientUpcallInterface *upcall);
	void detach(Rgbd_clientUpcallInterface *upcall);
};

#endif
