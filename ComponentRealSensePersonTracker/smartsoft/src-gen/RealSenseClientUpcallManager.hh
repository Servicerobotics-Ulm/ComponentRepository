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
#ifndef _REALSENSECLIENT_UPCALL_MANAGER_HH
#define _REALSENSECLIENT_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "RealSenseClientUpcallInterface.hh"

/** RealSenseClientUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort RealSenseClient and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class RealSenseClientUpcallManager
:	public Smart::IInputHandler<DomainVision::CommRGBDImage>
{
private:
	// list of associated updalls
	std::list<RealSenseClientUpcallInterface*> upcalls;

	// call the on_RealSenseClient of all the attached RealSenseClientUpcallInterfaces
	void notify_upcalls(const DomainVision::CommRGBDImage &input);
	
protected:
	virtual void handle_input(const DomainVision::CommRGBDImage &input) {
		// relay input-handling to all attached RealSenseClientUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	RealSenseClientUpcallManager(
		Smart::InputSubject<DomainVision::CommRGBDImage> *subject,
		const int &prescaleFactor=1
	);
	virtual ~RealSenseClientUpcallManager();
	
	void attach(RealSenseClientUpcallInterface *upcall);
	void detach(RealSenseClientUpcallInterface *upcall);
};

#endif
