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
#ifndef _AMCLVISUALIZATIONINFOIN_UPCALL_MANAGER_HH
#define _AMCLVISUALIZATIONINFOIN_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "AmclVisualizationInfoInUpcallInterface.hh"

/** AmclVisualizationInfoInUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort AmclVisualizationInfoIn and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class AmclVisualizationInfoInUpcallManager
:	public Smart::IInputHandler<CommLocalizationObjects::CommAmclVisualizationInfo>
{
private:
	// list of associated updalls
	std::list<AmclVisualizationInfoInUpcallInterface*> upcalls;

	// call the on_AmclVisualizationInfoIn of all the attached AmclVisualizationInfoInUpcallInterfaces
	void notify_upcalls(const CommLocalizationObjects::CommAmclVisualizationInfo &input);
	
protected:
	virtual void handle_input(const CommLocalizationObjects::CommAmclVisualizationInfo &input) {
		// relay input-handling to all attached AmclVisualizationInfoInUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	AmclVisualizationInfoInUpcallManager(
		Smart::InputSubject<CommLocalizationObjects::CommAmclVisualizationInfo> *subject,
		const int &prescaleFactor=1
	);
	virtual ~AmclVisualizationInfoInUpcallManager();
	
	void attach(AmclVisualizationInfoInUpcallInterface *upcall);
	void detach(AmclVisualizationInfoInUpcallInterface *upcall);
};

#endif
