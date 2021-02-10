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
#ifndef _LOCALIZATIONEVENTSERVICEIN_UPCALL_MANAGER_HH
#define _LOCALIZATIONEVENTSERVICEIN_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "LocalizationEventServiceInUpcallInterface.hh"

/** LocalizationEventServiceInUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort LocalizationEventServiceIn and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class LocalizationEventServiceInUpcallManager
:	public Smart::IInputHandler<Smart::EventInputType<CommLocalizationObjects::CommLocalizationEventResult>>
{
private:
	// list of associated updalls
	std::list<LocalizationEventServiceInUpcallInterface*> upcalls;

	// call the on_LocalizationEventServiceIn of all the attached LocalizationEventServiceInUpcallInterfaces
	void notify_upcalls(const Smart::EventInputType<CommLocalizationObjects::CommLocalizationEventResult> &input);
	
protected:
	virtual void handle_input(const Smart::EventInputType<CommLocalizationObjects::CommLocalizationEventResult> &input) {
		// relay input-handling to all attached LocalizationEventServiceInUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	LocalizationEventServiceInUpcallManager(
		Smart::InputSubject<Smart::EventInputType<CommLocalizationObjects::CommLocalizationEventResult>> *subject,
		const int &prescaleFactor=1
	);
	virtual ~LocalizationEventServiceInUpcallManager();
	
	void attach(LocalizationEventServiceInUpcallInterface *upcall);
	void detach(LocalizationEventServiceInUpcallInterface *upcall);
};

#endif
