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
#ifndef _PATHPLANNINGREQUESTIN_UPCALL_MANAGER_HH
#define _PATHPLANNINGREQUESTIN_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "PathPlanningRequestInUpcallInterface.hh"

/** PathPlanningRequestInUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort PathPlanningRequestIn and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class PathPlanningRequestInUpcallManager
:	public Smart::IInputHandler<CommManipulatorObjects::CommManipulatorTrajectory>
{
private:
	// list of associated updalls
	std::list<PathPlanningRequestInUpcallInterface*> upcalls;

	// call the on_PathPlanningRequestIn of all the attached PathPlanningRequestInUpcallInterfaces
	void notify_upcalls(const CommManipulatorObjects::CommManipulatorTrajectory &input);
	
protected:
	virtual void handle_input(const CommManipulatorObjects::CommManipulatorTrajectory &input) {
		// relay input-handling to all attached PathPlanningRequestInUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	PathPlanningRequestInUpcallManager(
		Smart::InputSubject<CommManipulatorObjects::CommManipulatorTrajectory> *subject,
		const int &prescaleFactor=1
	);
	virtual ~PathPlanningRequestInUpcallManager();
	
	void attach(PathPlanningRequestInUpcallInterface *upcall);
	void detach(PathPlanningRequestInUpcallInterface *upcall);
};

#endif