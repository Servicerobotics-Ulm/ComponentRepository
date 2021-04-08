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
#ifndef _MOVESENDSERVER_UPCALL_MANAGER_HH
#define _MOVESENDSERVER_UPCALL_MANAGER_HH

#include <list>
#include "aceSmartSoft.hh"
#include "MoveSendServerUpcallInterface.hh"

/** MoveSendServerUpcallManager connects input-handling with Upcall propagation
 *
 * This class implements an InputHandler for the InputPort moveSendServer and propagates the handling 
 * of incoming data to all associated (i.e. attached) Upcalls.
 */
class MoveSendServerUpcallManager
:	public Smart::IInputHandler<DomainPTU::CommPTUMoveRequest>
{
private:
	// list of associated updalls
	std::list<MoveSendServerUpcallInterface*> upcalls;

	// call the on_moveSendServer of all the attached MoveSendServerUpcallInterfaces
	void notify_upcalls(const DomainPTU::CommPTUMoveRequest &input);
	
protected:
	virtual void handle_input(const DomainPTU::CommPTUMoveRequest &input) {
		// relay input-handling to all attached MoveSendServerUpcallInterfaces
		this->notify_upcalls(input);
	}
public:
	MoveSendServerUpcallManager(
		Smart::InputSubject<DomainPTU::CommPTUMoveRequest> *subject,
		const int &prescaleFactor=1
	);
	virtual ~MoveSendServerUpcallManager();
	
	void attach(MoveSendServerUpcallInterface *upcall);
	void detach(MoveSendServerUpcallInterface *upcall);
};

#endif
