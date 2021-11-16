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

#include "TriggerHandlerCore.hh"

int TriggerHandlerCore::on_execute()
{
	// blocking wait until some active trigger request(s) come in
	sema.acquire();
	
	{
		SmartACE::SmartGuard g(mutex);
		
		// get the top trigger from the queue
		current_trigger_enumerator = trigger_queue.front();
		trigger_queue.pop();
		
		// retrieve the corresponding trigger attributes from the trigger specific queue
		switch(current_trigger_enumerator) {
		case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_CLEAR_PCS:
			current_CommManipulatorObjects_ManipulatorParameter_CLEAR_PCS = CommManipulatorObjects_ManipulatorParameter_CLEAR_PCS_queue.front();
			CommManipulatorObjects_ManipulatorParameter_CLEAR_PCS_queue.pop();
			break;
		case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_MOVE_CIRCULAR:
			current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR = CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR_queue.front();
			CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR_queue.pop();
			break;
		case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_MOVE_LINEAR:
			current_CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR = CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR_queue.front();
			CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR_queue.pop();
			break;
		case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_MOVE_PATH:
			current_CommManipulatorObjects_ManipulatorParameter_MOVE_PATH = CommManipulatorObjects_ManipulatorParameter_MOVE_PATH_queue.front();
			CommManipulatorObjects_ManipulatorParameter_MOVE_PATH_queue.pop();
			break;
		case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_SET_PCS:
			current_CommManipulatorObjects_ManipulatorParameter_SET_PCS = CommManipulatorObjects_ManipulatorParameter_SET_PCS_queue.front();
			CommManipulatorObjects_ManipulatorParameter_SET_PCS_queue.pop();
			break;
		case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_SET_TCP:
			current_CommManipulatorObjects_ManipulatorParameter_SET_TCP = CommManipulatorObjects_ManipulatorParameter_SET_TCP_queue.front();
			CommManipulatorObjects_ManipulatorParameter_SET_TCP_queue.pop();
			break;
		}
	} // mutex release

	// now call the corresponding trigger handler
	// (releasing the mutex before, allows to store newly incoming trigger commands on the queue in parallel)
	switch(current_trigger_enumerator) {
	case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_CLEAR_PCS:
		this->handleCommManipulatorObjects_ManipulatorParameter_CLEAR_PCS();
		break;
	case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_MOVE_CIRCULAR:
		this->handleCommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR(current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.viaX, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.viaY, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.viaZ, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.viaRX, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.viaRY, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.viaRZ, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.goalX, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.goalY, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.goalZ, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.goalRX, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.goalRY, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.goalRZ, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.acc, current_CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR.speed);
		break;
	case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_MOVE_LINEAR:
		this->handleCommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR(current_CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR.goalX, current_CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR.goalY, current_CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR.goalZ, current_CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR.goalRX, current_CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR.goalRY, current_CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR.goalRZ, current_CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR.acc, current_CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR.speed);
		break;
	case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_MOVE_PATH:
		this->handleCommManipulatorObjects_ManipulatorParameter_MOVE_PATH(current_CommManipulatorObjects_ManipulatorParameter_MOVE_PATH.pathID, current_CommManipulatorObjects_ManipulatorParameter_MOVE_PATH.overwriteSpeed, current_CommManipulatorObjects_ManipulatorParameter_MOVE_PATH.acc, current_CommManipulatorObjects_ManipulatorParameter_MOVE_PATH.speed);
		break;
	case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_SET_PCS:
		this->handleCommManipulatorObjects_ManipulatorParameter_SET_PCS(current_CommManipulatorObjects_ManipulatorParameter_SET_PCS.x, current_CommManipulatorObjects_ManipulatorParameter_SET_PCS.y, current_CommManipulatorObjects_ManipulatorParameter_SET_PCS.z, current_CommManipulatorObjects_ManipulatorParameter_SET_PCS.rX, current_CommManipulatorObjects_ManipulatorParameter_SET_PCS.rY, current_CommManipulatorObjects_ManipulatorParameter_SET_PCS.rZ);
		break;
	case TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_SET_TCP:
		this->handleCommManipulatorObjects_ManipulatorParameter_SET_TCP(current_CommManipulatorObjects_ManipulatorParameter_SET_TCP.x, current_CommManipulatorObjects_ManipulatorParameter_SET_TCP.y, current_CommManipulatorObjects_ManipulatorParameter_SET_TCP.z, current_CommManipulatorObjects_ManipulatorParameter_SET_TCP.rX, current_CommManipulatorObjects_ManipulatorParameter_SET_TCP.rY, current_CommManipulatorObjects_ManipulatorParameter_SET_TCP.rZ);
		break;
	}
	return 0;
}

//
// trigger internal handler methods
//

	// handle ADD_TCP
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_ADD_TCPCore(const std::string &name, const double &x, const double &y, const double &z, const double &rX, const double &rY, const double &rZ)
	{
		this->handleCommManipulatorObjects_ManipulatorParameter_ADD_TCP(name, x, y, z, rX, rY, rZ);
	}

	// handle ADD_TOOL
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_ADD_TOOLCore(const std::string &name, const std::list<float> &center_of_grav, const float &weight, const std::list<float> &inertia)
	{
		this->handleCommManipulatorObjects_ManipulatorParameter_ADD_TOOL(name, center_of_grav, weight, inertia);
	}

	// handle CANCEL_MOTION
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_CANCEL_MOTIONCore()
	{
		this->handleCommManipulatorObjects_ManipulatorParameter_CANCEL_MOTION();
	}

	// handle CLEAR_PCS
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_CLEAR_PCSCore()
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		CommManipulatorObjects_ManipulatorParameter_CLEAR_PCSAttributes attr;
		CommManipulatorObjects_ManipulatorParameter_CLEAR_PCS_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_CLEAR_PCS);
		
		// signal the task, in case it is waiting
		sema.release();
	}

	// handle LOAD_PROGRAM
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_LOAD_PROGRAMCore(const std::string &name)
	{
		this->handleCommManipulatorObjects_ManipulatorParameter_LOAD_PROGRAM(name);
	}

	// handle MOVE_CIRCULAR
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULARCore(const double &viaX, const double &viaY, const double &viaZ, const double &viaRX, const double &viaRY, const double &viaRZ, const double &goalX, const double &goalY, const double &goalZ, const double &goalRX, const double &goalRY, const double &goalRZ, const double &acc, const double &speed)
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULARAttributes attr;
		attr.acc = acc;
		attr.goalRX = goalRX;
		attr.goalRY = goalRY;
		attr.goalRZ = goalRZ;
		attr.goalX = goalX;
		attr.goalY = goalY;
		attr.goalZ = goalZ;
		attr.speed = speed;
		attr.viaRX = viaRX;
		attr.viaRY = viaRY;
		attr.viaRZ = viaRZ;
		attr.viaX = viaX;
		attr.viaY = viaY;
		attr.viaZ = viaZ;
		CommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULAR_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_MOVE_CIRCULAR);
		
		// signal the task, in case it is waiting
		sema.release();
	}

	// handle MOVE_LINEAR
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_MOVE_LINEARCore(const double &goalX, const double &goalY, const double &goalZ, const double &goalRX, const double &goalRY, const double &goalRZ, const double &acc, const double &speed)
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		CommManipulatorObjects_ManipulatorParameter_MOVE_LINEARAttributes attr;
		attr.acc = acc;
		attr.goalRX = goalRX;
		attr.goalRY = goalRY;
		attr.goalRZ = goalRZ;
		attr.goalX = goalX;
		attr.goalY = goalY;
		attr.goalZ = goalZ;
		attr.speed = speed;
		CommManipulatorObjects_ManipulatorParameter_MOVE_LINEAR_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_MOVE_LINEAR);
		
		// signal the task, in case it is waiting
		sema.release();
	}

	// handle MOVE_PATH
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_MOVE_PATHCore(const std::string &pathID, const bool &overwriteSpeed, const double &acc, const double &speed)
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		CommManipulatorObjects_ManipulatorParameter_MOVE_PATHAttributes attr;
		attr.acc = acc;
		attr.overwriteSpeed = overwriteSpeed;
		attr.pathID = pathID;
		attr.speed = speed;
		CommManipulatorObjects_ManipulatorParameter_MOVE_PATH_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_MOVE_PATH);
		
		// signal the task, in case it is waiting
		sema.release();
	}

	// handle SET_ACTIVE_TCP
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_SET_ACTIVE_TCPCore(const std::string &name)
	{
		this->handleCommManipulatorObjects_ManipulatorParameter_SET_ACTIVE_TCP(name);
	}

	// handle SET_ACTIVE_TOOL
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_SET_ACTIVE_TOOLCore(const std::string &name)
	{
		this->handleCommManipulatorObjects_ManipulatorParameter_SET_ACTIVE_TOOL(name);
	}

	// handle SET_PCS
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_SET_PCSCore(const double &x, const double &y, const double &z, const double &rX, const double &rY, const double &rZ)
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		CommManipulatorObjects_ManipulatorParameter_SET_PCSAttributes attr;
		attr.rX = rX;
		attr.rY = rY;
		attr.rZ = rZ;
		attr.x = x;
		attr.y = y;
		attr.z = z;
		CommManipulatorObjects_ManipulatorParameter_SET_PCS_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_SET_PCS);
		
		// signal the task, in case it is waiting
		sema.release();
	}

	// handle SET_TCP
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_SET_TCPCore(const double &x, const double &y, const double &z, const double &rX, const double &rY, const double &rZ)
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		CommManipulatorObjects_ManipulatorParameter_SET_TCPAttributes attr;
		attr.rX = rX;
		attr.rY = rY;
		attr.rZ = rZ;
		attr.x = x;
		attr.y = y;
		attr.z = z;
		CommManipulatorObjects_ManipulatorParameter_SET_TCP_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::COMMMANIPULATOROBJECTS_MANIPULATORPARAMETER_SET_TCP);
		
		// signal the task, in case it is waiting
		sema.release();
	}

	// handle START_FREEDRIVE
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_START_FREEDRIVECore()
	{
		this->handleCommManipulatorObjects_ManipulatorParameter_START_FREEDRIVE();
	}

	// handle START_PROGRAM
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_START_PROGRAMCore()
	{
		this->handleCommManipulatorObjects_ManipulatorParameter_START_PROGRAM();
	}

	// handle STOP_FREEDRIVE
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_STOP_FREEDRIVECore()
	{
		this->handleCommManipulatorObjects_ManipulatorParameter_STOP_FREEDRIVE();
	}

	// handle STOP_PROGRAM
	void TriggerHandlerCore::handleCommManipulatorObjects_ManipulatorParameter_STOP_PROGRAMCore()
	{
		this->handleCommManipulatorObjects_ManipulatorParameter_STOP_PROGRAM();
	}

//
// extended trigger internal handler methods
//
