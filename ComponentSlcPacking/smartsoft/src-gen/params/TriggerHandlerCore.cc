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
		case TriggerEnumerators::COMMOBJECTRECOGNITIONOBJECTS_OBJECTRECOGNITIONPARAMETER_CAPTURE:
			current_CommObjectRecognitionObjects_ObjectRecognitionParameter_CAPTURE = CommObjectRecognitionObjects_ObjectRecognitionParameter_CAPTURE_queue.front();
			CommObjectRecognitionObjects_ObjectRecognitionParameter_CAPTURE_queue.pop();
			break;
		case TriggerEnumerators::COMMOBJECTRECOGNITIONOBJECTS_OBJECTRECOGNITIONPARAMETER_RECOGNIZE:
			current_CommObjectRecognitionObjects_ObjectRecognitionParameter_RECOGNIZE = CommObjectRecognitionObjects_ObjectRecognitionParameter_RECOGNIZE_queue.front();
			CommObjectRecognitionObjects_ObjectRecognitionParameter_RECOGNIZE_queue.pop();
			break;
		}
	} // mutex release

	// now call the corresponding trigger handler
	// (releasing the mutex before, allows to store newly incoming trigger commands on the queue in parallel)
	switch(current_trigger_enumerator) {
	case TriggerEnumerators::COMMOBJECTRECOGNITIONOBJECTS_OBJECTRECOGNITIONPARAMETER_CAPTURE:
		this->handleCommObjectRecognitionObjects_ObjectRecognitionParameter_CAPTURE();
		break;
	case TriggerEnumerators::COMMOBJECTRECOGNITIONOBJECTS_OBJECTRECOGNITIONPARAMETER_RECOGNIZE:
		this->handleCommObjectRecognitionObjects_ObjectRecognitionParameter_RECOGNIZE(current_CommObjectRecognitionObjects_ObjectRecognitionParameter_RECOGNIZE.viewPointID);
		break;
	}
	return 0;
}

//
// trigger internal handler methods
//

	// handle ADDALGORITHM
	void TriggerHandlerCore::handleCommObjectRecognitionObjects_ObjectRecognitionParameter_ADDALGORITHMCore(const std::string &algorithm)
	{
		this->handleCommObjectRecognitionObjects_ObjectRecognitionParameter_ADDALGORITHM(algorithm);
	}

	// handle ADDOBJECT
	void TriggerHandlerCore::handleCommObjectRecognitionObjects_ObjectRecognitionParameter_ADDOBJECTCore(const std::string &type)
	{
		this->handleCommObjectRecognitionObjects_ObjectRecognitionParameter_ADDOBJECT(type);
	}

	// handle ADDSENSOR
	void TriggerHandlerCore::handleCommObjectRecognitionObjects_ObjectRecognitionParameter_ADDSENSORCore(const ADDSENSORType::sensorType &sensor)
	{
		this->handleCommObjectRecognitionObjects_ObjectRecognitionParameter_ADDSENSOR(sensor);
	}

	// handle BEHAVIOR
	void TriggerHandlerCore::handleCommObjectRecognitionObjects_ObjectRecognitionParameter_BEHAVIORCore(const BEHAVIORType::typeType &type)
	{
		this->handleCommObjectRecognitionObjects_ObjectRecognitionParameter_BEHAVIOR(type);
	}

	// handle CAPTURE
	void TriggerHandlerCore::handleCommObjectRecognitionObjects_ObjectRecognitionParameter_CAPTURECore()
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		CommObjectRecognitionObjects_ObjectRecognitionParameter_CAPTUREAttributes attr;
		CommObjectRecognitionObjects_ObjectRecognitionParameter_CAPTURE_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::COMMOBJECTRECOGNITIONOBJECTS_OBJECTRECOGNITIONPARAMETER_CAPTURE);
		
		// signal the task, in case it is waiting
		sema.release();
	}

	// handle DELALGORITHMS
	void TriggerHandlerCore::handleCommObjectRecognitionObjects_ObjectRecognitionParameter_DELALGORITHMSCore()
	{
		this->handleCommObjectRecognitionObjects_ObjectRecognitionParameter_DELALGORITHMS();
	}

	// handle DELOBJECTS
	void TriggerHandlerCore::handleCommObjectRecognitionObjects_ObjectRecognitionParameter_DELOBJECTSCore()
	{
		this->handleCommObjectRecognitionObjects_ObjectRecognitionParameter_DELOBJECTS();
	}

	// handle DELSENSORS
	void TriggerHandlerCore::handleCommObjectRecognitionObjects_ObjectRecognitionParameter_DELSENSORSCore()
	{
		this->handleCommObjectRecognitionObjects_ObjectRecognitionParameter_DELSENSORS();
	}

	// handle RECOGNIZE
	void TriggerHandlerCore::handleCommObjectRecognitionObjects_ObjectRecognitionParameter_RECOGNIZECore(const std::string &viewPointID)
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		CommObjectRecognitionObjects_ObjectRecognitionParameter_RECOGNIZEAttributes attr;
		attr.viewPointID = viewPointID;
		CommObjectRecognitionObjects_ObjectRecognitionParameter_RECOGNIZE_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::COMMOBJECTRECOGNITIONOBJECTS_OBJECTRECOGNITIONPARAMETER_RECOGNIZE);
		
		// signal the task, in case it is waiting
		sema.release();
	}

//
// extended trigger internal handler methods
//
