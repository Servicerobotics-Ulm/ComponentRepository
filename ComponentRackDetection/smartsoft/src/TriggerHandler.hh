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
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#ifndef _TRIGGERHANDLER_HH
#define _TRIGGERHANDLER_HH

#include "TriggerHandlerCore.hh"

class TriggerHandler: public TriggerHandlerCore 
{
public:
	TriggerHandler() {  }
	virtual ~TriggerHandler() {  }
	
	// trigger user methods
	virtual void handleCommObjectRecognitionObjects_ObjectRecognitionParameter_ADDALGORITHM(const std::string &algorithm);
	virtual void handleCommObjectRecognitionObjects_ObjectRecognitionParameter_ADDOBJECT(const std::string &type);
	virtual void handleCommObjectRecognitionObjects_ObjectRecognitionParameter_ADDSENSOR(const ADDSENSORType::sensorType &sensor);
	virtual void handleCommObjectRecognitionObjects_ObjectRecognitionParameter_BEHAVIOR(const BEHAVIORType::typeType &type);
	virtual void handleCommObjectRecognitionObjects_ObjectRecognitionParameter_CAPTURE();
	virtual void handleCommObjectRecognitionObjects_ObjectRecognitionParameter_DELALGORITHMS();
	virtual void handleCommObjectRecognitionObjects_ObjectRecognitionParameter_DELOBJECTS();
	virtual void handleCommObjectRecognitionObjects_ObjectRecognitionParameter_DELSENSORS();
	virtual void handleCommObjectRecognitionObjects_ObjectRecognitionParameter_RECOGNIZE(const std::string &viewPointID);
};

#endif // _TRIGGERHANDLER_HH
