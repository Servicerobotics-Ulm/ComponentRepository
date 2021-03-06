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
	virtual void handleCommBasicObjects_VisualizationParams_SHOW_BASE1();
	virtual void handleCommBasicObjects_VisualizationParams_SHOW_DEPTHIMAGE1();
	virtual void handleCommBasicObjects_VisualizationParams_SHOW_IR1();
	virtual void handleCommBasicObjects_VisualizationParams_SHOW_LASER1();
	virtual void handleCommBasicObjects_VisualizationParams_SHOW_LASER2();
	virtual void handleCommBasicObjects_VisualizationParams_SHOW_LASER3();
	virtual void handleCommBasicObjects_VisualizationParams_SHOW_MAP1();
	virtual void handleCommBasicObjects_VisualizationParams_SHOW_RGB1();
	virtual void handleCommBasicObjects_VisualizationParams_SHOW_RGBD1();
	virtual void handleCommBasicObjects_VisualizationParams_STOP_BASE1();
	virtual void handleCommBasicObjects_VisualizationParams_STOP_DEPTHIMAGE1();
	virtual void handleCommBasicObjects_VisualizationParams_STOP_IR1();
	virtual void handleCommBasicObjects_VisualizationParams_STOP_LASER1();
	virtual void handleCommBasicObjects_VisualizationParams_STOP_LASER2();
	virtual void handleCommBasicObjects_VisualizationParams_STOP_LASER3();
	virtual void handleCommBasicObjects_VisualizationParams_STOP_MAP1();
	virtual void handleCommBasicObjects_VisualizationParams_STOP_RGB1();
	virtual void handleCommBasicObjects_VisualizationParams_STOP_RGBD1();
};

#endif // _TRIGGERHANDLER_HH
