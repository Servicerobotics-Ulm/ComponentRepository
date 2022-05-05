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
#ifndef _ROBOTTASK_HH
#define _ROBOTTASK_HH

#include "RobotTaskCore.hh"

class RobotTask  : public RobotTaskCore
{
private:
	virtual void on_RGBDImagePushServiceIn(const DomainVision::CommRGBDImage &input);
public:
	RobotTask(SmartACE::SmartComponent *comp);
	virtual ~RobotTask();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();
};

#endif