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
#ifndef _KEYBOARDINPUTTASK_HH
#define _KEYBOARDINPUTTASK_HH

#include "KeyboardInputTaskCore.hh"

class KeyboardInputTask  : public KeyboardInputTaskCore
{
private:
public:
	KeyboardInputTask(SmartACE::SmartComponent *comp);
	virtual ~KeyboardInputTask();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();

	double x;
	double omega;
};

#endif
