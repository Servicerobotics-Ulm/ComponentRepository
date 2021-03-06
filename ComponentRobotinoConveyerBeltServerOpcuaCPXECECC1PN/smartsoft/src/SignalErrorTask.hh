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
#ifndef _SIGNALERRORTASK_HH
#define _SIGNALERRORTASK_HH

#include "SignalErrorTaskCore.hh"

class SignalErrorTask  : public SignalErrorTaskCore
{
private:
	bool blink;
	SmartACE::SmartMutex lock;


	unsigned int digital_output_bit;

	void queryDigitalOutput(const unsigned int & bit, const bool & value);

public:
	SignalErrorTask(SmartACE::SmartComponent *comp);
	virtual ~SignalErrorTask();
	
	void disableErrorSignal();

	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();
};

#endif
