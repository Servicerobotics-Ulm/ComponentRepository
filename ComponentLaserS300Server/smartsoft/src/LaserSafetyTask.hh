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
#ifndef _LASERSAFETYTASK_HH
#define _LASERSAFETYTASK_HH

#include "LaserSafetyTaskCore.hh"
#include "aceSmartSoft.hh"

class LaserSafetyTask  : public LaserSafetyTaskCore, public Smart::ITimerHandler
{
private:

	int laserSafetyFieldIOBit;
	int laserSafetyFieldTimerId;
	int laserSafetyFieldTimeoutSec, laserSafetyFieldTimeoutMsec;
	int laserSafetyFieldLastState;

	virtual void on_CommIOForkingServiceIn(const CommBasicObjects::CommIOValues &input);

	void timerExpired(const Smart::TimePoint &abs_time, const void * arg);
	void timerCancelled();
	void timerDeleted(const void * arg);


public:
	LaserSafetyTask(SmartACE::SmartComponent *comp);
	virtual ~LaserSafetyTask();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();
};

#endif
