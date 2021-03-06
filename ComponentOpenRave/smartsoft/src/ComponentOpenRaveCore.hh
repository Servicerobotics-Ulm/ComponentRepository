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

#ifndef _COMPONENTOPENRAVECORE_HH
#define _COMPONENTOPENRAVECORE_HH
	
#include "aceSmartSoft.hh"
#include <iostream>
#include "wrapper/ORUtil.hh"

class ComponentOpenRaveCore
{
private:

public:
	ComponentOpenRaveCore();

	/**
	 * Modes in which the manipulator can be.
	 *
	 * SEND_TRAJECTORY:	Another trajectory can be sent. The manipulator is ready.
	 * NEUTRAL:			The manipulator is busy.
	 * FAILURE:			A failure in the manipulator has occurred.
	 */
	enum ManipulatorMode {
		SEND_TRAJECTORY,
		NEUTRAL,
		FAILURE
	};

	// Holds the current state of the manipulator
	ManipulatorMode manipulatorMode;

	// Semaphore to synchronize PathPlanningSendHandler and the ManipulatorEventClientHandler
	SmartACE::SmartSemaphore ManipulatorWaitSem;

	// Mutex to secure parameters
	SmartACE::SmartMutex ParameterMutex;
	SmartACE::SmartMutex OpenRaveMutex;

	ORUtil OpenRaveWrapper;
};
	
#endif
