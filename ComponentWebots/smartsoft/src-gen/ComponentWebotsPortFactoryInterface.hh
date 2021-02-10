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

#ifndef COMPONENTWEBOTS_PORTFACTORYINTERFACE_HH_
#define COMPONENTWEBOTS_PORTFACTORYINTERFACE_HH_

// include communication objects

#include <chrono>

// include component's main class
#include "ComponentWebots.hh"

// forward declaration
class ComponentWebots;

class ComponentWebotsPortFactoryInterface {
public:
	ComponentWebotsPortFactoryInterface() { };
	virtual ~ComponentWebotsPortFactoryInterface() { };

	virtual void initialize(ComponentWebots *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTWEBOTS_PORTFACTORYINTERFACE_HH_ */
