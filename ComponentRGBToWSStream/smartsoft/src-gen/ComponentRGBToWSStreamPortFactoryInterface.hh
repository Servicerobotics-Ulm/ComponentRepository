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

#ifndef COMPONENTRGBTOWSSTREAM_PORTFACTORYINTERFACE_HH_
#define COMPONENTRGBTOWSSTREAM_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <DomainVision/CommVideoImage.hh>
#include <DomainVision/CommVideoImageACE.hh>

#include <chrono>

// include component's main class
#include "ComponentRGBToWSStream.hh"

// forward declaration
class ComponentRGBToWSStream;

class ComponentRGBToWSStreamPortFactoryInterface {
public:
	ComponentRGBToWSStreamPortFactoryInterface() { };
	virtual ~ComponentRGBToWSStreamPortFactoryInterface() { };

	virtual void initialize(ComponentRGBToWSStream *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IPushClientPattern<DomainVision::CommVideoImage> * createVideoImageClient() = 0;
	

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTRGBTOWSSTREAM_PORTFACTORYINTERFACE_HH_ */
