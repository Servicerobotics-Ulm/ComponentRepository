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

#ifndef COMPONENTLASERLMS1XX_PORTFACTORYINTERFACE_HH_
#define COMPONENTLASERLMS1XX_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBaseStateACE.hh>
#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommMobileLaserScanACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

#include <chrono>

// include component's main class
#include "ComponentLaserLMS1xx.hh"

// forward declaration
class ComponentLaserLMS1xx;

class ComponentLaserLMS1xxPortFactoryInterface {
public:
	ComponentLaserLMS1xxPortFactoryInterface() { };
	virtual ~ComponentLaserLMS1xxPortFactoryInterface() { };

	virtual void initialize(ComponentLaserLMS1xx *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseStateServiceIn() = 0;
	
	virtual Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommMobileLaserScan> * createLaserQueryServiceAnsw(const std::string &serviceName) = 0;
	virtual Smart::IPushServerPattern<CommBasicObjects::CommMobileLaserScan> * createLaserServiceOut(const std::string &serviceName) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTLASERLMS1XX_PORTFACTORYINTERFACE_HH_ */
