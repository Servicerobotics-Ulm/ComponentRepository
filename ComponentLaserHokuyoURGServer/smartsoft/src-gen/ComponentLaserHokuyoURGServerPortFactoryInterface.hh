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

#ifndef COMPONENTLASERHOKUYOURGSERVER_PORTFACTORYINTERFACE_HH_
#define COMPONENTLASERHOKUYOURGSERVER_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBaseStateACE.hh>
#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommMobileLaserScanACE.hh>
#include <CommManipulatorObjects/CommMobileManipulatorState.hh>
#include <CommManipulatorObjects/CommMobileManipulatorStateACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

#include <chrono>

// include component's main class
#include "ComponentLaserHokuyoURGServer.hh"

// forward declaration
class ComponentLaserHokuyoURGServer;

class ComponentLaserHokuyoURGServerPortFactoryInterface {
public:
	ComponentLaserHokuyoURGServerPortFactoryInterface() { };
	virtual ~ComponentLaserHokuyoURGServerPortFactoryInterface() { };

	virtual void initialize(ComponentLaserHokuyoURGServer *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseTimedClient() = 0;
	virtual Smart::IPushClientPattern<CommManipulatorObjects::CommMobileManipulatorState> * createManipulatorTimedClient() = 0;
	
	virtual Smart::IPushServerPattern<CommBasicObjects::CommMobileLaserScan> * createLaserPushNewestServer(const std::string &serviceName) = 0;
	virtual Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommMobileLaserScan> * createLaserQueryServer(const std::string &serviceName) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTLASERHOKUYOURGSERVER_PORTFACTORYINTERFACE_HH_ */
