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

#ifndef COMPONENTPLAYERSTAGESIMULATOR_PORTFACTORYINTERFACE_HH_
#define COMPONENTPLAYERSTAGESIMULATOR_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommBasicObjects/CommBasePositionUpdate.hh>
#include <CommBasicObjects/CommBasePositionUpdateACE.hh>
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBaseStateACE.hh>
#include <CommBasicObjects/CommBatteryEvent.hh>
#include <CommBasicObjects/CommBatteryEventACE.hh>
#include <CommBasicObjects/CommBatteryParameter.hh>
#include <CommBasicObjects/CommBatteryParameterACE.hh>
#include <CommBasicObjects/CommBatteryState.hh>
#include <CommBasicObjects/CommBatteryStateACE.hh>
#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommMobileLaserScanACE.hh>
#include <CommBasicObjects/CommNavigationVelocity.hh>
#include <CommBasicObjects/CommNavigationVelocityACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

#include <chrono>

// include component's main class
#include "ComponentPlayerStageSimulator.hh"

// forward declaration
class ComponentPlayerStageSimulator;

class ComponentPlayerStageSimulatorPortFactoryInterface {
public:
	ComponentPlayerStageSimulatorPortFactoryInterface() { };
	virtual ~ComponentPlayerStageSimulatorPortFactoryInterface() { };

	virtual void initialize(ComponentPlayerStageSimulator *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	
	virtual Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState> * createBaseStateAnswerer(const std::string &serviceName) = 0;
	virtual Smart::IPushServerPattern<CommBasicObjects::CommBaseState> * createBaseStateServiceOut(const std::string &serviceName) = 0;
	virtual Smart::IEventServerPattern<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState> * createBatteryEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState>> batteryEventServiceOutEventTestHandler) = 0;
	virtual Smart::IPushServerPattern<CommBasicObjects::CommMobileLaserScan> * createLaserServiceOut(const std::string &serviceName) = 0;
	virtual Smart::ISendServerPattern<CommBasicObjects::CommBasePositionUpdate> * createLocalizationUpdateServiceIn(const std::string &serviceName) = 0;
	virtual Smart::ISendServerPattern<CommBasicObjects::CommNavigationVelocity> * createNavigationVelocityServiceIn(const std::string &serviceName) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTPLAYERSTAGESIMULATOR_PORTFACTORYINTERFACE_HH_ */
