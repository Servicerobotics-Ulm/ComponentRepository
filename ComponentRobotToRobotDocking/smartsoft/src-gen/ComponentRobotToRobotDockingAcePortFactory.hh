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

#ifndef COMPONENTROBOTTOROBOTDOCKING_ACE_PORTFACTORY_HH_
#define COMPONENTROBOTTOROBOTDOCKING_ACE_PORTFACTORY_HH_

// include ACE/SmartSoft component implementation
#include "ComponentRobotToRobotDockingImpl.hh"

// include the main component-definition class
#include "ComponentRobotToRobotDockingPortFactoryInterface.hh"

class ComponentRobotToRobotDockingAcePortFactory: public ComponentRobotToRobotDockingPortFactoryInterface
{
private:
	ComponentRobotToRobotDockingImpl *componentImpl;
public:
	ComponentRobotToRobotDockingAcePortFactory();
	virtual ~ComponentRobotToRobotDockingAcePortFactory();

	virtual void initialize(ComponentRobotToRobotDocking *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseStatePushClient() override;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * createLaserClient() override;
	virtual Smart::ISendClientPattern<CommBasicObjects::CommNavigationVelocity> * createNavVelSendClient() override;
	
	virtual Smart::IEventServerPattern<CommRobotinoObjects::CommRobotinoDockingEventParameter, CommRobotinoObjects::CommRobotinoDockingEventResult, CommRobotinoObjects::RobotinoDockingEventState> * createDockingEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommRobotinoObjects::CommRobotinoDockingEventParameter, CommRobotinoObjects::CommRobotinoDockingEventResult, CommRobotinoObjects::RobotinoDockingEventState>> dockingEventServerEventTestHandler) override;
	
	// get a pointer to the internal component implementation
	SmartACE::SmartComponent* getComponentImpl();

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTROBOTTOROBOTDOCKING_ACE_PORTFACTORY_HH_ */