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

#ifndef COMPONENTROBOTINOIRSERVER_ACE_PORTFACTORY_HH_
#define COMPONENTROBOTINOIRSERVER_ACE_PORTFACTORY_HH_

// include ACE/SmartSoft component implementation
#include "ComponentRobotinoIRServerImpl.hh"

// include the main component-definition class
#include "ComponentRobotinoIRServerPortFactoryInterface.hh"

class ComponentRobotinoIRServerAcePortFactory: public ComponentRobotinoIRServerPortFactoryInterface
{
private:
	ComponentRobotinoIRServerImpl *componentImpl;
public:
	ComponentRobotinoIRServerAcePortFactory();
	virtual ~ComponentRobotinoIRServerAcePortFactory();

	virtual void initialize(ComponentRobotinoIRServer *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseStateServiceIn() override;
	
	virtual Smart::IPushServerPattern<CommBasicObjects::CommMobileIRScan> * createAdditionalIRScanPushOut(const std::string &serviceName) override;
	virtual Smart::IPushServerPattern<CommBasicObjects::CommMobileIRScan> * createMobileIRPushOut(const std::string &serviceName) override;
	
	// get a pointer to the internal component implementation
	SmartACE::SmartComponent* getComponentImpl();

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTROBOTINOIRSERVER_ACE_PORTFACTORY_HH_ */