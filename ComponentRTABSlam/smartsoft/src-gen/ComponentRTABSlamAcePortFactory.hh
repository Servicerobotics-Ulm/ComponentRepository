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

#ifndef COMPONENTRTABSLAM_ACE_PORTFACTORY_HH_
#define COMPONENTRTABSLAM_ACE_PORTFACTORY_HH_

// include ACE/SmartSoft component implementation
#include "ComponentRTABSlamImpl.hh"

// include the main component-definition class
#include "ComponentRTABSlamPortFactoryInterface.hh"

class ComponentRTABSlamAcePortFactory: public ComponentRTABSlamPortFactoryInterface
{
private:
	ComponentRTABSlamImpl *componentImpl;
public:
	ComponentRTABSlamAcePortFactory();
	virtual ~ComponentRTABSlamAcePortFactory();

	virtual void initialize(ComponentRTABSlam *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseStatePushNewestClient() override;
	virtual Smart::IPushClientPattern<DomainVision::CommRGBDImage> * createRgbd_client() override;
	
	
	// get a pointer to the internal component implementation
	SmartACE::SmartComponent* getComponentImpl();

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTRTABSLAM_ACE_PORTFACTORY_HH_ */