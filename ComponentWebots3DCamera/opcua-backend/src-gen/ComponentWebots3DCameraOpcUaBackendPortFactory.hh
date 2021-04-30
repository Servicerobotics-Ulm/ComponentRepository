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

#ifndef COMPONENTWEBOTS3DCAMERA_OPC_UA_BACKEND_PORTFACTORY_HH_
#define COMPONENTWEBOTS3DCAMERA_OPC_UA_BACKEND_PORTFACTORY_HH_

// include the main component-definition class
#include "ComponentWebots3DCameraPortFactoryInterface.hh"

#include <thread>
#include <chrono>

// include SeRoNetSDK library
#include <SeRoNetSDK/SeRoNet/Utils/Task.hpp>
#include <SeRoNetSDK/SeRoNet/Utils/Component.hpp>

class ComponentWebots3DCameraOpcUaBackendPortFactory: public ComponentWebots3DCameraPortFactoryInterface
{
private:
	// internal component instance
	SeRoNet::Utils::Component *componentImpl;
	
	// component thread
	std::thread component_thread;
	
	// internal component thread method
	int task_execution();
public:
	ComponentWebots3DCameraOpcUaBackendPortFactory();
	virtual ~ComponentWebots3DCameraOpcUaBackendPortFactory();

	virtual void initialize(ComponentWebots3DCamera *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IPushClientPattern<CommManipulatorObjects::CommMobileManipulatorState> * createUrPosePushTimedClient() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState> * createUrPoseQueryClient() override;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBasePushTimedClient() override;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommDevicePoseState> * createPtuPosePushNewestClient() override;
	
	virtual Smart::IPushServerPattern<DomainVision::CommRGBDImage> * createRGBDImagePushServiceOut(const std::string &serviceName) override;
	virtual Smart::IPushServerPattern<DomainVision::CommVideoImage> * createRGBImagePushServiceOut(const std::string &serviceName) override;
	virtual Smart::IQueryServerPattern<CommBasicObjects::CommVoid, DomainVision::CommVideoImage> * createColorImageQueryServer(const std::string &serviceName) override;
	virtual Smart::IPushServerPattern<DomainVision::CommDepthImage> * createDepthPushNewestServer(const std::string &serviceName) override;
	virtual Smart::IQueryServerPattern<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> * createImageQueryServer(const std::string &serviceName) override;
	
	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTWEBOTS3DCAMERA_SERONET_SDK_PORTFACTORY_HH_ */