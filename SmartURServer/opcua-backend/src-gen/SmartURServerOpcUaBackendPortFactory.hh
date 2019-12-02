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

#ifndef SMARTURSERVER_OPC_UA_BACKEND_PORTFACTORY_HH_
#define SMARTURSERVER_OPC_UA_BACKEND_PORTFACTORY_HH_

// include the main component-definition class
#include "SmartURServerPortFactoryInterface.hh"

#include <thread>
#include <chrono>

// include SeRoNetSDK library
#include <SeRoNetSDK/SeRoNet/Utils/Task.hpp>
#include <SeRoNetSDK/SeRoNet/Utils/Component.hpp>

class SmartURServerOpcUaBackendPortFactory: public SmartURServerPortFactoryInterface
{
private:
	// internal component instance
	SeRoNet::Utils::Component *componentImpl;
	
	// component thread
	std::thread component_thread;
	
	// internal component thread method
	int task_execution();
public:
	SmartURServerOpcUaBackendPortFactory();
	virtual ~SmartURServerOpcUaBackendPortFactory();

	virtual void initialize(SmartURServer *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseStateServiceIn() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommMobileLaserScan> * createLaserQueryServiceReq() override;
	
	virtual Smart::IPushServerPattern<CommManipulatorObjects::CommMobileManipulatorState> * createPosePushServer(const std::string &serviceName) override;
	virtual Smart::IQueryServerPattern<CommManipulatorObjects::CommManipulatorId, DomainVision::Comm3dPointCloud> * createScan3dQueryServer(const std::string &serviceName) override;
	virtual Smart::IQueryServerPattern<CommRobotinoObjects::CommRobotinoIOValues, CommRobotinoObjects::CommRobotinoIOValues> * createIoQueryServer(const std::string &serviceName) override;
	virtual Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState> * createPoseQueryServer(const std::string &serviceName) override;
	virtual Smart::IEventServerPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommManipulatorId, CommManipulatorObjects::CommScanEventState> * createScan3dEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommVoid, CommManipulatorObjects::CommManipulatorId, CommManipulatorObjects::CommScanEventState>> scan3dEventServerEventTestHandler) override;
	virtual Smart::IEventServerPattern<CommRobotinoObjects::CommDigitalInputEventParameter, CommRobotinoObjects::CommDigitalInputEventResult, CommRobotinoObjects::CommDigitalInputEventState> * createDigitalInputEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommRobotinoObjects::CommDigitalInputEventParameter, CommRobotinoObjects::CommDigitalInputEventResult, CommRobotinoObjects::CommDigitalInputEventState>> digitalInputEventServerEventTestHandler) override;
	virtual Smart::ISendServerPattern<CommManipulatorObjects::CommManipulatorTrajectory> * createTrajectorySendServer(const std::string &serviceName) override;
	virtual Smart::IEventServerPattern<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult, CommManipulatorObjects::CommManipulatorEventState> * createManipulatorEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult, CommManipulatorObjects::CommManipulatorEventState>> manipulatorEventServiceOutEventTestHandler) override;
	virtual Smart::ISendServerPattern<CommManipulatorObjects::CommManipulatorRequestScan3d> * createRequestScan3dSendServer(const std::string &serviceName) override;
	
	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* SMARTURSERVER_SERONET_SDK_PORTFACTORY_HH_ */