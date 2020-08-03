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

#ifndef COMPONENTFILEMOVER_OPC_UA_BACKEND_PORTFACTORY_HH_
#define COMPONENTFILEMOVER_OPC_UA_BACKEND_PORTFACTORY_HH_

// include the main component-definition class
#include "ComponentFileMoverPortFactoryInterface.hh"

#include <thread>
#include <chrono>

// include SeRoNetSDK library
#include <SeRoNetSDK/SeRoNet/Utils/Task.hpp>
#include <SeRoNetSDK/SeRoNet/Utils/Component.hpp>

class ComponentFileMoverOpcUaBackendPortFactory: public ComponentFileMoverPortFactoryInterface
{
private:
	// internal component instance
	SeRoNet::Utils::Component *componentImpl;
	
	// component thread
	std::thread component_thread;
	
	// internal component thread method
	int task_execution();
public:
	ComponentFileMoverOpcUaBackendPortFactory();
	virtual ~ComponentFileMoverOpcUaBackendPortFactory();

	virtual void initialize(ComponentFileMover *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IQueryClientPattern<CommBasicObjects::CommFileReadRequest, CommBasicObjects::CommFileReadAnswer> * createCommFileReadQueryReq() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommFileWriteRequest, CommBasicObjects::CommFileWriteAnswer> * createCommFileWriteQueryReq() override;
	
	virtual Smart::IEventServerPattern<CommBasicObjects::CommFileMoverEventParameter, CommBasicObjects::CommFileMoverEventResult, CommBasicObjects::CommFileMoverEventState> * createCommFileMoveEventOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommFileMoverEventParameter, CommBasicObjects::CommFileMoverEventResult, CommBasicObjects::CommFileMoverEventState>> commFileMoveEventOutEventTestHandler) override;
	
	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTFILEMOVER_SERONET_SDK_PORTFACTORY_HH_ */