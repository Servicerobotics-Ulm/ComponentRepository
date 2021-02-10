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

#ifndef COMPONENTWEBINTERFACECOMISSIONING_PORTFACTORYINTERFACE_HH_
#define COMPONENTWEBINTERFACECOMISSIONING_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommBasicObjects/CommSkillMsg.hh>
#include <CommBasicObjects/CommSkillMsgACE.hh>

#include <chrono>

// include component's main class
#include "ComponentWebInterfaceComissioning.hh"

// forward declaration
class ComponentWebInterfaceComissioning;

class ComponentWebInterfaceComissioningPortFactoryInterface {
public:
	ComponentWebInterfaceComissioningPortFactoryInterface() { };
	virtual ~ComponentWebInterfaceComissioningPortFactoryInterface() { };

	virtual void initialize(ComponentWebInterfaceComissioning *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	
	virtual Smart::IEventServerPattern<CommBasicObjects::CommSkillMsg, CommBasicObjects::CommSkillMsg, CommBasicObjects::CommSkillMsg> * createSequencerEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommSkillMsg, CommBasicObjects::CommSkillMsg, CommBasicObjects::CommSkillMsg>> sequencerEventServerEventTestHandler) = 0;
	virtual Smart::ISendServerPattern<CommBasicObjects::CommSkillMsg> * createSequencerSendServer(const std::string &serviceName) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTWEBINTERFACECOMISSIONING_PORTFACTORYINTERFACE_HH_ */