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

#ifndef COMPONENTSLCPACKING_PORTFACTORYINTERFACE_HH_
#define COMPONENTSLCPACKING_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommObjectRecognitionObjects/CommObjectRecognitionEnvironment.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEnvironmentACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEventResult.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEventResultACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEventState.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEventStateACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionId.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionIdACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionObjectProperties.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionObjectPropertiesACE.hh>
#include <DomainVision/CommRGBDImage.hh>
#include <DomainVision/CommRGBDImageACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

#include <chrono>

// include component's main class
#include "ComponentSlcPacking.hh"

// forward declaration
class ComponentSlcPacking;

class ComponentSlcPackingPortFactoryInterface {
public:
	ComponentSlcPackingPortFactoryInterface() { };
	virtual ~ComponentSlcPackingPortFactoryInterface() { };

	virtual void initialize(ComponentSlcPacking *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IQueryClientPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> * createObjectPropertyQueryClient() = 0;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> * createRgbdQueryClient() = 0;
	
	virtual Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionEnvironment> * createEnvironmentQueryServer(const std::string &serviceName) = 0;
	virtual Smart::IEventServerPattern<CommBasicObjects::CommVoid, CommObjectRecognitionObjects::CommObjectRecognitionEventResult, CommObjectRecognitionObjects::CommObjectRecognitionEventState> * createObjectEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommVoid, CommObjectRecognitionObjects::CommObjectRecognitionEventResult, CommObjectRecognitionObjects::CommObjectRecognitionEventState>> objectEventServerEventTestHandler) = 0;
	virtual Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> * createObjectPropertyQueryServer(const std::string &serviceName) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTSLCPACKING_PORTFACTORYINTERFACE_HH_ */
