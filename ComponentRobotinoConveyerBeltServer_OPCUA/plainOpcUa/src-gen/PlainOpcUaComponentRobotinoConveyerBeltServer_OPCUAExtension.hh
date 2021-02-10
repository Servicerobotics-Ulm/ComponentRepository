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

#ifndef PLAINOPCUA_COMPONENTROBOTINOCONVEYERBELTSERVER_OPCUA_EXTENSION_H_
#define PLAINOPCUA_COMPONENTROBOTINOCONVEYERBELTSERVER_OPCUA_EXTENSION_H_

#include "ComponentRobotinoConveyerBeltServer_OPCUAExtension.hh"

// include component's main class
#include "ComponentRobotinoConveyerBeltServer_OPCUA.hh"

// include plain OPC UA device clients
#include "OpcUaProductionStation.hh"
// include plain OPC UA status servers

class PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension : public ComponentRobotinoConveyerBeltServer_OPCUAExtension 
{
private:
	OPCUA::ProductionStation *productionStation;
	bool productionStationAutoConnect;
	std::string productionStationDeviceURI;
	std::string productionStationRootObjectPath;
public:
	PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension();
	virtual ~PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension();

	virtual void loadParameters(const SmartACE::SmartIniParameter &parameter);
	virtual void initialize(ComponentRobotinoConveyerBeltServer_OPCUA *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual int extensionExecution() override;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* PLAINOPCUA_COMPONENTROBOTINOCONVEYERBELTSERVER_OPCUA_EXTENSION_H_ */