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

#include "PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension.hh"

// the ace port-factory is used as a default port-mapping

// statically create a global PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension instance
static PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension extension;

PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension::PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension()
:	ComponentRobotinoConveyerBeltServer_OPCUAExtension("PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension")
{
	productionStation = 0;
	productionStationAutoConnect = false;
	productionStationDeviceURI = "opc.tcp://localhost:4840";
	productionStationRootObjectPath = "Server";
	
}

PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension::~PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension()
{  }

void PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension::loadParameters(const SmartACE::SmartIniParameter &parameter)
{
	parameter.getBoolean("ProductionStation", "autoConnect", productionStationAutoConnect);
	parameter.getString("ProductionStation", "deviceURI", productionStationDeviceURI);
	parameter.getString("ProductionStation", "rootObjectPath", productionStationRootObjectPath);
}

void PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension::initialize(ComponentRobotinoConveyerBeltServer_OPCUA *component, int argc, char* argv[])
{
	productionStation = new OPCUA::ProductionStation();
	component->productionStation = productionStation;
}

int PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension::onStartup()
{
	if(productionStationAutoConnect == true) {
		// connect OPC/UA DeviceClient ProductionStation
		productionStation->connect(productionStationDeviceURI, productionStationRootObjectPath, false);
	}
	
	
	return startExtensionThread();
}

int PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension::extensionExecution()
{
	while(cancelled == false) {
		productionStation->run_once();
	}
	return 0;
}

int PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	// disconnect OPC/UA DeviceClient ProductionStation
	productionStation->disconnect();
	
	
	return stopExtensionThread(timeoutTime);
}

void PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension::destroy()
{
	delete productionStation;
}
