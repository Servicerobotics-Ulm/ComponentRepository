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

#include "ComponentRobotinoConveyerBeltServer_OPCUAExtension.hh"

ComponentRobotinoConveyerBeltServer_OPCUAExtension::ComponentRobotinoConveyerBeltServer_OPCUAExtension(const std::string &name)
:	extension_name(name)
{
	cancelled = false;
	COMP->addExtension(this);
}

ComponentRobotinoConveyerBeltServer_OPCUAExtension::~ComponentRobotinoConveyerBeltServer_OPCUAExtension()
{  }

int ComponentRobotinoConveyerBeltServer_OPCUAExtension::startExtensionThread()
{
	cancelled = false;
	// execute the task_execution() method in a new thread now
	extension_future = std::async(std::launch::async, &ComponentRobotinoConveyerBeltServer_OPCUAExtension::extensionExecution, this);
	return 0;
}

int ComponentRobotinoConveyerBeltServer_OPCUAExtension::stopExtensionThread(const std::chrono::steady_clock::duration &timeoutTime)
{
	cancelled = true;
	// wait on extension thread to exit
	if (extension_future.wait_for(timeoutTime) == std::future_status::timeout) {
		return -1;
    }
	return 0;
}

void ComponentRobotinoConveyerBeltServer_OPCUAExtension::loadParameters(const SmartACE::SmartIniParameter &parameter)
{
	// no-op
}

int ComponentRobotinoConveyerBeltServer_OPCUAExtension::onStartup()
{
	// default implementation just starts the internal thread
	return startExtensionThread();
}

int ComponentRobotinoConveyerBeltServer_OPCUAExtension::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	// default implementation stops the internal thread waiting up to timeoutTime until it is stopped (or a timeout occurs)
	return stopExtensionThread(timeoutTime);
}
