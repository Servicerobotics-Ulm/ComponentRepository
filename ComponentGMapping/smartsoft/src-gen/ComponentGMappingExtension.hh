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

#ifndef COMPONENTGMAPPING_EXTENSION_HH_
#define COMPONENTGMAPPING_EXTENSION_HH_

#include <string>
#include <atomic>
#include <future>
#include <chrono>

// include component's main class
#include "ComponentGMapping.hh"

// forward declaration
class ComponentGMapping;

class ComponentGMappingExtension {
private:
	std::string extension_name;
	std::future<int> extension_future;

protected:
	/// use this variable within extensionExecution() to check if the thread is commanded to shutdown
	std::atomic<bool> cancelled;
	
	/// use this method to start internal extension thread
	virtual int startExtensionThread();
	
	/// implement this method in derived classes
	virtual int extensionExecution() = 0;
	
	/// use this method to start internal extension thread
	virtual int stopExtensionThread(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2));
public:
	ComponentGMappingExtension(const std::string &name);
	virtual ~ComponentGMappingExtension();

	inline std::string getName() const {
		return extension_name;
	}

	virtual void loadParameters(const SmartACE::SmartIniParameter &parameter);
	virtual void initialize(ComponentGMapping *component, int argc, char* argv[]) = 0;
	virtual int onStartup();

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2));
	virtual void destroy() = 0;

};

#endif /* COMPONENTGMAPPING_EXTENSION_HH_ */
