//--------------------------------------------------------------------------
// This file is generated by the SeRoNet Tooling. The SeRoNet Tooling is 
// developed by the SeRoNet Project consortium: 
// http://www.seronet-projekt.de
//
// The ROS Mixed-Port Component is developed by:
// Service Robotics Research Center of Ulm University of Applied Sciences
// Fraunhofer Institute for Manufacturing Engineering and Automation IPA
//
// This code-generator uses infrastructure of the SmartMDSD Toolchain on
// which the SeRoNet Tooling is based on.
//
// CAUTION: 
// This software is a preview for the purpose of technology demonstration. 
// It is experimental and comes with no support. Use at your own risk.
// Please do not modify this file. It will be re-generated
// running the code generator.
//--------------------------------------------------------------------------

#include "ComponentRosDockRosPortExtension.hh"

// statically create a global PlainOpcUaComponentRosDockExtension instance
static ComponentRosDockRosPortExtension ros_port_extension;

ComponentRosDockRosPortExtension::ComponentRosDockRosPortExtension()
:	ComponentRosDockExtension("ComponentRosDockRosPortExtension")
{
	nh = 0;
	callbacksPtr = 0;
}

ComponentRosDockRosPortExtension::~ComponentRosDockRosPortExtension()
{  }

void ComponentRosDockRosPortExtension::loadParameters(const SmartACE::SmartIniParameter &parameter)
{  }

void ComponentRosDockRosPortExtension::initialize(ComponentRosDock *component, int argc, char* argv[])
{
	ros::init(argc, argv, "ComponentRosDock", ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle();
	
	callbacksPtr = new ComponentRosDockRosPortCallbacks();
	
	component->rosPorts = this;
	
	charging_pub = nh->advertise<std_msgs::Bool>("/charging_seronet", 10);
	dock_action_goal = nh->advertise<std_msgs::String>("/docker_control/dock_seronet/goal", 10);
	dock_action_result = nh->subscribe("/docker_control/dock_seronet/result", 10, &ComponentRosDockRosPortCallbacks::dock_action_result_cb, callbacksPtr);
	laser_pub = nh->advertise<sensor_msgs::LaserScan>("/scan_unified_seronet", 10);
	twist_sub = nh->subscribe("/base/twist_controller/command", 10, &ComponentRosDockRosPortCallbacks::twist_sub_cb, callbacksPtr);
	undock_action_goal = nh->advertise<std_msgs::String>("/docker_control/undock_seronet/goal", 10);
	undock_action_result = nh->subscribe("/docker_control/undock_seronet/result", 10, &ComponentRosDockRosPortCallbacks::undock_action_result_cb, callbacksPtr);
}

int ComponentRosDockRosPortExtension::onStartup()
{
	return startExtensionThread();
}

int ComponentRosDockRosPortExtension::extensionExecution()
{
	ros::spin();
	return 0;
}

int ComponentRosDockRosPortExtension::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	ros::shutdown();
	return stopExtensionThread(timeoutTime);
}

void ComponentRosDockRosPortExtension::destroy()
{
	delete nh;
	delete callbacksPtr;
}
