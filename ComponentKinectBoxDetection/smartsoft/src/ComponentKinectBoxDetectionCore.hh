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
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#ifndef _COMPONENTKINECTBOXDETECTIONCORE_HH
#define _COMPONENTKINECTBOXDETECTIONCORE_HH
	
#include "aceSmartSoft.hh"
#include "basic/ConcreteObject.hh"
#include <iostream>

class ComponentKinectBoxDetectionCore
{
private:

public:
	ComponentKinectBoxDetectionCore();
	/**
	 * List of all objects in the current recognition.
	 */
	std::list<ConcreteObject> concreteObjects;
	uint32_t objectIdCounter;
	uint32_t environmentIdCounter;

	pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

	pcl::PointCloud<pcl::PointXYZRGB> model_point_cloud;

	SmartACE::SmartSemaphore start_recognition;

	std::string searched_obj_type;
};
	
#endif
