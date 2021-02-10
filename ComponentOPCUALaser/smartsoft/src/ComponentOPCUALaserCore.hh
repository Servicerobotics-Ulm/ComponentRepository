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
#ifndef _COMPONENTOPCUALASERCORE_HH
#define _COMPONENTOPCUALASERCORE_HH
	
#include "aceSmartSoft.hh"
#include <iostream>
#include "CommBasicObjects/CommMobileLaserScan.hh"

class ComponentOPCUALaserCore
{
private:

public:
	ComponentOPCUALaserCore();
	SmartACE::SmartMutex ScanLock;

	CommBasicObjects::CommMobileLaserScan global_scan;
};
	
#endif