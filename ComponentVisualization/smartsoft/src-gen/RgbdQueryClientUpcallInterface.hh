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
#ifndef _RGBDQUERYCLIENT_UPCALL_INTERFACE_HH
#define _RGBDQUERYCLIENT_UPCALL_INTERFACE_HH

#include "DomainVision/CommDepthImage.hh"

class RgbdQueryClientUpcallInterface {
public:
	virtual ~RgbdQueryClientUpcallInterface() {  }

	virtual void on_rgbdQueryClient(const DomainVision::CommDepthImage &input) = 0;
};

#endif