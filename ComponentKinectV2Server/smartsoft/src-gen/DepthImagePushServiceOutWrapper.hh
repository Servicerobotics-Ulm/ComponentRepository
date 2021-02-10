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
#ifndef _DEPTHIMAGEPUSHSERVICEOUTWRAPPER_HH
#define _DEPTHIMAGEPUSHSERVICEOUTWRAPPER_HH

#include <mutex>
#include "aceSmartSoft.hh"

// include communication objects
#include <DomainVision/CommDepthImage.hh>
#include <DomainVision/CommDepthImageACE.hh>


class DepthImagePushServiceOutWrapper
{
private:
	std::mutex update_mutex;
	Smart::StatusCode update_status;
	
	DomainVision::CommDepthImage updateData;
	
	Smart::IPushServerPattern<DomainVision::CommDepthImage> *depthImagePushServiceOut;
	
public:
	DepthImagePushServiceOutWrapper(Smart::IPushServerPattern<DomainVision::CommDepthImage> *depthImagePushServiceOut);
	virtual ~DepthImagePushServiceOutWrapper();
	
	Smart::StatusCode put(DomainVision::CommDepthImage &depthImagePushServiceOutDataObject);
	
	Smart::StatusCode getLatestUpdate(DomainVision::CommDepthImage &depthImagePushServiceOutDataObject);
	
};

#endif // _DEPTHIMAGEPUSHSERVICEOUTWRAPPER_HH