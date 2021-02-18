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
#ifndef _COLORIMAGEPUSHNEWESTSERVERWRAPPER_HH
#define _COLORIMAGEPUSHNEWESTSERVERWRAPPER_HH

#include <mutex>
#include "aceSmartSoft.hh"

// include communication objects
#include <DomainVision/CommVideoImage.hh>
#include <DomainVision/CommVideoImageACE.hh>


class ColorImagePushNewestServerWrapper
{
private:
	std::mutex update_mutex;
	Smart::StatusCode update_status;
	
	DomainVision::CommVideoImage updateData;
	
	Smart::IPushServerPattern<DomainVision::CommVideoImage> *colorImagePushNewestServer;
	
public:
	ColorImagePushNewestServerWrapper(Smart::IPushServerPattern<DomainVision::CommVideoImage> *colorImagePushNewestServer);
	virtual ~ColorImagePushNewestServerWrapper();
	
	Smart::StatusCode put(DomainVision::CommVideoImage &colorImagePushNewestServerDataObject);
	
	Smart::StatusCode getLatestUpdate(DomainVision::CommVideoImage &colorImagePushNewestServerDataObject);
	
};

#endif // _COLORIMAGEPUSHNEWESTSERVERWRAPPER_HH
