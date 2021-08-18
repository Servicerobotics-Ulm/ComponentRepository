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
#ifndef _DEVICEPOSESTATESERVERWRAPPER_HH
#define _DEVICEPOSESTATESERVERWRAPPER_HH

#include <mutex>
#include "aceSmartSoft.hh"

// include communication objects
#include <CommBasicObjects/CommDevicePoseState.hh>
#include <CommBasicObjects/CommDevicePoseStateACE.hh>


class DevicePoseStateServerWrapper
{
private:
	std::mutex update_mutex;
	Smart::StatusCode update_status;
	
	CommBasicObjects::CommDevicePoseState updateData;
	
	Smart::IPushServerPattern<CommBasicObjects::CommDevicePoseState> *devicePoseStateServer;
	
public:
	DevicePoseStateServerWrapper(Smart::IPushServerPattern<CommBasicObjects::CommDevicePoseState> *devicePoseStateServer);
	virtual ~DevicePoseStateServerWrapper();
	
	Smart::StatusCode put(CommBasicObjects::CommDevicePoseState &devicePoseStateServerDataObject);
	
	Smart::StatusCode getLatestUpdate(CommBasicObjects::CommDevicePoseState &devicePoseStateServerDataObject);
	
};

#endif // _DEVICEPOSESTATESERVERWRAPPER_HH