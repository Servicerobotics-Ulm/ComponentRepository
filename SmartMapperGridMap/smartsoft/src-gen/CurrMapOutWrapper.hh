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
#ifndef _CURRMAPOUTWRAPPER_HH
#define _CURRMAPOUTWRAPPER_HH

#include <mutex>
#include "aceSmartSoft.hh"

// include communication objects
#include <CommNavigationObjects/CommGridMap.hh>
#include <CommNavigationObjects/CommGridMapACE.hh>


class CurrMapOutWrapper
{
private:
	std::mutex update_mutex;
	Smart::StatusCode update_status;
	
	CommNavigationObjects::CommGridMap updateData;
	
	Smart::IPushServerPattern<CommNavigationObjects::CommGridMap> *currMapOut;
	
public:
	CurrMapOutWrapper(Smart::IPushServerPattern<CommNavigationObjects::CommGridMap> *currMapOut);
	virtual ~CurrMapOutWrapper();
	
	Smart::StatusCode put(CommNavigationObjects::CommGridMap &currMapOutDataObject);
	
	Smart::StatusCode getLatestUpdate(CommNavigationObjects::CommGridMap &currMapOutDataObject);
	
};

#endif // _CURRMAPOUTWRAPPER_HH
