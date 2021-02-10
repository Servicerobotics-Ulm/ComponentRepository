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
#ifndef _LOCALIZATIONUPDATESERVICEOUTWRAPPER_HH
#define _LOCALIZATIONUPDATESERVICEOUTWRAPPER_HH

#include <mutex>
#include "aceSmartSoft.hh"

// include communication objects
#include <CommBasicObjects/CommBasePositionUpdate.hh>
#include <CommBasicObjects/CommBasePositionUpdateACE.hh>


class LocalizationUpdateServiceOutWrapper
{
private:
	std::mutex update_mutex;
	Smart::StatusCode update_status;
	
	CommBasicObjects::CommBasePositionUpdate updateData;
	
	Smart::ISendClientPattern<CommBasicObjects::CommBasePositionUpdate> *localizationUpdateServiceOut;
	
public:
	LocalizationUpdateServiceOutWrapper(Smart::ISendClientPattern<CommBasicObjects::CommBasePositionUpdate> *localizationUpdateServiceOut);
	virtual ~LocalizationUpdateServiceOutWrapper();
	
	Smart::StatusCode send(CommBasicObjects::CommBasePositionUpdate &localizationUpdateServiceOutDataObject);
	
	Smart::StatusCode getLatestUpdate(CommBasicObjects::CommBasePositionUpdate &localizationUpdateServiceOutDataObject);
	
};

#endif // _LOCALIZATIONUPDATESERVICEOUTWRAPPER_HH