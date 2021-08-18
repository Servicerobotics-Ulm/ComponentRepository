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

#include "TriggerHandlerCore.hh"


//
// trigger internal handler methods
//

	// handle FOLLOW_RESET
	void TriggerHandlerCore::handleCommTrackingObjects_TrackingParam_FOLLOW_RESETCore(const double &initX, const double &initY)
	{
		this->handleCommTrackingObjects_TrackingParam_FOLLOW_RESET(initX, initY);
	}

	// handle SET_MAX_COV
	void TriggerHandlerCore::handleCommTrackingObjects_TrackingParam_SET_MAX_COVCore(const double &sigma)
	{
		this->handleCommTrackingObjects_TrackingParam_SET_MAX_COV(sigma);
	}

//
// extended trigger internal handler methods
//

		// handle PersonTofollow
		void TriggerHandlerCore::handlePersonTofollowCore(const short &id_to_follow)
		{
			this->handlePersonTofollow(id_to_follow);
		}