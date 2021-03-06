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
#include "RGBImagePushServiceInUpcallManager.hh"

RGBImagePushServiceInUpcallManager::RGBImagePushServiceInUpcallManager(
	Smart::InputSubject<DomainVision::CommVideoImage> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<DomainVision::CommVideoImage>(subject, prescaleFactor)
{  }
RGBImagePushServiceInUpcallManager::~RGBImagePushServiceInUpcallManager()
{  }

void RGBImagePushServiceInUpcallManager::notify_upcalls(const DomainVision::CommVideoImage &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_RGBImagePushServiceIn(input);
	}
}

void RGBImagePushServiceInUpcallManager::attach(RGBImagePushServiceInUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void RGBImagePushServiceInUpcallManager::detach(RGBImagePushServiceInUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}
