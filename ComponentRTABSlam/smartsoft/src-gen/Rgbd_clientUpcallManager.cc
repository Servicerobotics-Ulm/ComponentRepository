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
#include "Rgbd_clientUpcallManager.hh"

Rgbd_clientUpcallManager::Rgbd_clientUpcallManager(
	Smart::InputSubject<DomainVision::CommRGBDImage> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<DomainVision::CommRGBDImage>(subject, prescaleFactor)
{  }
Rgbd_clientUpcallManager::~Rgbd_clientUpcallManager()
{  }

void Rgbd_clientUpcallManager::notify_upcalls(const DomainVision::CommRGBDImage &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_rgbd_client(input);
	}
}

void Rgbd_clientUpcallManager::attach(Rgbd_clientUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void Rgbd_clientUpcallManager::detach(Rgbd_clientUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}