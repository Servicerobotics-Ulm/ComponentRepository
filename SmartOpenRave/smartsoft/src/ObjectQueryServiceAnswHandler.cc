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
#include "ObjectQueryServiceAnswHandler.hh"
#include "SmartOpenRave.hh"

ObjectQueryServiceAnswHandler::ObjectQueryServiceAnswHandler(Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties, SmartACE::QueryId>* server)
:	ObjectQueryServiceAnswHandlerCore(server)
{
	
}

ObjectQueryServiceAnswHandler::~ObjectQueryServiceAnswHandler()
{
	
}


void ObjectQueryServiceAnswHandler::handleQuery(const SmartACE::QueryId &id, const CommObjectRecognitionObjects::CommObjectRecognitionId& request) 
{
	CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties answer;
	
	// implement your query handling logic here and fill in the answer object
	
   	double x, y, z;
	CommBasicObjects::CommPose3d pose;

	// TODO:
	//COMP->parameterTask.waitQueueEmpty();

	if (OPENRAVE->getPositonOfKinbody(request.get_id(), x, y, z))
	{
		answer.set_id(request.get_id());
		answer.set_type("UNKOWN");

		pose.set_x(x, 1);
		pose.set_y(y, 1);
		pose.set_z(z, 1);
		pose.set_azimuth(0); //TODO
		pose.set_elevation(0);
		pose.set_roll(0);

		answer.set_pose(pose);
		answer.set_dimension(0, 0, 1);
		answer.set_valid(true);
	} else
	{
		answer.set_valid(false);
	}

	this->server->answer(id, answer);
}