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
#include "ComponentRackDetection.hh"

ObjectQueryServiceAnswHandler::ObjectQueryServiceAnswHandler(IQueryServer *server)
:	ObjectQueryServiceAnswHandlerCore(server)
{
	
}


void ObjectQueryServiceAnswHandler::handleQuery(const Smart::QueryIdPtr &id, const CommObjectRecognitionObjects::CommObjectRecognitionId& request) 
{
	CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties answer;

	answer.set_valid(false);

	std::cout << "[ObjectPropertyQueryHandler] Object property Query for id: " << request.get_id() << std::endl;

	CommBasicObjects::CommPose3d pose;
	CommBasicObjects::CommPose3d surface_pose;
	for (std::list<ConcreteObject>::const_iterator iter = COMP->concreteObjects.begin(); iter != COMP->concreteObjects.end(); iter++) {
		if (iter->getId() == request.get_id()) {
			answer.set_id(iter->getId());
			answer.set_type(iter->getObjectClass());

			const CPose3D& p = iter->getPose();
			pose.set_x(p.x(), 1);
			pose.set_y(p.y(), 1);
			pose.set_z(p.z(), 1);
			pose.set_azimuth(p.yaw());
			pose.set_elevation(p.pitch());
			pose.set_roll(p.roll());
			answer.set_pose(pose);

			const CPose3D& surface_p = iter->getSurfacePose();
			surface_pose.set_x(surface_p.x(), 1);
			surface_pose.set_y(surface_p.y(), 1);
			surface_pose.set_z(surface_p.z(), 1);
			surface_pose.set_azimuth(surface_p.yaw());
			surface_pose.set_elevation(surface_p.pitch());
			surface_pose.set_roll(surface_p.roll());
			std::vector<CommBasicObjects::CommPose3d> surface_poses;
			surface_poses.push_back(surface_pose);
			answer.setObjectSurfacePoses(surface_poses);

			answer.setDimension(iter->getDimension());

			CommBasicObjects::CommPosition3d dimension;

			if(iter->getObjectClass() == "OBSTACLE-HULL"){
				std::cout<<"[ObjectPropertyQueryHandler] found ObstacleHullPointCloud.size(): "<<iter->getObstacleHullPointCloud().getAsStdVector().size()<<std::endl;
				std::cout<<"[ObjectPropertyQueryHandler] found vertices.size(): "<<iter->getObstacleHullvertices().size()<<std::endl;
				answer.set_triMesh( iter->getObstacleHullvertices(), iter->getObstacleHullPointCloud().getAsStdVector());
			}

			answer.set_valid(true);
			break;
		}
	}

	server->answer(id, answer);

	if(answer.is_valid()){
		std::cout << "[ObjectPropertyQueryHandler] Answer send -- VALID!" << std::endl;
	}
	else{
		std::cout << "[ObjectPropertyQueryHandler] Answer send -- NOT VALID! "<< std::endl;
	}
}