// --------------------------------------------------------------------------
//
//  Copyright (C) 2011 Manuel Wopfner
//
//        schlegel@hs-ulm.de
//
//        ZAFH Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// --------------------------------------------------------------------------

#include "BaseVisualization.hh"
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CArrow.h>

#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::opengl;
const std::string BaseVisualization::id_robot_pose_obj = "_robot";
const std::string BaseVisualization::id_robot_orientation_obj = "_robot_orientation";
const std::string BaseVisualization::id_robot_odom_pose_obj = "_robot_odom_pose";
const std::string BaseVisualization::id_robot_odom_orientation_obj = "_robot_odom_orientation";
const std::string BaseVisualization::id_robot_label1_obj = "_robotLabel1";
const std::string BaseVisualization::id_robot_label2_obj = "_robotLabel2";
const std::string BaseVisualization::id_robot_trajectory_pose = "_traj_pose";
const std::string BaseVisualization::id_robot_trajectory_orient = "_traj_orient";
const std::string BaseVisualization::id_robot_trajectory_odom_pose = "_traj_odom_pose";
const std::string BaseVisualization::id_robot_trajectory_odom_orient = "_traj_odom_orient";
const std::string BaseVisualization::id_robot_future_trajectory_pose = "_future_traj_position";
const std::string BaseVisualization::id_robot_future_trajectory_orient = "_future_traj_orient";


BaseVisualization::BaseVisualization(CDisplayWindow3D& window3D, const std::string& identifier, bool show_traj) :
	AbstractVisualization(window3D, identifier) {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CArrow::Ptr arrow = opengl::CArrow::Create(0, 0, 0, 0.5, 0, 0);
		arrow->setName(identifier + id_robot_orientation_obj);
		arrow->setColor(0, 0, 1);
		ptrScene->insert(arrow);

		opengl::CCylinder::Ptr cylinder = opengl::CCylinder::Create(0.2, 0.2, 0.4, 10);
		cylinder->setName(identifier + id_robot_pose_obj);
		cylinder->setColor(0, 0, 1);
		ptrScene->insert(cylinder);


		opengl::CText::Ptr robotText1 = opengl::CText::Create();
		robotText1->setName(identifier + id_robot_label1_obj);
		robotText1->setColor(0, 0, 0);
		ptrScene->insert(robotText1);

		opengl::CText::Ptr robotText2 = opengl::CText::Create();
		robotText2->setName(identifier + id_robot_label2_obj);
		robotText2->setColor(0, 0, 0);
		ptrScene->insert(robotText2);

		show_trajectory = show_traj;

		//plot trajectory of robot pose and robot odometry pose
		if(show_trajectory){//create trajectory objects

			{// create objects for showing base pose and orientation
				opengl::CPointCloudColoured::Ptr estimated_positions = opengl::CPointCloudColoured::Create();
				estimated_positions->setName(identifier+id_robot_trajectory_pose);
				estimated_positions->setPointSize(3);
				ptrScene->insert(estimated_positions);

				opengl::CSetOfLines::Ptr estimated_orientations = opengl::CSetOfLines::Create();
				estimated_orientations->setName(identifier+id_robot_trajectory_orient);
				estimated_orientations->setColor(0, 1, 0, 0.4);
				ptrScene->insert(estimated_orientations);
			}
			{// create objects for showing Odometry pose and orientation
				opengl::CPointCloudColoured::Ptr odometry_positions = opengl::CPointCloudColoured::Create();
				odometry_positions->setName(identifier+id_robot_trajectory_odom_pose);
				odometry_positions->setPointSize(3);
				ptrScene->insert(odometry_positions);

				opengl::CSetOfLines::Ptr odometry_orientations = opengl::CSetOfLines::Create();
				odometry_orientations->setName(identifier+id_robot_trajectory_odom_orient);
				odometry_orientations->setColor(1, 0, 0, 0.4);
				ptrScene->insert(odometry_orientations);
			}
			{// showing the robot future path based on current pose and current velocities
				opengl::CPointCloudColoured::Ptr future_positions = opengl::CPointCloudColoured::Create();
				future_positions->setName(identifier+id_robot_future_trajectory_pose);
				future_positions->setPointSize(5);
				future_positions->setColor(1, 1, 0, 0.4);
				ptrScene->insert(future_positions);
			}
		}
	}
		window3D.unlockAccess3DScene();
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CArrowPtr arrow = opengl::CArrow::Create(0, 0, 0, 1.0, 0, 0);
		arrow->setName(identifier + "_robot_orientation");
		arrow->setColor(0, 0, 1);
		ptrScene->insert(arrow);

		opengl::CCylinderPtr cylinder = opengl::CCylinder::Create(0.2, 0.2, 0.4, 10, 10);
		cylinder->setName(identifier + "_robot");
		cylinder->setColor(0, 0, 1);
		ptrScene->insert(cylinder);

		opengl::CTextPtr robotText1 = opengl::CText::Create();
		robotText1->setName(identifier + "_robotLabel1");
		robotText1->setColor(0, 0, 0);
		ptrScene->insert(robotText1);

		opengl::CTextPtr robotText2 = opengl::CText::Create();
		robotText2->setName(identifier + "_robotLabel2");
		robotText2->setColor(0, 0, 0);
		ptrScene->insert(robotText2);
	}
	window3D.unlockAccess3DScene();
#endif

}

BaseVisualization::~BaseVisualization() {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CRenderizable::Ptr obj1 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + id_robot_pose_obj));
		ptrScene->removeObject(obj1);

		opengl::CRenderizable::Ptr obj2 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + id_robot_orientation_obj));
		ptrScene->removeObject(obj2);

		opengl::CText::Ptr label1 = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + id_robot_label1_obj));
		ptrScene->removeObject(label1);

		opengl::CText::Ptr label2 = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + id_robot_label2_obj));
		ptrScene->removeObject(label2);

		if(show_trajectory){//remove trajectory objects

			{//remove pose objects
				opengl::CPointCloudColoured::Ptr pose_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + id_robot_trajectory_pose));
				ptrScene->removeObject(pose_obj);

				opengl::CSetOfLines::Ptr orient_obj = std::dynamic_pointer_cast<opengl::CSetOfLines>(ptrScene->getByName(identifier + id_robot_trajectory_orient));
				ptrScene->removeObject(orient_obj);
			}

			{//remove odometry objects
				opengl::CPointCloudColoured::Ptr pose_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + id_robot_trajectory_odom_pose));
				ptrScene->removeObject(pose_obj);

				opengl::CSetOfLines::Ptr orient_obj = std::dynamic_pointer_cast<opengl::CSetOfLines>(ptrScene->getByName(identifier + id_robot_trajectory_odom_orient));
				ptrScene->removeObject(orient_obj);
			}

			{//remove future tarjectory objects
				opengl::CPointCloudColoured::Ptr pose_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + id_robot_future_trajectory_pose));
				ptrScene->removeObject(pose_obj);
			}
		}



	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CRenderizablePtr obj1 = ptrScene->getByName(identifier + "_robot");
		ptrScene->removeObject(obj1);
		//obj1->setVisibility(false);


		opengl::CRenderizablePtr obj2 = ptrScene->getByName(identifier + "_robot_orientation");
		ptrScene->removeObject(obj2);
		//obj2->setVisibility(false);

		opengl::CTextPtr label1 = (opengl::CTextPtr) ptrScene->getByName(identifier + "_robotLabel1");
		ptrScene->removeObject(label1);
		//label1->setVisibility(false);

		opengl::CTextPtr label2 = (opengl::CTextPtr) ptrScene->getByName(identifier + "_robotLabel2");
		ptrScene->removeObject(label2);
		//label2->setVisibility(false);

	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}

void BaseVisualization::displayBase(const CommBasicObjects::CommBaseState& pos) {

	CommBasicObjects::CommPose3d p = pos.get_base_position().get_base_pose3d();
	CommBasicObjects::CommPose3d rp = pos.get_base_raw_position().get_base_pose3d();

	poses::CPose3D pose      (p.get_x(1.0),  p.get_y(1.0),  p.get_z(1.0),  p.get_azimuth(),  p.get_elevation(),  p.get_roll());
	poses::CPose3D poseLabel1(p.get_x(1.0) - 0.3, p.get_y(1.0) + 0.3, p.get_z(1.0)+ 1.0, p.get_azimuth(), p.get_elevation(), p.get_roll());

	poses::CPose3D odom_pose(rp.get_x(1.0)+1.0, rp.get_y(1.0)+1.0, rp.get_z(1.0), rp.get_azimuth(), rp.get_elevation(), rp.get_roll());
	poses::CPose3D poseLabel2(rp.get_x(1.0) - 0.3, rp.get_y(1.0) + 0.3, rp.get_z(1.0) + 1.25, rp.get_azimuth(), rp.get_elevation(), rp.get_roll());

	std::stringstream labelString;

	//round off values to 2 decimals for label
	double pose_x = std::ceil(p.get_x(1.0) * 100.0-0.55555) / 100.0;
	double pose_y = std::ceil(p.get_y(1.0) * 100.0-0.55555) / 100.0;
	double pose_azimuth = std::ceil((p.get_azimuth()*180/M_PI) * 100.0-0.55555) / 100.0;

	//round off values to 2 decimals for label
	double odom_x = std::ceil(rp.get_x(1.0) * 100.0-0.55555) / 100.0;
	double odom_y = std::ceil(rp.get_y(1.0) * 100.0-0.55555) / 100.0;
	double odom_azimuth = std::ceil((rp.get_azimuth()*180/M_PI) * 100.0-0.55555) / 100.0;


	labelString << "pose: x=" << pose_x << ", y=" << pose_y << ", a=" << pose_azimuth;
	std::string esitmated_pose_label = labelString.str();

	labelString.str("");
	labelString << "Odom pose: x=" << odom_x << ", y=" << odom_y << ", a=" << odom_azimuth;
	std::string odometry_pose_label = labelString.str();

#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{

		opengl::CRenderizable::Ptr obj1 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + id_robot_pose_obj));
		obj1->setPose(pose);

		opengl::CRenderizable::Ptr obj2 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + id_robot_orientation_obj));
		obj2->setPose(pose);

		opengl::CText::Ptr label1 = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + id_robot_label1_obj));
		label1->setPose(poseLabel1);
		label1->setString(esitmated_pose_label);

		opengl::CText::Ptr label2 = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + id_robot_label2_obj));
		label2->setPose(poseLabel2);
		label2->setString(odometry_pose_label);

		if(show_trajectory){
			double r = 0.1;

			{//add current robot position
			opengl::CPointCloudColoured::Ptr pose_traj_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + id_robot_trajectory_pose));
			if(pose_traj_obj)
			pose_traj_obj->push_back(p.get_x(1.0), p.get_y(1.0), p.get_z(1.0), 0, 1, 0);
			else
							std::cout << "robot pose obj not available" <<std::endl;
			}

			{//add current robot orientation
			opengl::CSetOfLines::Ptr orient_traj_obj = std::dynamic_pointer_cast<opengl::CSetOfLines>(ptrScene->getByName(identifier + id_robot_trajectory_orient));
			if(orient_traj_obj)
			orient_traj_obj->appendLine(p.get_x(1.0) ,p.get_y(1.0), p.get_z(1.0),
									p.get_x(1.0) + r * cos(p.get_azimuth()),
									p.get_y(1.0) + r * sin(p.get_azimuth()), p.get_z(1.0));
			else
							std::cout << "robot orient obj not available" <<std::endl;
			}

			{//add current odometry position
			opengl::CPointCloudColoured::Ptr odom_pose_traj_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + id_robot_trajectory_odom_pose));
			if(odom_pose_traj_obj)
			odom_pose_traj_obj->push_back(rp.get_x(1.0), rp.get_y(1.0), rp.get_z(1.0), 1, 0, 0);
			else
							std::cout << "odom pose obj not available" <<std::endl;
			}

			{//add current odometry orientation
			opengl::CSetOfLines::Ptr odom_orient_traj_obj = std::dynamic_pointer_cast<opengl::CSetOfLines>(ptrScene->getByName(identifier + id_robot_trajectory_odom_orient));
			if(odom_orient_traj_obj)
			odom_orient_traj_obj->appendLine(rp.get_x(1.0) ,rp.get_y(1.0), rp.get_z(1.0),
									rp.get_x(1.0) + r * cos(rp.get_azimuth()),
									rp.get_y(1.0) + r * sin(rp.get_azimuth()), rp.get_z(1.0));
			else
							std::cout << "odom orient obj not available" <<std::endl;
			}


		//show predicted path based on current pose and current velocity
			{
				//std::cout << "vel: " << pos.get_base_velocity().getVX() << ", " <<pos.get_base_velocity().getVY() << "," << pos.get_base_velocity().getVZ()<< "  :::   "
				//		                 << pos.get_base_velocity().getWX() << ", " <<pos.get_base_velocity().getWY() << "," << pos.get_base_velocity().getWZ() <<std::endl;
				double vx = pos.getBaseVelocity().get_vX(1.0);
				double vy = pos.getBaseVelocity().get_vY(1.0);
				double omega = pos.getBaseVelocity().getWZ();

				double v = sqrt(vx*vx + vy*vy);

				double v_by_omega = v/omega;


				double current_pose_x = p.get_x(1.0);
				double current_pose_y = p.get_y(1.0);
				double current_pose_azimuth = p.get_azimuth();



				opengl::CPointCloudColoured::Ptr vel_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + id_robot_future_trajectory_pose));
				if(vel_obj)
				vel_obj->clear();

				opengl::CSetOfLines::Ptr orient_obj = std::dynamic_pointer_cast<opengl::CSetOfLines>(ptrScene->getByName(identifier + id_robot_future_trajectory_orient));
				if(orient_obj)
				orient_obj->clear();


				double delta_t = 5/10.0;
				double prev_pose_x, prev_pose_y, prev_pose_az;

				prev_pose_x = current_pose_x;
				prev_pose_y = current_pose_y;
				prev_pose_az = current_pose_azimuth;

                for(unsigned int i =1; i<=10; ++i)
                {
                	double next_pose_x = prev_pose_x - (v_by_omega)*sin(prev_pose_az) + (v_by_omega)*sin(prev_pose_az+ omega*delta_t);
                	double next_pose_y = prev_pose_y + (v_by_omega)*cos(prev_pose_az) - (v_by_omega)*cos(prev_pose_az+ omega*delta_t);
                	double next_pose_azimuth = prev_pose_az + omega*delta_t;

                	prev_pose_x = next_pose_x;
                	prev_pose_y = next_pose_y;
                	prev_pose_az = next_pose_azimuth;

                	if(vel_obj)
                	vel_obj->push_back(next_pose_x, next_pose_y, 0, 1, 1, 0);
		}
			}
		}

	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CRenderizablePtr obj1 = ptrScene->getByName(identifier + "_robot");
		obj1->setPose(pose);

		opengl::CRenderizablePtr obj2 = ptrScene->getByName(identifier + "_robot_orientation");
		obj2->setPose(pose);

		opengl::CTextPtr label1 = (opengl::CTextPtr) ptrScene->getByName(identifier + "_robotLabel1");
		label1->setPose(poseLabel1);
		label1->setString(sLabel1);

		opengl::CTextPtr label2 = (opengl::CTextPtr) ptrScene->getByName(identifier + "_robotLabel2");
		label2->setPose(poseLabel2);
		label2->setString(sLabel2);

	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}

void BaseVisualization::clear() {

}

#ifdef WITH_MRPT_2_0_VERSION
void BaseVisualization::set_show_trajectory(bool in_flag)
{
	show_trajectory = in_flag;
}
#endif
