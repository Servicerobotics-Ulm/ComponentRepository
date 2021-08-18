/*--------------------------------------------------------------------------

 Copyright (C) 2020

 Created on: Nov 09, 2020
 Author    : Nayabrasul Shaik (nayabrasul.shaik@thu.de)

 ZAFH Servicerobotik Ulm
 University of Applied Sciences
 Prittwitzstr. 10
 D-89075 Ulm
 Germany

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2.1
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this library; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

 --------------------------------------------------------------------------*/

#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CArrow.h>

#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/stock_objects.h>
#include <visualization/PoseVisualization.hh>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::opengl;
const std::string PoseVisualization::id_location_pose_obj = "_pose_position";
const std::string PoseVisualization::id_location_orientation_obj = "_pose_orientation";
const std::string PoseVisualization::id_location_label_obj = "_pose_label";


PoseVisualization::PoseVisualization(CDisplayWindow3D& window3D, const std::string& identifier) :
	AbstractVisualization(window3D, identifier) {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
//		opengl::CArrow::Ptr arrow = opengl::CArrow::Create(0, 0, 0, 0.5, 0, 0);
//		arrow->setName(identifier + id_location_orientation_obj);
//		arrow->setColor(0, 1, 0);
//		ptrScene->insert(arrow);

		opengl::CCylinder::Ptr cylinder = opengl::CCylinder::Create(0.1, 0.1, 0.4, 10);
		cylinder->setName(identifier + id_location_pose_obj);
		cylinder->setColor(0, 1, 0);
		ptrScene->insert(cylinder);

		opengl::CText::Ptr pose_label = opengl::CText::Create();
		pose_label->setName(identifier + id_location_label_obj);
		pose_label->setColor(0, 0, 0);
		ptrScene->insert(pose_label);
	}
		window3D.unlockAccess3DScene();
#else
//	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
//	{
//		opengl::CArrowPtr arrow = opengl::CArrow::Create(0, 0, 0, 1.0, 0, 0);
//		arrow->setName(identifier + "_robot_orientation");
//		arrow->setColor(0, 0, 1);
//		ptrScene->insert(arrow);
//
//		opengl::CCylinderPtr cylinder = opengl::CCylinder::Create(0.2, 0.2, 0.4, 10, 10);
//		cylinder->setName(identifier + "_robot");
//		cylinder->setColor(0, 0, 1);
//		ptrScene->insert(cylinder);
//
//		opengl::CTextPtr robotText1 = opengl::CText::Create();
//		robotText1->setName(identifier + "_robotLabel1");
//		robotText1->setColor(0, 0, 0);
//		ptrScene->insert(robotText1);
//
//		opengl::CTextPtr robotText2 = opengl::CText::Create();
//		robotText2->setName(identifier + "_robotLabel2");
//		robotText2->setColor(0, 0, 0);
//		ptrScene->insert(robotText2);
//	}
//	window3D.unlockAccess3DScene();
#endif

}

PoseVisualization::~PoseVisualization() {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CRenderizable::Ptr obj1 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + id_location_pose_obj));
		ptrScene->removeObject(obj1);

//		opengl::CRenderizable::Ptr obj2 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + id_location_orientation_obj));
//		ptrScene->removeObject(obj2);

		opengl::CText::Ptr label1 = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + id_location_label_obj));
	    ptrScene->removeObject(label1);
	}
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
#else
//	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
//	{
//		opengl::CRenderizablePtr obj1 = ptrScene->getByName(identifier + "_robot");
//		ptrScene->removeObject(obj1);
//		//obj1->setVisibility(false);
//
//
//		opengl::CRenderizablePtr obj2 = ptrScene->getByName(identifier + "_robot_orientation");
//		ptrScene->removeObject(obj2);
//		//obj2->setVisibility(false);
//
//		opengl::CTextPtr label1 = (opengl::CTextPtr) ptrScene->getByName(identifier + "_robotLabel1");
//		ptrScene->removeObject(label1);
//		//label1->setVisibility(false);
//
//		opengl::CTextPtr label2 = (opengl::CTextPtr) ptrScene->getByName(identifier + "_robotLabel2");
//		ptrScene->removeObject(label2);
//		//label2->setVisibility(false);
//
//	}
//	window3D.unlockAccess3DScene();
//	window3D.forceRepaint();
#endif
}

void PoseVisualization::displayPose(const CommBasicObjects::CommPose3d& pose,  const VizConfig& config) {

	poses::CPose3D mrpt_pose(pose.get_x(1.0),  pose.get_y(1.0),  pose.get_z(1.0),  pose.get_azimuth(),  pose.get_elevation(),  pose.get_roll());
	poses::CPose3D label_pose(pose.get_x(1.0) - 0.3, pose.get_y(1.0) + 0.3, pose.get_z(1.0) + 1.0, pose.get_azimuth(), pose.get_elevation(), pose.get_roll());


	//round off values to 2 decimals for label
	double pose_x = std::ceil(pose.get_x(1.0) * 100.0-0.55555) / 100.0;
	double pose_y = std::ceil(pose.get_y(1.0) * 100.0-0.55555) / 100.0;
	double pose_azimuth = std::ceil((pose.get_azimuth()*180/M_PI) * 100.0-0.55555) / 100.0;


	std::stringstream pose_label;
	pose_label << config.label_prefix <<" x =" << pose_x << ", y=" << pose_y << ", a=" << pose_azimuth;
	std::string str_pose_label = pose_label.str();
//
//	labelString.str("");
//	labelString << "raw pose: x=" << rp.get_x(1.0) << ", y=" << rp.get_y(1.0) << ", a=" << rp.get_azimuth();
//	std::string sLabel2 = labelString.str();
//
//	std::cout << "pose: " << p <<std::endl;
//	std::cout << "rawpose: " << rp <<std::endl;
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CRenderizable::Ptr obj1 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + id_location_pose_obj));
		obj1->setPose(mrpt_pose);
		obj1->setColor(config.color[0], config.color[1], config.color[2]);

//		opengl::CRenderizable::Ptr obj2 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + id_location_orientation_obj));
//		obj2->setPose(mrpt_pose);
//		obj2->setColor(config.color[0], config.color[1], config.color[2]);

		opengl::CText::Ptr obj3 = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + id_location_label_obj));
		obj3->setPose(label_pose);
		obj3->setString(str_pose_label);
	}
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
#else
//	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
//	{
//		opengl::CRenderizablePtr obj1 = ptrScene->getByName(identifier + "_robot");
//		obj1->setPose(pose);
//
//		opengl::CRenderizablePtr obj2 = ptrScene->getByName(identifier + "_robot_orientation");
//		obj2->setPose(pose);
//
//		opengl::CTextPtr label1 = (opengl::CTextPtr) ptrScene->getByName(identifier + "_robotLabel1");
//		label1->setPose(poseLabel1);
//		label1->setString(sLabel1);
//
//		opengl::CTextPtr label2 = (opengl::CTextPtr) ptrScene->getByName(identifier + "_robotLabel2");
//		label2->setPose(poseLabel2);
//		label2->setString(sLabel2);
//
//	window3D.unlockAccess3DScene();
//	window3D.forceRepaint();
//	}
#endif
}

void PoseVisualization::clear() {

}
