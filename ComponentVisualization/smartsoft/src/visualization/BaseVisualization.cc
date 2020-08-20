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

BaseVisualization::BaseVisualization(CDisplayWindow3D& window3D, const std::string& identifier) :
	AbstractVisualization(window3D, identifier) {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
			opengl::CArrow::Ptr arrow = opengl::CArrow::Create(0, 0, 0, 1.0, 0, 0);
			arrow->setName(identifier + "_robot_orientation");
			arrow->setColor(0, 0, 1);
			ptrScene->insert(arrow);

			opengl::CCylinder::Ptr cylinder = opengl::CCylinder::Create(0.2, 0.2, 0.4, 10);
			cylinder->setName(identifier + "_robot");
			cylinder->setColor(0, 0, 1);
			ptrScene->insert(cylinder);

			opengl::CText::Ptr robotText1 = opengl::CText::Create();
			robotText1->setName(identifier + "_robotLabel1");
			robotText1->setColor(0, 0, 0);
			ptrScene->insert(robotText1);

			opengl::CText::Ptr robotText2 = opengl::CText::Create();
			robotText2->setName(identifier + "_robotLabel2");
			robotText2->setColor(0, 0, 0);
			ptrScene->insert(robotText2);
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
			opengl::CRenderizable::Ptr obj1 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + "_robot"));
			ptrScene->removeObject(obj1);

			opengl::CRenderizable::Ptr obj2 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + "_robot_orientation"));
			ptrScene->removeObject(obj2);

			opengl::CText::Ptr label1 = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + "_robotLabel1"));
			ptrScene->removeObject(label1);

			opengl::CText::Ptr label2 = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + "_robotLabel2"));
			ptrScene->removeObject(label2);

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

	poses::CPose3D pose(p.get_x(1.0), p.get_y(1.0), p.get_z(1.0), p.get_azimuth(), p.get_elevation(), p.get_roll());
	poses::CPose3D poseLabel1(p.get_x(1.0) - 0.3, p.get_y(1.0) + 0.3, p.get_z(1.0), p.get_azimuth(), p.get_elevation(), p.get_roll());
	poses::CPose3D poseLabel2(p.get_x(1.0) - 0.4, p.get_y(1.0) + 0.3, p.get_z(1.0), p.get_azimuth(), p.get_elevation(), p.get_roll());

	std::stringstream labelString;
	labelString << "pose: x=" << p.get_x(1.0) << ", y=" << p.get_y(1.0) << ", a=" << p.get_azimuth();
	std::string sLabel1 = labelString.str();

	labelString.str("");
	labelString << "raw pose: x=" << rp.get_x(1.0) << ", y=" << rp.get_y(1.0) << ", a=" << rp.get_azimuth();
	std::string sLabel2 = labelString.str();
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CRenderizable::Ptr obj1 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + "_robot"));
		obj1->setPose(pose);

		opengl::CRenderizable::Ptr obj2 = std::dynamic_pointer_cast<opengl::CRenderizable>(ptrScene->getByName(identifier + "_robot_orientation"));
		obj2->setPose(pose);

		opengl::CText::Ptr label1 = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + "_robotLabel1"));
		label1->setPose(poseLabel1);
		label1->setString(sLabel1);

		opengl::CText::Ptr label2 = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + "_robotLabel2"));
		label2->setPose(poseLabel2);
		label2->setString(sLabel2);

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
