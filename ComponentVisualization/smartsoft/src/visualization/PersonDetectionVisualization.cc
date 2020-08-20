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
#include "PersonDetectionVisualization.hh"
#include <mrpt/opengl/include/mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/include/mrpt/opengl/CText.h>

PersonDetectionVisualization::PersonDetectionVisualization(CDisplayWindow3D& window3D, const std::string& identifier) :
	AbstractVisualization(window3D, identifier)
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloud::Ptr cloud = opengl::CPointCloud::Create();
		cloud->setName(identifier + "_cloud");
		cloud->setColor(mrpt::img::TColorf(0.83, 0.33));
		cloud->setPointSize(5.0);
		ptrScene->insert(cloud);

		opengl::CSetOfObjects::Ptr text = opengl::CSetOfObjects::Create();
		text->setName(identifier + "_text");
		ptrScene->insert(text);
	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudPtr cloud = opengl::CPointCloud::Create();
		cloud->setName(identifier + "_cloud");
		cloud->setColor(mrpt::utils::TColorf(0.83, 0.33));
		cloud->setPointSize(5.0);
		ptrScene->insert(cloud);

		opengl::CSetOfObjectsPtr text = opengl::CSetOfObjects::Create();
		text->setName(identifier + "_text");
		ptrScene->insert(text);
	}
#endif
	window3D.unlockAccess3DScene();
}

PersonDetectionVisualization::~PersonDetectionVisualization()
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloud::Ptr cloud = std::dynamic_pointer_cast<opengl::CPointCloud>(ptrScene->getByName(identifier + "_cloud"));
		ptrScene->removeObject(cloud);
		opengl::CSetOfObjects::Ptr textObj = std::dynamic_pointer_cast<opengl::CSetOfObjects>(ptrScene->getByName(identifier + "_text"));
		ptrScene->removeObject(textObj);
	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudPtr cloud = (opengl::CPointCloudPtr) ptrScene->getByName(identifier + "_cloud");
		ptrScene->removeObject(cloud);
		opengl::CSetOfObjectsPtr textObj = (opengl::CSetOfObjectsPtr) ptrScene->getByName(identifier + "_text");
		ptrScene->removeObject(textObj);
	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}

void PersonDetectionVisualization::displayPersons(std::vector<CommTrackingObjects::CommDetectedPerson>& persons)
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloud::Ptr cloud = std::dynamic_pointer_cast<opengl::CPointCloud>(ptrScene->getByName(identifier + "_cloud"));
		opengl::CSetOfObjects::Ptr textObj = std::dynamic_pointer_cast<opengl::CSetOfObjects>(ptrScene->getByName(identifier + "_text"));

		for (size_t i = 0; i < persons.size(); ++i)
		{
			CommBasicObjects::CommPose3d p = persons[i].getPose();

			// add point
			CommBasicObjects::CommPose3d pose = persons[i].getPose();
			cloud->insertPoint(pose.get_x(1.0), pose.get_y(1.0), pose.get_z(1.0));

			// add label
			poses::CPose3D textPose(pose.get_x(1.0) - 0.1, pose.get_y(1.0), pose.get_z(1.0) - 0.1, 0, 0, 0);
			std::stringstream label;
			label << "id=" << persons[i].getId() << "(" << p.get_x(1) << ", " << p.get_y(1) << ", " << p.get_z(1) << ")";

			opengl::CText::Ptr text = opengl::CText::Create();
			text->setPose(textPose);
			text->setString(label.str());
			textObj->insert(text);
		}
	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudPtr cloud = (opengl::CPointCloudPtr) ptrScene->getByName(identifier + "_cloud");
		opengl::CSetOfObjectsPtr textObj = (opengl::CSetOfObjectsPtr) ptrScene->getByName(identifier + "_text");

		for (size_t i = 0; i < persons.size(); ++i)
		{
			CommBasicObjects::CommPose3d p = persons[i].getPose();

			// add point
			CommBasicObjects::CommPose3d pose = persons[i].getPose();
			cloud->insertPoint(pose.get_x(1.0), pose.get_y(1.0), pose.get_z(1.0));

			// add label
			poses::CPose3D textPose(pose.get_x(1.0) - 0.1, pose.get_y(1.0), pose.get_z(1.0) - 0.1, 0, 0, 0);
			std::stringstream label;
			label << "id=" << persons[i].getId() << "(" << p.get_x(1) << ", " << p.get_y(1) << ", " << p.get_z(1) << ")";

			opengl::CTextPtr text = opengl::CText::Create();
			text->setPose(textPose);
			text->setString(label.str());
			textObj->insert(text);
		}
	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}

void PersonDetectionVisualization::clear()
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr & ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloud::Ptr cloud = std::dynamic_pointer_cast<opengl::CPointCloud>(ptrScene->getByName(identifier + "_cloud"));
		cloud->clear();

		opengl::CSetOfObjects::Ptr text = std::dynamic_pointer_cast<opengl::CSetOfObjects>(ptrScene->getByName(identifier + "_text"));
		text->clear();
	}
#else
	opengl::COpenGLScenePtr & ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudPtr cloud = (opengl::CPointCloudPtr) ptrScene->getByName(identifier + "_cloud");
		cloud->clear();

		opengl::CSetOfObjectsPtr text = (opengl::CSetOfObjectsPtr) ptrScene->getByName(identifier + "_text");
		text->clear();
	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}
