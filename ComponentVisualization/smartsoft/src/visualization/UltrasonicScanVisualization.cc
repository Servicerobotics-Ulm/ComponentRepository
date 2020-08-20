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

#include "UltrasonicScanVisualization.hh"

#include <mrpt/opengl/CPointCloudColoured.h>

UltrasonicScanVisualization::UltrasonicScanVisualization(CDisplayWindow3D& window3D, const std::string& identifier) :
	AbstractVisualization(window3D, identifier)
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColoured::Ptr cloud = opengl::CPointCloudColoured::Create();
		cloud->setName(identifier + "_cloud");
		cloud->setPointSize(2.0);
		ptrScene->insert(cloud);
	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColouredPtr cloud = opengl::CPointCloudColoured::Create();
		cloud->setName(identifier + "_cloud");
		cloud->setPointSize(2.0);
		ptrScene->insert(cloud);
	}
#endif
	window3D.unlockAccess3DScene();
}

UltrasonicScanVisualization::~UltrasonicScanVisualization()
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr & ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColoured::Ptr cloud = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "_cloud"));
		ptrScene->removeObject(cloud);
	}
#else
	opengl::COpenGLScenePtr & ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColouredPtr cloud = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "_cloud");
		ptrScene->removeObject(cloud);
	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}

void UltrasonicScanVisualization::displayScan(CommBasicObjects::CommMobileUltrasonicScan& scan)
{
		// calculate points and colors
		arma::vec q(4);
		q(3) = 1;

		double x, y, z;
		float r = 0, g = 0, b = 0;
		std::vector<ColPoint3d> points;

		CommBasicObjects::CommPose3d comm_pose3d;
		std::vector<CommBasicObjects::CommPose3d> usarr_vector = scan.getUsScan().getScanPointsCopy();



		for(unsigned i=0; i<usarr_vector.size(); ++i) {
			comm_pose3d = usarr_vector[i];
			points.push_back(ColPoint3d(comm_pose3d.get_x(1), comm_pose3d.get_y(1), comm_pose3d.get_z(1), 255, 0,0));
			std::cout<<"Point: "<<comm_pose3d.get_x(1)<<" "<< comm_pose3d.get_y(1)<<" "<<comm_pose3d.get_z(1)<<std::endl;
		}
#ifdef WITH_MRPT_2_0_VERSION
		opengl::COpenGLScene::Ptr & ptrScene = window3D.get3DSceneAndLock();
		//////////////////////////////////////////
		// show coordinates
		{
			opengl::CPointCloudColoured::Ptr cloud = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "_cloud"));
			cloud->clear();
			for (uint32_t i = 0; i < points.size(); i++)
			{
				cloud->push_back(points[i].x, points[i].y, points[i].z, points[i].r, points[i].g, points[i].b);
			}

		}
#else
		opengl::COpenGLScenePtr & ptrScene = window3D.get3DSceneAndLock();
		//////////////////////////////////////////
		// show coordinates
		{
			opengl::CPointCloudColouredPtr cloud = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "_cloud");
			cloud->clear();
			for (uint32_t i = 0; i < points.size(); i++)
			{
				cloud->push_back(points[i].x, points[i].y, points[i].z, points[i].r, points[i].g, points[i].b);
			}

		}
#endif
		window3D.unlockAccess3DScene();
		window3D.forceRepaint();

}

void UltrasonicScanVisualization::clear()
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr & ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColoured::Ptr cloud = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "_cloud"));
		cloud->clear();
	}
#else
	opengl::COpenGLScenePtr & ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColouredPtr cloud = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "_cloud");
		cloud->clear();
	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}
