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
//  This file is part of the "SmartSoft Communication Classes".
//  It provides basic standardized data types for communication between
//  different components in the mobile robotics context. These classes
//  are designed to be used in conjunction with the SmartSoft Communication
//  Library.
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

#include "LaserVisualization.hh"

#include <mrpt/opengl/CPlanarLaserScan.h>

#include "../src-gen/ComponentVisualization.hh"


LaserVisualization::LaserVisualization(CDisplayWindow3D& window3D, const std::string& identifier) :
	AbstractVisualization(window3D, identifier) {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		if(COMP->getGlobalState().getServices().getSimple_laser_visualization() == true) {
			opengl::CPointCloud::Ptr scan = opengl::CPointCloud::Create();
			scan->setName(identifier);
			ptrScene->insert(scan);
		} else {
			opengl::CPlanarLaserScan::Ptr scan2 = opengl::CPlanarLaserScan::Create();
			scan2->setName(identifier);
			ptrScene->insert(scan2);
		}
		//show laser sensor coordinate frame
		mrpt::opengl::CSetOfObjects::Ptr laser_frame_obj = mrpt::opengl::stock_objects::CornerXYZ(0.25);
		laser_frame_obj->setName(identifier + "_laser_frame_");
		ptrScene->insert(laser_frame_obj);


	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		if(COMP->getGlobalState().getServices().getSimple_laser_visualization() == true) {
			opengl::CPointCloudPtr scan = opengl::CPointCloud::Create();
			scan->setName(identifier);
			ptrScene->insert(scan);
		} else {
			opengl::CPlanarLaserScanPtr scan2 = opengl::CPlanarLaserScan::Create();
			scan2->setName(identifier);
			ptrScene->insert(scan2);
		}

		//show laser sensor coordinate frame
		mrpt::opengl::CSetOfObjectsPtr laser_frame_obj = mrpt::opengl::stock_objects::CornerXYZ(0.25);
		laser_frame_obj->setName(identifier + "_laser_frame_");
		ptrScene->insert(laser_frame_obj);
	}
#endif
	window3D.unlockAccess3DScene();

}

LaserVisualization::~LaserVisualization() {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	if(COMP->getGlobalState().getServices().getSimple_laser_visualization() == true) {
		opengl::CPointCloud::Ptr sPtr = std::dynamic_pointer_cast<opengl::CPointCloud>(ptrScene->getByName(identifier));
		ptrScene->removeObject(sPtr);
	}else{
		opengl::CPlanarLaserScan::Ptr sPtr = std::dynamic_pointer_cast<opengl::CPlanarLaserScan>(ptrScene->getByName(identifier));
		ptrScene->removeObject(sPtr);;
	}

	opengl::CSetOfObjects::Ptr laser_frame_obj = std::dynamic_pointer_cast<opengl::CSetOfObjects>(ptrScene->getByName(identifier + "_laser_frame_"));
	ptrScene->removeObject(laser_frame_obj);
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	if(COMP->getGlobalState().getServices().getSimple_laser_visualization() == true) {
		opengl::CPointCloudPtr sPtr = (opengl::CPointCloudPtr) ptrScene->getByName(identifier);
		ptrScene->removeObject(sPtr);
	}else{
		opengl::CPlanarLaserScanPtr sPtr = (opengl::CPlanarLaserScanPtr) ptrScene->getByName(identifier);
		ptrScene->removeObject(sPtr);;
	}
	opengl::CSetOfObjectsPtr laser_frame_obj = (opengl::CSetOfObjectsPtr) ptrScene->getByName(identifier + "_laser_frame_");
	ptrScene->removeObject(laser_frame_obj);
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();

}

void LaserVisualization::displayLaserScan(const CommBasicObjects::CommMobileLaserScan& scan) {

	// display valid laser scan
	if (scan.is_scan_valid()) {

		CommBasicObjects::CommPose3d bp = scan.get_base_state().get_base_position().get_base_pose3d();
		CommBasicObjects::CommPose3d sp = scan.get_sensor_pose();

		poses::CPose3D pBase(bp.get_x(1), bp.get_y(1), bp.get_z(1), bp.get_azimuth(), bp.get_elevation(), bp.get_roll());
		poses::CPose3D pSensor(sp.get_x(1), sp.get_y(1), sp.get_z(1), sp.get_azimuth(), sp.get_elevation(), sp.get_roll());

		size_t numScans = scan.get_scan_size();
		double resolution = scan.get_scan_resolution();
		double startAngle = pi_to_pi(scan.get_scan_start_angle());
		int max_scan_size = scan.get_max_scan_size();
		double endAngle = pi_to_pi(startAngle+(max_scan_size*resolution));

		if(COMP->getGlobalState().getSettings().getVerbose() == true){
			std::cout<<"numScan: "<<numScans << " maxScan: "<<max_scan_size<<" res: in deg "<<resolution*180/M_PI<<" startAngle: "<<startAngle*180/M_PI<<" endAngle: "<<endAngle*180/M_PI<<std::endl;
		}

#ifdef WITH_OLD_MRPT_VERSION
		CObservation2DRangeScan s;
#else
		obs::CObservation2DRangeScan s;
#endif


#ifdef WITH_MRPT_2_0_VERSION
		opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
		{
			if(COMP->getGlobalState().getServices().getSimple_laser_visualization() == true) {
				opengl::CPointCloud::Ptr sPtr = std::dynamic_pointer_cast<opengl::CPointCloud>(ptrScene->getByName(identifier));
				sPtr->clear();
				sPtr->setPointSize(3.0);
				sPtr->setColorR(0);
				sPtr->setColorG(1);
				sPtr->setColorB(0);
				for (size_t i = 0; i < numScans; ++i) {
					double x, y, z;
					scan.get_scan_cartesian_3dpoint_world(i, x, y, z, 1);
					sPtr->insertPoint(x, y, z);
				}
			} else {

				s.resizeScan(max_scan_size);
				//s.validRange.resize(max_scan_size, 0);
				s.aperture = fabs(endAngle - startAngle);
				s.maxRange = scan.get_max_distance(1.0);
				s.sensorPose = pBase + pSensor;

				for (size_t i = 0; i < numScans; ++i) {
					int index = fabs(pi_to_pi(scan.get_scan_angle(i)) - startAngle) / resolution;

					s.setScanRange(index, scan.get_scan_distance(i, 1));
					s.setScanRangeValidity(index, true);
				}

				opengl::CPlanarLaserScan::Ptr sPtr2 = std::dynamic_pointer_cast<opengl::CPlanarLaserScan>(ptrScene->getByName(identifier));
				sPtr2->setScan(s);
			}


		}
			//show laser sensor coordinate frame
			{
				opengl::CSetOfObjects::Ptr laser_frame_obj = std::dynamic_pointer_cast<opengl::CSetOfObjects>(ptrScene->getByName(identifier + "_laser_frame_"));
				laser_frame_obj->setPose(pBase + pSensor);
			}
#else
		opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
		{
			if(COMP->getGlobalState().getServices().getSimple_laser_visualization() == true) {
				opengl::CPointCloudPtr sPtr = (opengl::CPointCloudPtr) ptrScene->getByName(identifier);
				sPtr->clear();
				sPtr->setPointSize(3.0);
				sPtr->setColorR(0);
				sPtr->setColorG(1);
				sPtr->setColorB(0);
				for (size_t i = 0; i < numScans; ++i) {
					double x, y, z;
					scan.get_scan_cartesian_3dpoint_world(i, x, y, z, 1);
					sPtr->insertPoint(x, y, z);
				}
			} else {

#if defined (WITH_MRPT_1_5_VERSION) || (WITH_MRPT_2_0_VERSION)
				s.resizeScan(max_scan_size);
#else
				s.scan.resize(max_scan_size);
				s.validRange.resize(max_scan_size, 0);
#endif

				s.aperture = fabs(endAngle - startAngle);
				s.maxRange = scan.get_max_distance(1.0);
				s.sensorPose = pBase + pSensor;

				for (size_t i = 0; i < numScans; ++i) {
					int index = fabs(pi_to_pi(scan.get_scan_angle(i)) - startAngle) / resolution;

#if defined (WITH_MRPT_1_5_VERSION) || (WITH_MRPT_2_0_VERSION)
                        s.setScanRange(index, scan.get_scan_distance(i, 1));
                        s.setScanRangeValidity(index, true);
#else
                        s.scan[index] = scan.get_scan_distance(i, 1.0);
                        s.validRange[index] = 1;
#endif


				}

				opengl::CPlanarLaserScanPtr sPtr2 = (opengl::CPlanarLaserScanPtr) ptrScene->getByName(identifier);
				sPtr2->setScan(s);
			}
			//show laser sensor coordinate frame
			{
				opengl::CSetOfObjectsPtr laser_frame_obj = (opengl::CSetOfObjectsPtr) ptrScene->getByName(identifier + "_laser_frame_");
				laser_frame_obj->setPose(pBase + pSensor);
			}
		}
#endif
		window3D.unlockAccess3DScene();
		window3D.forceRepaint();
	} else {
		clear();
	}
}

void LaserVisualization::clear() {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPlanarLaserScan::Ptr sPtr = std::dynamic_pointer_cast<opengl::CPlanarLaserScan>(ptrScene->getByName(identifier));
		sPtr->clear();
	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPlanarLaserScanPtr sPtr = (opengl::CPlanarLaserScanPtr) ptrScene->getByName(identifier);
		sPtr->clear();
	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}
