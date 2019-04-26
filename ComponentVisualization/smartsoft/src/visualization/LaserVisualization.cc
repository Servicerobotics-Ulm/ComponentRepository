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
#include <mrpt/opengl/include/mrpt/opengl.h>

#include "../src-gen/ComponentVisualization.hh"


LaserVisualization::LaserVisualization(CDisplayWindow3D& window3D, const std::string& identifier) :
	AbstractVisualization(window3D, identifier) {
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		//opengl::CPlanarLaserScanPtr scan = opengl::CPlanarLaserScan::Create();
		//opengl::CPointCloudPtr scan;
		if(COMP->getGlobalState().getServices().getSimple_laser_visualization() == true) {
			opengl::CPointCloudPtr scan = opengl::CPointCloud::Create();
			scan->setName(identifier);
			ptrScene->insert(scan);
		} else {
			opengl::CPlanarLaserScanPtr scan = opengl::CPlanarLaserScan::Create();
			scan->setName(identifier);
			ptrScene->insert(scan);
		}


	}
	window3D.unlockAccess3DScene();

}

LaserVisualization::~LaserVisualization() {
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	if(COMP->getGlobalState().getServices().getSimple_laser_visualization() == true) {
		opengl::CPointCloudPtr sPtr = (opengl::CPointCloudPtr) ptrScene->getByName(identifier);
		ptrScene->removeObject(sPtr);
	}else{
		opengl::CPlanarLaserScanPtr sPtr = (opengl::CPlanarLaserScanPtr) ptrScene->getByName(identifier);
		ptrScene->removeObject(sPtr);;
	}
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
		double endAngle = pi_to_pi(scan.get_scan_angle(numScans - 1));
		int scanSize = fabs(endAngle - startAngle) / resolution;

		if(COMP->getGlobalState().getSettings().getVerbose() == true){
			std::cout<<"numScan: "<<numScans<<" res: in deg "<<resolution*180/3.14<<" startAngle: "<<startAngle*180/3.14<<" endAngle: "<<endAngle*180/3.14<<std::endl;
		}

#ifdef WITH_OLD_MRPT_VERSION
		CObservation2DRangeScan s;
#else
		obs::CObservation2DRangeScan s;
#endif

		s.scan.resize(scanSize);
		s.validRange.resize(scanSize, 0);
		s.aperture = scanSize * resolution;
		s.maxRange = scan.get_max_distance(1.0);
		s.sensorPose = pBase + pSensor;

		for (size_t i = 0; i < numScans; ++i) {
			int index = fabs(pi_to_pi(scan.get_scan_angle(i)) - startAngle) / resolution;

			if (index < scanSize) {
				s.scan[index] = scan.get_scan_distance(i, 1);// visualization needs distance in meters
				s.validRange[index] = 1;
			}
		}

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
					int index = fabs(pi_to_pi(scan.get_scan_angle(i)) - startAngle) / resolution;

					double x, y, z;
					scan.get_scan_cartesian_3dpoint_world(i, x, y, z, 1);
					sPtr->insertPoint(x, -y, 1.5);
				}
			} else {
				opengl::CPlanarLaserScanPtr sPtr = (opengl::CPlanarLaserScanPtr) ptrScene->getByName(identifier);
				sPtr->setScan(s);
			}


		}
		//show laser sensor coordinate frame
		{
			mrpt::opengl::CSetOfObjectsPtr gl_corner = mrpt::opengl::stock_objects::CornerXYZ();
			gl_corner->setScale(0.4);
			ptrScene->insert(gl_corner);

		}
		//show axis
		{
			mrpt::opengl::CGridPlaneXYPtr gl_grid = mrpt::opengl::CGridPlaneXY::Create();
			gl_grid->setColor(0.6,0.6,0.6);
			ptrScene->insert( gl_grid );
		}

		window3D.unlockAccess3DScene();
		window3D.forceRepaint();
	} else {
		clear();
	}
}

void LaserVisualization::clear() {
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPlanarLaserScanPtr sPtr = (opengl::CPlanarLaserScanPtr) ptrScene->getByName(identifier);
		sPtr->clear();
	}
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}
