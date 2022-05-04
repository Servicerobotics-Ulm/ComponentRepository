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
//--------------------------------------------------------------------------
//
//  Copyright (C)  2018 Matthias Lutz
//
//              lutz@hs-ulm.de
//              schlegel@hs-ulm.de
//
//      ZAFH Servicerobotic Ulm
//      Christian Schlegel
//      University of Applied Sciences
//      Prittwitzstr. 10
//      89075 Ulm
//      Germany
//
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
//-------------------------------------------------------------------------
#include "VisTask.hh"
#include "ComponentRobotToRobotDocking.hh"

#include <iostream>

VisTask::VisTask(SmartACE::SmartComponent *comp) 
:	VisTaskCore(comp)
{
	std::cout << "constructor VisTask\n";
}
VisTask::~VisTask() 
{
	std::cout << "destructor VisTask\n";
}



int VisTask::on_entry()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> nviewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer = nviewer;
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();


	return 0;
}

void VisTask::showPointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud, std::string id, double color_r, double color_g, double color_b){

	if(viewer.get()!=NULL){
		SmartACE::SmartGuard g(lock);
		if(objectInserted[id] == false){
			viewer->addPointCloud(cloud, id);
		} else {
			viewer->updatePointCloud(cloud,id);
		}
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_r, color_g, color_b,id);

		objectInserted[id] = true;
	}
}

void VisTask::showCircle(pcl::ModelCoefficients coefficients, std::string id){

	//	std::cout<<__FUNCTION__<<"Circle coeffs: "<<coefficients<<std::endl;
	if(viewer.get()!=NULL){


		pcl::PointCloud<pcl::PointXYZ>::Ptr centerPoint (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ point(coefficients.values[0],coefficients.values[1],0.0);
		centerPoint->push_back(point);
		std::stringstream ss;
		ss<<id<<"_centerPoint";

		SmartACE::SmartGuard g(lock);
		if(objectInserted[id] == false){
			viewer->addCircle(coefficients, id);
			viewer->addPointCloud(centerPoint,ss.str());
		} else {
			viewer->removeShape(id);
			viewer->updatePointCloud(centerPoint,ss.str());
			viewer->addCircle(coefficients, id);
		}
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2,id);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255,0,0,id);

		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, ss.str());
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,   ss.str());

		objectInserted[id] = true;
	}

}

void VisTask::removeCircle(std::string id){
	if(viewer.get()!=NULL){

		SmartACE::SmartGuard g(lock);
		std::stringstream ss;
		ss<<id<<"_centerPoint";
		viewer->removeShape(id,1);
		viewer->removePointCloud(ss.str(),1);
		objectInserted[id] == false;
	}
}

void VisTask::removeShape(std::string id){
	if(viewer.get()!=NULL){
		SmartACE::SmartGuard g(lock);
		viewer->removeShape(id,1);
		objectInserted[id] == false;
	}
}


void VisTask::removePointCloud(std::string id){
	if(viewer.get()!=NULL){


		SmartACE::SmartGuard g(lock);
		viewer->removePointCloud(id,1);
		objectInserted[id] == false;
	}
}

void VisTask::clear(){
	if(viewer.get()!=NULL){
		SmartACE::SmartGuard g(lock);
		viewer->removeAllShapes(1);
		viewer->removeAllPointClouds(1);
	}
}


int VisTask::on_execute()
{
	// this method is triggered periodically with period time 100 ms
	SmartACE::SmartGuard g(lock);
	viewer->spinOnce (30);

	return 0;
}
int VisTask::on_exit()
{
	SmartACE::SmartGuard g(lock);
	viewer->close();
	return 0;
}
