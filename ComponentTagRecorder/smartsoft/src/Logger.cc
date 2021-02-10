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
//  Copyright (C) 2020 Nayabrasul Shaik
//
//        nayabrasul.shaik@thu.de
//
//        Christian Schlegel (christian.schlegel@thu.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
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
//-------------------------------------------------------------------------

#include <Logger.hh>
#include <cmath>
#ifdef WITH_MRPT_2_0_VERSION
#else
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math.h>
#endif
Logger::Logger() {

	//create filename from current date and time
	std::ostringstream oss;
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	oss<<"markers_" <<std::put_time(std::localtime(&now_c), "%d_%m_%Y_%H_%M_%S")<<".txt";
	filename = oss.str();

	//open log file
	log_file.open(filename.c_str());
	std::cout << "logfile is opened : " << filename << std::endl;

	size_t formatting_width = 30;
	//configure stream format and write Header
	log_file << std::setprecision(6);
	log_file << std::fixed;
	log_file <<std::setw(formatting_width) <<"Tag Id"
			 <<std::setw(formatting_width) <<"X"
			 <<std::setw(formatting_width) <<"Y"
			 <<std::setw(formatting_width) <<"Z"
			 <<std::setw(formatting_width) <<"Azimuth"
			 <<std::setw(formatting_width) <<"Elevation"
			 <<std::setw(formatting_width) <<"Roll"
			 <<std::setw(formatting_width) <<"Robot_X"
			 <<std::setw(formatting_width) <<"Robot_Y"
			 <<std::setw(formatting_width) <<"Robot_Z"
			 <<std::setw(formatting_width) <<"Robot_Azimuth"
			 <<std::setw(formatting_width) <<"Robot_Elevation"
			 <<std::setw(formatting_width) <<"Robot_Roll"
			 <<std::setw(formatting_width) <<"Tag_in_camera_X"
			 <<std::setw(formatting_width) <<"Tag_in_camera_Y"
			 <<std::setw(formatting_width) <<"Tag_in_camera_Z"
			 <<std::setw(formatting_width) <<"Tag_in_camera_Azimuth"
			 <<std::setw(formatting_width) <<"Tag_in_camera_Elevation"
			 <<std::setw(formatting_width) <<"Tag_in_camera_Roll"<<"\n";
}

Logger::~Logger() {
	log_file << std::endl;
	log_file.close();
	std::cout << "logfile is closed : " << filename << std::endl;
}

void Logger::write_markerlist(const CommTrackingObjects::CommDetectedMarkerList& dml)
{
	size_t num_markers = dml.getMarkersSize();
	if(num_markers == 0)
		return;

	CommBasicObjects::CommPose3d robot_pose = dml.getBase_state().getBasePose().getPose3D();
//	CommBasicObjects::CommPose3d sensor_pose = dml.getSensor_pose();
//
//
//	mrpt::poses::CPose3D mrpt_sensor_pose(sensor_pose.get_x(1.0),
//										  sensor_pose.get_y(1.0),
//										  sensor_pose.get_z(1.0),
//										  sensor_pose.get_azimuth(),
//										  sensor_pose.get_elevation(),
//										  sensor_pose.get_roll());
//
//	mrpt::poses::CPose3D mrpt_robot_pose(robot_pose.get_x(1.0),
//										 robot_pose.get_y(1.0),
//										 robot_pose.get_z(1.0),
//										 robot_pose.get_azimuth(),
//										 robot_pose.get_elevation(),
//										 robot_pose.get_roll());

	size_t formatting_width = 30;

	for(size_t i =0; i< num_markers; ++i)
	{
		CommTrackingObjects::CommDetectedMarker current_marker = dml.getMarkersElemAtPos(i);

		CommBasicObjects::CommPose3d marker_pose_in_world = dml.get_tag_pose_in_world_frame_by_index(i);
		CommBasicObjects::CommPose3d marker_pose_in_camera = dml.get_tag_pose_in_sensor_frame_by_index(i);
//		CommBasicObjects::CommPose3d marker_pose_in_sensor_frame = dml.get_tag_pose_in_sensor_frame_by_index(i);

//		mrpt::poses::CPose3D mrpt_marker_pose_world(marker_pose_in_world.get_x(1.0),
//				marker_pose_in_world.get_y(1.0),
//				marker_pose_in_world.get_z(1.0),
//				marker_pose_in_world.get_azimuth(),
//				marker_pose_in_world.get_elevation(),
//				marker_pose_in_world.get_roll());
//
//		mrpt::poses::CPose3D mrpt_marker_pose_world = mrpt_robot_pose + mrpt_sensor_pose + mrpt_marker_pose_in_sensor_frame;


		log_file <<std::setw(10) << current_marker.getId() //1
						 <<std::setw(formatting_width) <<marker_pose_in_world.get_x(1.0)//2
						 <<std::setw(formatting_width) <<marker_pose_in_world.get_y(1.0)//3
						 <<std::setw(formatting_width) <<marker_pose_in_world.get_z(1.0)//4
						 <<std::setw(formatting_width) <<marker_pose_in_world.get_azimuth()//5
						 <<std::setw(formatting_width) <<marker_pose_in_world.get_elevation()//6
						 <<std::setw(formatting_width) <<marker_pose_in_world.get_roll()//7

						 <<std::setw(formatting_width) <<robot_pose.get_x(1.0)//8
						 <<std::setw(formatting_width) <<robot_pose.get_y(1.0)//9
						 <<std::setw(formatting_width) <<robot_pose.get_z(1.0)//10
						 <<std::setw(formatting_width) <<robot_pose.get_azimuth()//11
						 <<std::setw(formatting_width) <<robot_pose.get_elevation()//12
						 <<std::setw(formatting_width) <<robot_pose.get_roll()//13

						 <<std::setw(formatting_width) <<marker_pose_in_camera.get_x(1.0)//14
						 <<std::setw(formatting_width) <<marker_pose_in_camera.get_y(1.0)//15
						 <<std::setw(formatting_width) <<marker_pose_in_camera.get_z(1.0)//16
						 <<std::setw(formatting_width) <<marker_pose_in_camera.get_azimuth()//17
						 <<std::setw(formatting_width) <<marker_pose_in_camera.get_elevation()//18
						 <<std::setw(formatting_width) <<marker_pose_in_camera.get_roll()//19

						 <<std::endl;

		// add marker to data to calculate average pose of id
//		CommBasicObjects::CommPose3d p(mrpt_marker_pose_world.x(),
//				mrpt_marker_pose_world.y(),
//				mrpt_marker_pose_world.z(),
//				mrpt_marker_pose_world.yaw(),
//				mrpt_marker_pose_world.pitch(),
//				mrpt_marker_pose_world.roll());

		data[current_marker.getId()].push_back(marker_pose_in_world);
	}
}

std::map<unsigned int, CommBasicObjects::CommPose3d>& Logger::calculate_mean_poses()
{
	for( auto it = data.begin(); it != data.end(); ++it )
	{
		unsigned int id = it->first;
		std::cout << "processing poses of tag Id = " << id <<std::endl;
		std::vector<CommBasicObjects::CommPose3d> poses = it->second;
		unsigned int size = poses.size();
		double x =0;
		double y =0;
		double z =0;

		double azimuth =0;
		double elevation =0;
		double roll = 0;

		for(auto current_pose : poses)
		{
			if(is_pose_valid(current_pose))
			{
				x += current_pose.get_x(1);
				y += current_pose.get_y(1);
				z += current_pose.get_z(1);

				azimuth += current_pose.get_azimuth();
				elevation += current_pose.get_elevation();
				roll += current_pose.get_roll();
			}
		}

		CommBasicObjects::CommPose3d current_avg_pose(x/size, y/size, z/size, azimuth/size, elevation/size, roll/size, 1);
		data_average[id] = current_avg_pose;
	}
	return data_average;
}
double Logger::pi_to_pi(double angle)
{

	   angle+=M_PI;
	   double ret_angle = fmod(angle,2*M_PI);

	   if(angle<0)
	     ret_angle+=2*M_PI;

	   ret_angle-=M_PI;

	   return ret_angle;
}
bool Logger::is_pose_valid(const CommBasicObjects::CommPose3d& pose)
{
	return 	!(std::isnan(pose.get_x(1))||
			  std::isnan(pose.get_y(1))||
			  std::isnan(pose.get_z(1))||
			  std::isnan(pose.get_azimuth())||
			  std::isnan(pose.get_elevation())||
			  std::isnan(pose.get_roll()));
}
