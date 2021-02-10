//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain Version 0.10.3
// The SmartSoft Toolchain has been developed by:
//
// ZAFH Servicerobotic Ulm
// Christian Schlegel (schlegel@hs-ulm.de)
// University of Applied Sciences
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// smart-robotics.sourceforge.net
//
// This file is generated once. Modify this file to your needs.
// If you want the toolchain to re-generate this file, please
// delete it before running the code generator.
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//
//  Copyright (C) 2010 Manuel Wopfner, Matthias Lutz
//
//        schlegel@hs-ulm.de
//        lutz@hs-ulm.de
//
//        ZAFH Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2.1
//  of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this library; if not, write to the Free Software Foundation, Inc.,
//  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
//  This work is based on previous work by the folks from PlayerStage.
//
//--------------------------------------------------------------------------

//----------------------------------------------------------------------------
//
// CREDITS:
//
// The code for the amcl algorithm was taken from the
// Playerstage Project, which is distributed under GPL, and you can find at
// http://playerstage.sourceforge.net/
//
// Player - One Hell of a Robot Server
// Copyright (C) 2000
//    Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
//
//----------------------------------------------------------------------------

#include "AmclTypes.hh"
#include <iostream>
#include <cmath>
#include <yaml.h>
#include <libgen.h> // for dirname
double normalize(double z) {
	return atan2(sin(z), cos(z));
}

double angle_diff(double a, double b) {

	double d1, d2;
	a = normalize(a);
	b = normalize(b);
	d1 = a - b;
	d2 = 2* M_PI - fabs(d1);

	if (d1 > 0) {
		d2 *= -1.0;
	}

	if (fabs(d1) < fabs(d2)) {
		return (d1);
	} else {
		return (d2);
	}
}

double pi_to_pi(double angle) {
	angle += M_PI;
	double ret_angle = fmod(angle, 2* M_PI );

	if (angle < 0)
		ret_angle += 2* M_PI ;

	ret_angle -= M_PI;

	return ret_angle;
}

std::ostream& operator<<(std::ostream& in, const amcl_hyp_t& hyp)
{
	in << "[ " <<hyp.pf_pose_mean.v[0] << ", " << hyp.pf_pose_mean.v[1] << ", "<< hyp.pf_pose_mean.v[2]*180.0f/M_PI << ", weight : "<< hyp.weight<<" ]"<<std::endl;
	return in;
}

std::ostream& operator<<(std::ostream& in, const pf_vector_t& pose)
{
	in << "[ " <<pose.v[0] << ", " << pose.v[1] << ", "<< pose.v[2]*180.0f/M_PI << " ]"<<std::endl;
	return in;
}

pf_vector_t operator-(const pf_vector_t& a, const pf_vector_t& b)
{
	pf_vector_t c;
	c.v[0] = a.v[0] - b.v[0];
	c.v[1] = a.v[1] - b.v[1];
	c.v[2] = angle_diff(a.v[2], b.v[2]);

	return c;
}

amcl_sensor_to_use get_sensors_to_use(int val)
{
	switch(val)
	{
	case 0:
		std::cout << "Only Laser will be used for Localization"<<std::endl;
		return amcl_sensor_to_use::LASER_ONLY;
		break;
	case 1:
		std::cout << "Only VisualTags will be used for Localization"<<std::endl;
		return amcl_sensor_to_use::VISUAL_TAGS_ONLY;
		break;
	case 2:
		std::cout << "Both Laser and VisualTags will be used for Localization"<<std::endl;
		return amcl_sensor_to_use::LASER_PLUS_VISUAL_TAGS;
		break;

	default:
		break;
	}
	std::cout << "Unknown param " << val <<", use only 0,1,2 to choose sensors to be used"<<std::endl;
	return amcl_sensor_to_use::LASER_ONLY;
}
int get_map_info(const std::string yaml_file_name, amcl_map_info_t& map_info)
{


	// load yaml file
	std::ifstream fin(yaml_file_name);
	if (fin.fail()) {
		std::cerr << "[AMCL] Could not open " << yaml_file_name << "\n";
		return -1;
	}


	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	double res;
	int negate;
	double occ_th, free_th;
	std::string mapfname = "";
	double origin[3];

	try {
		doc["resolution"] >> map_info.resolution;
	} catch (YAML::InvalidScalar) {
		std::cerr << "[AMCL] The map does not contain a resolution tag or it is invalid.\n";
		return -1;
	}
	try {
		doc["negate"] >> map_info.negate;
	} catch (YAML::InvalidScalar) {
		std::cerr << "[AMCL] The map does not contain a negate tag or it is invalid.\n";
		return -1;
	}
	try {
		doc["occupied_thresh"] >> map_info.occ_threshold;
	} catch (YAML::InvalidScalar) {
		std::cerr << "[AMCL] The map does not contain an occupied_thresh tag or it is invalid.\n";
		return -1;
	}
	try {
		doc["free_thresh"] >> map_info.free_threshold;
	} catch (YAML::InvalidScalar) {
		std::cerr << "[AMCL] The map does not contain a free_thresh tag or it is invalid.\n";
		return -1;
	}
	try {
		doc["origin"][0] >> map_info.origin.v[0];
		doc["origin"][1] >> map_info.origin.v[1];
		doc["origin"][2] >> map_info.origin.v[2];
	} catch (YAML::InvalidScalar) {
		std::cerr << "[AMCL] The map does not contain an origin tag or it is invalid.\n";
		return -1;
	}
	try {
		doc["image"] >> map_info.mapfname;
		// TODO: make this path-handling more robust
		if (map_info.mapfname.size() == 0) {
			std::cerr << "[AMCL] The image tag cannot be an empty string.\n";
			return -1;
		}
		if (map_info.mapfname[0] != '/') {
			// dirname can modify what you pass it
			char* fname_copy = strdup(yaml_file_name.c_str());
			mapfname = std::string(dirname(fname_copy)) + '/' + map_info.mapfname;
			free(fname_copy);
		}
	} catch (YAML::InvalidScalar) {
		std::cerr << "[AMCL] The map does not contain an image tag or it is invalid.\n";
		return -1;
	}

	return 0;
}

//void rotation_matrix_to_eurle_angles(double rotation_mat[], double& roll, double& elevation, double& azimuth)
//{
//	double r00 = rotation_mat[0];
//	double r01 = rotation_mat[1];
//	double r02 = rotation_mat[2];
//
//	double r10 = rotation_mat[3];
//	double r11 = rotation_mat[4];
//	double r12 = rotation_mat[5];
//
//	double r20 = rotation_mat[6];
//	double r21 = rotation_mat[7];
//	double r22 = rotation_mat[8];
//
////	std::cout << "\nr_xx : [" << r00 << " "<< r01 << " "<< r02 << ";"
////			                 << r10 << " "<< r11 << " "<< r12 <<";"
////			                 << r20 << " "<< r21 << " "<< r22 <<"]"<< std::endl;
//
//	double thetaX, thetaY, thetaZ;
//
//	//roll X
//	//ele  Y
//	//azi  Z
//
//	if(r02 < 1.0)
//	{
//		if(r02 > -1.0)
//		{
//			thetaY=asin(r02);
//			thetaX=atan2(-1.0*r12,r22);
//			thetaZ=atan2(-1.0*r01,r00);
//		}
//		else//r02=−1
//		{
//			//Notauniquesolution:thetaZ−thetaX=atan2(r10,r11)
//			thetaY=-1.0*M_PI_2;
//			thetaX=-1.0*atan2(r10,r11);
//			thetaZ=0;
//		}
//	}
//	else//r20=+1
//	{
//		//Notauniquesolution:thetaZ+thetaX=atan2(r10,r11)
//		thetaY=M_PI_2;
//		thetaX=atan2(r10,r11);
//		thetaZ=0;
//	}
//
//	azimuth = pi_to_pi(thetaZ);
//	elevation = pi_to_pi(thetaY);
//	roll = pi_to_pi(thetaX);
//
//
//	arma::mat res_r(3,3);
//	res_r(0,0) = r00;
//	res_r(0,1) = r01;
//	res_r(0,2) = r02;
//
//	res_r(1,0) = r10;
//	res_r(1,1) = r11;
//	res_r(1,2) = r12;
//
//	res_r(2,0) = r20;
//	res_r(2,1) = r21;
//	res_r(2,2) = r22;
//
//
////	double eyaw, epitch, eroll;
////	EulerTransformationMatrices::zyx_from_matrix(res_r, eyaw, epitch, eroll);
////
////
////
////	std::cout << "diff R :" << std::setw(10)<<(azimuth - pi_to_pi(eyaw))*180/M_PI << ", "
////							<< std::setw(10)<<(elevation - pi_to_pi(epitch))*180/M_PI << ", "
////							<< std::setw(10)<<(roll - pi_to_pi(eroll))*180/M_PI
////							<< std::endl;
//
//}
