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
//  Copyright (C) 2020 Nayabrasul Shaik
//
//        christian.schlegel@thu.de
//        nayabrasul.shaik@thu.de
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

#include <sensors/amcl_visual_tag.h>
#include <cmath>
#include <iostream>
#include<iomanip>
namespace amcl {

AMCLVisualTag::AMCLVisualTag() {
	// TODO Auto-generated constructor stub

}

AMCLVisualTag::~AMCLVisualTag() {
	// TODO Auto-generated destructor stub
}

bool AMCLVisualTag::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
	AMCLTagData* tag_data = (AMCLTagData*)data;
	pf_update_sensor(pf, (pf_sensor_model_fn_t)TagSensorModel, tag_data);
	return true;
}

double AMCLVisualTag::TagSensorModel(AMCLTagData *data, pf_sample_set_t* set)
{
	double sum_all_particles_weight =0.0; //return the sum of weight of all the particles
	AMCLVisualTag* sensor = (AMCLVisualTag*)data->sensor;

	double robot_x = data->robot_pose_from_tags.get_x(1.0);
	double robot_y = data->robot_pose_from_tags.get_y(1.0);
	double robot_theta = data->robot_pose_from_tags.get_azimuth();


	for (size_t i = 0; i < set->sample_count; i++)
	{
		pf_sample_t *current_particle = set->samples +i;


		double particle_x = current_particle->pose.v[0];
		double particle_y = current_particle->pose.v[1];
		double particle_theta = current_particle->pose.v[2];

		//2D guassian
		//https://en.wikipedia.org/wiki/Gaussian_function#Two-dimensional_Gaussian_function
		// variance_x = variance_y

		double robot_to_particle_distance =  (particle_x - robot_x)*(particle_x - robot_x) + (particle_y - robot_y)*(particle_y - robot_y);
		double p_distance = std::exp(-0.5*(robot_to_particle_distance)/data->distance_variance);

		//1D guassian
		double robot_to_particle_orientation =  angle_diff(robot_theta, particle_theta) * angle_diff(robot_theta, particle_theta);
		double p_orientation = std::exp(-0.5*(robot_to_particle_orientation)/data->orientation_variance);

		//double p = (data->distance_weight * p_distance) + (data->orientation_weight * p_orientation);
		double p = p_distance * p_orientation;

		//double current_particle_weight = 1+ p*p*p;
		double current_particle_weight = p;


        current_particle->weight = current_particle_weight;

        sum_all_particles_weight += current_particle->weight;
	}
	return sum_all_particles_weight;
}

} /* namespace amcl */
