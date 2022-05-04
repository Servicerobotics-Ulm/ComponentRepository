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

#ifndef BASEVISUALIZATION_H_
#define BASEVISUALIZATION_H_

#include "AbstractVisualization.hh"

#include "CommBasicObjects/CommBaseState.hh"

class BaseVisualization : public AbstractVisualization {
private:
	static const std::string id_robot_pose_obj;
	static const std::string id_robot_orientation_obj;
	static const std::string id_robot_odom_pose_obj;
	static const std::string id_robot_odom_orientation_obj;
	static const std::string id_robot_label1_obj;
	static const std::string id_robot_label2_obj;
	static const std::string id_robot_trajectory_pose;
	static const std::string id_robot_trajectory_orient;
	static const std::string id_robot_trajectory_odom_pose;
	static const std::string id_robot_trajectory_odom_orient;
	static const std::string id_robot_future_trajectory_pose;
	static const std::string id_robot_future_trajectory_orient;

public:
	BaseVisualization(CDisplayWindow3D& window3D, const std::string& identifier, bool show_traj);
	virtual ~BaseVisualization();

	void displayBase(const CommBasicObjects::CommBaseState& pos);
	void clear();
#ifdef WITH_MRPT_2_0_VERSION
	bool show_trajectory;
	void set_show_trajectory(bool in_flag);
#endif
};

#endif /* BASEVISUALIZATION_H_ */
