/*--------------------------------------------------------------------------

 Copyright (C) 2020

 Created on: Nov 09, 2020
 Author    : Nayabrasul Shaik (nayabrasul.shaik@thu.de)

 ZAFH Servicerobotik Ulm
 University of Applied Sciences
 Prittwitzstr. 10
 D-89075 Ulm
 Germany

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2.1
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this library; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

 --------------------------------------------------------------------------*/

#ifndef LOCATIONPOSEVISUALIZATION_H_
#define LOCATIONPOSEVISUALIZATION_H_

#include "AbstractVisualization.hh"

#include "CommBasicObjects/CommBaseState.hh"
struct VizConfig{
	std::string label_prefix;
	double color[3];
};
class PoseVisualization : public AbstractVisualization {
private:
	static const std::string id_location_pose_obj;
	static const std::string id_location_orientation_obj;
	static const std::string id_location_label_obj;

public:
	PoseVisualization(CDisplayWindow3D& window3D, const std::string& identifier);
	virtual ~PoseVisualization();

	void displayPose(const CommBasicObjects::CommPose3d& pose, const VizConfig& config);
	void clear();

};

#endif /* LOCATIONPOSEVISUALIZATION_H_ */
