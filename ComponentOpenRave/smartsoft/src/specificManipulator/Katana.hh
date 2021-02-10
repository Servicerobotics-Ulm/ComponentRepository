//--------------------------------------------------------------------------
//  Copyright (C) 2010 Jonas Brich
//
//        brich@mail.hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartOpenRave component".
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
//---------------------------------------------------------------------

#ifndef _KATANA_HH
#define _KATANA_HH

#include "Manipulator.hh"
#include <string>

namespace SpecificManipulator {

/**
 * Manipulator specific class which includes all Katana specific code.
 */
class Katana: public Manipulator {

public:
	Katana();

	bool convertRealAnglesToOpenRaveAngles(const std::vector<double>& realAngles, std::vector<double>& openRaveAngles);
	bool convertOpenRaveAnglesToRealAngles(const std::vector<double>& openRaveAngles, std::vector<double>& realAngles);

	void getActivDofsWithoutGripper(std::vector<int>& dofs);

	const vector<int>& getActiveDofs();
	const vector<int>& getActiveArmDofs();
	const vector<int>& getActiveGripperDofs();


	bool removeDummyJointAngles(std::vector<double>& angles);

	arma::mat getOffsetRealManipulatortoOpenRaveManipulator();
	arma::mat getSimulationTransform();
	void getSimulationJointAngles(std::vector<double>& angles);

	//int parseTrajectory(std::vector<std::vector<double> >& traj);

	void parseIterParameters(ORUtil::PlanningParameter& params, ORUtil& orutil);

	virtual ~Katana();

private:
	// Values which are necessary for transformation from real gripper angles to OpenRave gripper angles
	double center;
	double offset;
	double factor;

	// Manipulator offset coordinates
	double manipulatorCoord_x;
	double manipulatorCoord_y;
	double manipulatorCoord_z;

	vector<int> activeDofs;
	vector<int> activeArmDofs;
	vector<int> activeGripperDofs;

};

}

#endif /* _KATANA_HH */
