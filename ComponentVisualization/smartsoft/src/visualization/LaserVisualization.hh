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

#ifndef LASERVISUALIZATION_H_
#define LASERVISUALIZATION_H_

#include "AbstractVisualization.hh"

#include "CommBasicObjects/CommMobileLaserScan.hh"

class LaserVisualization : public AbstractVisualization {
public:
	LaserVisualization(CDisplayWindow3D& window3D, const std::string& identifier);
	virtual ~LaserVisualization();

	void displayLaserScan(const CommBasicObjects::CommMobileLaserScan& scan);
	void clear();

private:
	static double pi_to_pi(double angle) {
		angle += M_PI;
		double ret_angle = fmod(angle, 2* M_PI );

		if (angle < 0)
			ret_angle += 2* M_PI ;

		ret_angle -= M_PI;

		return ret_angle;
	}
};

#endif /* LASERVISUALIZATION_H_ */
