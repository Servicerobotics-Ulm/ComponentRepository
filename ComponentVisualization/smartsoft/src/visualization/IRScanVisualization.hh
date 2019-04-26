// --------------------------------------------------------------------------
//
//  Copyright (C) 2014 Matthias Lutz
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

#ifndef IRVISUALIZATION_HH_
#define IRVISUALIZATION_HH_

#include "AbstractVisualization.hh"

#include <CommBasicObjects/CommMobileIRScan.hh>

class IRScanVisualization: public AbstractVisualization
{
private:
	struct ColPoint3d
	{
		ColPoint3d(float x, float y, float z, float r, float g, float b)
		{
			this->x = x;
			this->y = y;
			this->z = z;

			this->r = r;
			this->g = g;
			this->b = b;
		}

		float x, y, z;
		float r, g, b;
	};

public:
	IRScanVisualization(CDisplayWindow3D& window3D, const std::string& identifier);
	virtual ~IRScanVisualization();

	void displayScan(CommBasicObjects::CommMobileIRScan& scan);
	void clear();
};

#endif /* USARVISUALIZATION_HH_ */
