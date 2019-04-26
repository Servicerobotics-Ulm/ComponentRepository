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

#ifndef ABSTRACTVISUALIZATION_H_
#define ABSTRACTVISUALIZATION_H_

#include <string>

#include <mrpt/gui/CDisplayWindow3D.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::gui;

class AbstractVisualization {
protected:
	CDisplayWindow3D& window3D;
	std::string identifier;

public:
	AbstractVisualization(CDisplayWindow3D& window3D, const std::string& identifier);
	virtual ~AbstractVisualization();

};

#endif /* ABSTRACTVISUALIZATION_H_ */
