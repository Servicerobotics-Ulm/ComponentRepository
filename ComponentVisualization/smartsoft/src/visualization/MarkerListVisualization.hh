// --------------------------------------------------------------------------
//
//  Copyright (C) 2019 Nayabrasul Shaik
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

#ifndef SMARTSOFT_SRC_VISUALIZATION_MARKERLISTVISUALIZATION_HH_
#define SMARTSOFT_SRC_VISUALIZATION_MARKERLISTVISUALIZATION_HH_

#include "AbstractVisualization.hh"
#include "CommTrackingObjects/CommDetectedMarkerList.hh"
#include "CommTrackingObjects/CommDetectedMarker.hh"

class MarkerListVisualization: public AbstractVisualization {
	size_t max_markers;
public:
	MarkerListVisualization(CDisplayWindow3D& window3D, const std::string& identifier);
	virtual ~MarkerListVisualization();

	void displayMarkerList(const CommTrackingObjects::CommDetectedMarkerList& marker_list);
    void clear();
};

#endif /* SMARTSOFT_SRC_VISUALIZATION_MARKERLISTVISUALIZATION_HH_ */
