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

#include "MarkerListVisualization.hh"
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/stock_objects.h>
#include <string>

MarkerListVisualization::MarkerListVisualization(CDisplayWindow3D& window3D, const std::string& identifier) :
	AbstractVisualization(window3D, identifier) {

	max_markers = 10;
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
		{
			for(size_t i = 0; i< max_markers; ++i)
			{
			auto marker_pose_obj = mrpt::opengl::stock_objects::CornerXYZSimple(0.5);
			marker_pose_obj->setName(identifier + "_marker_" + std::to_string(i));
			ptrScene->insert(marker_pose_obj);

			opengl::CTextPtr marker_tag_text_obj = opengl::CText::Create();
			marker_tag_text_obj->setName(identifier + "_marker_tag_text_" + std::to_string(i));
			ptrScene->insert(marker_tag_text_obj);

			}
		}
		window3D.unlockAccess3DScene();
}

MarkerListVisualization::~MarkerListVisualization() {
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		for(size_t i = 0; i< max_markers; ++i)
		{
			opengl::CRenderizablePtr obj1 = ptrScene->getByName(identifier + "_marker_" + std::to_string(i));
			ptrScene->removeObject(obj1);

			opengl::CRenderizablePtr marker_tag_text_obj = ptrScene->getByName(identifier + "_marker_tag_text_" + std::to_string(i));
			ptrScene->removeObject(marker_tag_text_obj);
		}

	}
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}

void MarkerListVisualization::displayMarkerList(const CommTrackingObjects::CommDetectedMarkerList& marker_list)
{
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{


		// we have `max_markers` number of marker objects ex: 10
		size_t num_markers = marker_list.getMarkersSize();
        size_t num_show_markers = (num_markers < max_markers) ? num_markers : max_markers;

		// display pose of markers found ex: 3
        for(size_t i =0; i< num_show_markers; ++i)
		{

			CommTrackingObjects::CommDetectedMarker current_marker = marker_list.getMarkersElemAtPos(i);
			CommBasicObjects::CommPose3d p = marker_list.get_tag_pose_in_robot_frame_by_index(i);

			mrpt::poses::CPose3D marker_pose(p.get_x(1.0), p.get_y(1.0), p.get_z(1.0), p.get_azimuth(), p.get_elevation(), p.get_roll());


			opengl::CRenderizablePtr obj1 = ptrScene->getByName(identifier + "_marker_" + std::to_string(i));
			if(obj1)
			obj1->setPose(marker_pose);
			else
			std::cout << "obj1 is null " << "\n";

			// set tag id text
			opengl::CTextPtr marker_tag_text_obj = (opengl::CTextPtr)ptrScene->getByName(identifier + "_marker_tag_text_" + std::to_string(i));
			marker_tag_text_obj->setString(std::to_string(current_marker.getId()));

			mrpt::poses::CPose3D marker_text_pose(p.get_x(1.0), p.get_y(1.0), p.get_z(1.0), p.get_azimuth(), p.get_elevation(), p.get_roll());
			marker_tag_text_obj->setPose(marker_text_pose);

		}

		//for remaining display objects(ex: 3 = 10-7), show them at origin and tag = ""
        for(size_t i =num_show_markers; i< max_markers; ++i)
		{
			mrpt::poses::CPose3D marker_pose(0, 0, 0, 0, 0, 0);
			opengl::CRenderizablePtr obj1 = ptrScene->getByName(identifier + "_marker_" + std::to_string(i));
			if(obj1)
				obj1->setPose(marker_pose);
			else
				std::cout << "obj1 is null " << "\n";

			// set default tag id text
			opengl::CTextPtr marker_tag_text_obj = (opengl::CTextPtr)ptrScene->getByName(identifier + "_marker_tag_text_" + std::to_string(i));
			marker_tag_text_obj->setString(std::string(""));
			marker_tag_text_obj->setPose(marker_pose);


		}


	}
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();

}


void MarkerListVisualization::clear()
{
}


