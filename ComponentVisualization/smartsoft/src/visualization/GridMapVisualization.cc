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

#include "GridMapVisualization.hh"
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/CAxis.h>


GridMapVisualization::GridMapVisualization(CDisplayWindow3D& window3D, const std::string& identifier, bool showAxis,
		bool activateTransparency) :
	AbstractVisualization(window3D, identifier) {

	this->showAxis = showAxis;
	this->activateTransparency = activateTransparency;

	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CTexturedPlanePtr objs = opengl::CTexturedPlane::Create();
		objs->setName(identifier + "_ltm");
		objs->setPlaneCorners(0, 0, 0, 0);
		ptrScene->insert(objs);

		if (showAxis) {
			opengl::CAxisPtr axis = opengl::CAxis::Create(-10, -10, 0, 10, 10, 2, 1, 1, true);
			axis->setName(identifier + "_axis");
			axis->setColor(0, 0, 0);
			axis->enableTickMarks();
			ptrScene->insert(axis);
		}
	}
	window3D.unlockAccess3DScene();

}

GridMapVisualization::~GridMapVisualization() {
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
		{
			opengl::CTexturedPlanePtr ptrPlane = (opengl::CTexturedPlanePtr) ptrScene->getByName(identifier + "_ltm");
			ptrScene->removeObject(ptrPlane);

			if (showAxis) {
				opengl::CAxisPtr ptrAxis = (opengl::CAxisPtr) ptrScene->getByName(identifier + "_axis");
				ptrScene->removeObject(ptrAxis);
			}

		}
		window3D.unlockAccess3DScene();
		window3D.forceRepaint();
}

void GridMapVisualization::displayGridMap(const CommNavigationObjects::CommGridMap& map) {

	bool is_valid = false;
	timeval time;
	uint32_t id, cellSizeMM, xSizeMM, ySizeMM, xSizeCells, ySizeCells;
	int xOffsetMM, yOffsetMM, xOffsetCells, yOffsetCells;

	map.get_parameter(id, is_valid, time, xOffsetMM, yOffsetMM, xOffsetCells, yOffsetCells, cellSizeMM, xSizeMM, ySizeMM, xSizeCells,
			ySizeCells);

	float xmin = (float) xOffsetMM / 1000.0;
	float ymin = (float) yOffsetMM / 1000.0;
	float xmax = (float) (xOffsetMM + xSizeMM) / 1000.0;
	float ymax = (float) (yOffsetMM + ySizeMM) / 1000.0;

	//	The longterm map holds cell values from 0 to 255. Values from 0 to 127
	//	denote the traversability where 0 is completely free. Values from 128 to 255
	//	are special values: Obstacles are marked with 128, cells occupied by obstacle
	//	growing with 129 and undeletable grids are marked with 130. The cell values
	//	can be accumulated over time to represent the environment over a longer period.

	utils::CImage imgColor(xSizeCells, ySizeCells, 3);
	utils::CImage imgTrans(xSizeCells, ySizeCells, 1);

	std::cout<<"displayGridMap: xSizeCells: "<<xSizeCells<<" ySizeCells: "<<ySizeCells <<std::endl;
	for (uint32_t j = 0; j < ySizeCells; j++) {
		for (uint32_t i = 0; i < xSizeCells; i++) {

			uint8_t cell = map.get_cells(i, j);
			*imgTrans(i, j) = 255;

			if (cell <= 127) {
				//free
				uint8_t cell255 = 254 - (cell * 2);
				imgColor.setPixel(i, j, utils::TColor(cell255, cell255, cell255));

			//	if (activateTransparency) {
			//		*imgTrans(i, j) = 255 - cell255;
			//	}

			} else if (cell == 128) {
				// obstacle
				imgColor.setPixel(i, j, utils::TColor::black);
			} else if (cell == 129) {
				// obstacle growing
				imgColor.setPixel(i, j, utils::TColor::blue);
			} else if (cell == 130) {
				//undeletable grids
				imgColor.setPixel(i, j, utils::TColor::red);
			} else if (cell == 205) {
				//unkown grids
				imgColor.setPixel(i, j, utils::TColor::gray);
			}

		}
	}

	//	w1.showImage(imgColor);
	//	w2.showImage(imgTrans);

	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CTexturedPlanePtr ptrPlane = (opengl::CTexturedPlanePtr) ptrScene->getByName(identifier + "_ltm");
		ptrPlane->setPlaneCorners(xmin, xmax, ymin, ymax);
		ptrPlane->assignImage(imgColor, imgTrans);

		if (showAxis) {
			opengl::CAxisPtr ptrAxis = (opengl::CAxisPtr) ptrScene->getByName(identifier + "_axis");
			ptrAxis->setAxisLimits(xmin, ymin, 0, xmax, ymax, 2);
		}

	}
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}

void GridMapVisualization::clear() {
	std::cout<<__FUNCTION__<<" : "<<__LINE__<<std::endl;
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		std::cout<<__FUNCTION__<<" : "<<__LINE__<<std::endl;
		opengl::CTexturedPlanePtr ptrPlane = (opengl::CTexturedPlanePtr) ptrScene->getByName(identifier + "_ltm");
		std::cout<<__FUNCTION__<<" : "<<__LINE__<<std::endl;
		if(ptrPlane.null() == false){
			std::cout<<__FUNCTION__<<" : "<<__LINE__<<std::endl;
			ptrPlane->setPlaneCorners(0, 0, 0, 0);
		} else {
			std::cout<<__FUNCTION__<<" : "<<__LINE__<<std::endl;
			std::cout<<__FUNCTION__<<" clear on NULL empty!"<<std::endl;
		}
	}
	std::cout<<__FUNCTION__<<" : "<<__LINE__<<std::endl;
	window3D.unlockAccess3DScene();
	std::cout<<__FUNCTION__<<" : "<<__LINE__<<std::endl;
	window3D.forceRepaint();
	std::cout<<__FUNCTION__<<" : "<<__LINE__<<std::endl;
}
