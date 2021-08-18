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
#include "PlannerGridTask.hh"

#include <mrpt/opengl.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CTexturedPlane.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;
#ifdef WITH_MRPT_2_0_VERSION
using namespace mrpt::img;
#include <mrpt/img/CImage.h>
#else
using namespace mrpt::utils;
#endif

GridMapVisualization::GridMapVisualization(CDisplayWindow3D& window3D, const VizConfig& config) :
	AbstractVisualization(window3D, config.identifier) {

	this->showAxis = config.showAxis;
	this->activateTransparency = config.activateTransparency;
	this->identifier = config.identifier;
	this->mapType = config.mapType;

#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CTexturedPlane::Ptr objs = opengl::CTexturedPlane::Create();
		objs->setName(identifier + "_ltm");
		objs->setPlaneCorners(0, 1, 0, 1);
		ptrScene->insert(objs);

		if (showAxis) {
			opengl::CAxis::Ptr axis = opengl::CAxis::Create(-10, -10, 0, 10, 10, 2, 1, 1, true);
			axis->setName(identifier + "_axes");
			axis->setColor(0, 0, 0);
			axis->enableTickMarks();
			ptrScene->insert(axis);
		}

		mrpt::opengl::CText::Ptr gl_txt = mrpt::opengl::CText::Create();
		gl_txt->setName(identifier + "_text");
		ptrScene->insert(gl_txt);


	}
#else
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
#endif
	window3D.unlockAccess3DScene();

}

GridMapVisualization::~GridMapVisualization() {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
		{
			opengl::CTexturedPlane::Ptr ptrPlane = std::dynamic_pointer_cast<opengl::CTexturedPlane>(ptrScene->getByName(identifier + "_ltm"));
			ptrScene->removeObject(ptrPlane);

			if (showAxis) {
				opengl::CAxis::Ptr ptrAxis = std::dynamic_pointer_cast<opengl::CAxis>(ptrScene->getByName(identifier + "_axis"));
				ptrScene->removeObject(ptrAxis);
			}

			mrpt::opengl::CText::Ptr gl_txt = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + "_text"));
			ptrScene->removeObject(gl_txt);

		}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
		{
			opengl::CTexturedPlanePtr ptrPlane = (opengl::CTexturedPlanePtr) ptrScene->getByName(identifier + "_ltm");
			ptrScene->removeObject(ptrPlane);

			if (showAxis) {
				opengl::CAxisPtr ptrAxis = (opengl::CAxisPtr) ptrScene->getByName(identifier + "_axis");
				ptrScene->removeObject(ptrAxis);
			}

		}
#endif
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

	//std::cout << "id : "<< (int)id <<std::endl;
	//std::cout << "x : " << xmin << " : " << xmax << "       y : " << ymin << " : " << ymax <<std::endl;
	//std::cout << "xSizeCells : "<< xSizeCells << "  ySizeCells : " << ySizeCells <<std::endl;
	//std::cout << "cellSizeMM : "<< cellSizeMM <<std::endl;
	//std::cout << "xOffsetCells : "<< xOffsetCells << "  yOffsetCells : " << yOffsetCells <<std::endl;


	//	The longterm map holds cell values from 0 to 255. Values from 0 to 127
	//	denote the traversability where 0 is completely free. Values from 128 to 255
	//	are special values: Obstacles are marked with 128, cells occupied by obstacle
	//	growing with 129 and undeletable grids are marked with 130. The cell values
	//	can be accumulated over time to represent the environment over a longer period.

#ifdef WITH_MRPT_2_0_VERSION
//try{
	//mrpt::img::CImage imgColor(xSizeCells, ySizeCells, mrpt::img::TImageChannels::CH_RGB);
	mrpt::img::CImage::Ptr imgColor = std::make_shared<mrpt::img::CImage>(xSizeCells, ySizeCells, mrpt::img::TImageChannels::CH_RGB);
	//mrpt::img::CImage imgTrans(xSizeCells, ySizeCells, mrpt::img::TImageChannels::CH_GRAY);
	mrpt::img::CImage::Ptr imgTrans = std::make_shared<mrpt::img::CImage>(xSizeCells, ySizeCells, mrpt::img::TImageChannels::CH_GRAY);
//}
//catch(mrpt::utils::CExceptionEOF &e)
//{
  // This was a EOF error
//  std::cerr << e.what() << std::endl;
//  return 1;
//}
#else
	utils::CImage imgColor(xSizeCells, ySizeCells, 3);
	utils::CImage imgTrans(xSizeCells, ySizeCells, 1);
#endif

	//std::cout<<"displayGridMap: xSizeCells: "<<xSizeCells<<" ySizeCells: "<<ySizeCells <<std::endl;

	for (uint32_t j = 0; j < ySizeCells; j++) {
		for (uint32_t i = 0; i < xSizeCells; i++) {

			uint8_t cell = map.get_cells(i, j);
#ifdef WITH_MRPT_2_0_VERSION

			//map is a current map or LTM map from mapper component
			if(mapType == MapType::LTM_MAP || mapType == MapType::CURRENT_MAP)
			{
			imgTrans->setPixel(i, j, img::TColor::white());
			if (cell >=0 && cell <=127) {
				//free cell
				imgColor->setPixel(i, j, img::TColor::white());
			} else if (cell == MAPPER_OBSTACLE) {
				imgColor->setPixel(i, j, img::TColor::black());
			} else if (cell == MAPPER_GROWING) {
				imgColor->setPixel(i, j, img::TColor::blue());
			} else if (cell == MAPPER_UNDELETABLE) {
				imgColor->setPixel(i, j, img::TColor::red());
			} else if (cell == MAPPER_UNKNOWN) {
				imgColor->setPixel(i, j, img::TColor::gray());
			}else{
				std::cout << (int)cell <<", ";
			}
			}else if(mapType == MapType::PLANNER_MAP) // map from planner containing the wavefront
			{
				if(cell >=8 && cell <=15){ // path planned by planner
					imgColor->setPixel(i, j, img::TColor(64, 128, 128));
				}
				else if (cell == PLANNER_OBSTACLE) {
					imgColor->setPixel(i, j, img::TColor::black());
				} else if (cell == PLANNER_GROWING){
					imgColor->setPixel(i, j, img::TColor(64, 64, 64));
				}else if (cell == PLANNER_GOAL){
					imgColor->setPixel(i, j, img::TColor::blue());
				}else if (cell == PLANNER_START){
					imgColor->setPixel(i, j, img::TColor::green());
				}else if (cell == PLANNER_FREE){
					imgColor->setPixel(i, j, img::TColor::white());
				}else if (cell == PLANNER_NORTH){
					imgColor->setPixel(i, j, img::TColor(128, 64, 64));
				}else if (cell == PLANNER_WEST){
					imgColor->setPixel(i, j, img::TColor(128, 96, 96));
				}else if (cell == PLANNER_SOUTH){
					imgColor->setPixel(i, j, img::TColor(128, 160, 160));
				}else if (cell == PLANNER_EAST){
					imgColor->setPixel(i, j, img::TColor(128, 192, 192));
				}else if (cell == 99){// special value: planner default
					imgColor->setPixel(i, j, img::TColor(64, 128, 128));
				}else{
					std::cout << (int)cell << ",";
				}
				imgTrans->setPixel(i, j, img::TColor::white());
			}
#else
			*imgTrans(i, j) = 255;
			imgColor.setPixel(i, j, utils::TColor(cell, cell, cell));

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
#endif
		}
	}
	//	w1.showImage(imgColor);
	//	w2.showImage(imgTrans);
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CTexturedPlane::Ptr ptrPlane = std::dynamic_pointer_cast<opengl::CTexturedPlane>(ptrScene->getByName(identifier + "_ltm"));


		if((xmin !=0 && xmax !=0)&&(ymin !=0 && ymax !=0))
		{
		ptrPlane->setPlaneCorners(xmin, xmax, ymin, ymax);

        // assign new current map
		ptrPlane->assignImage(*imgColor, *imgTrans);
		ptrPlane->notifyChange();

		if (showAxis) {
			opengl::CAxis::Ptr ptrAxis = std::dynamic_pointer_cast<opengl::CAxis>(ptrScene->getByName(identifier + "_axis"));
			ptrAxis->setAxisLimits(xmin, ymin, 0, xmax, ymax, 2);
		}

		mrpt::opengl::CText::Ptr gl_txt  = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + "_text"));
		gl_txt->setString(identifier+ " id = " + std::to_string(id));
		gl_txt->setLocation(0.0, 0.0, 1.0);
		}else{
			std::cout << "Grid visualization skipped, xmin, xmax, ymin, ymax = " << xmin << ", " << xmax << ", " << ymin << ", " << ymax << std::endl;
		}

	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{

		//clean the prevous map
		mrpt::utils::CImage previous_map = ptrPlane->getTextureImage();
        size_t width  = previous_map.getWidth();
        size_t height = previous_map.getHeight();

		for (uint32_t j = 0; j < height; j++) {
			for (uint32_t i = 0; i < width; i++) {
				previous_map.setPixel(i, j, img::TColor::gray());
			}
		}


		opengl::CTexturedPlanePtr ptrPlane = (opengl::CTexturedPlanePtr) ptrScene->getByName(identifier + "_ltm");
		ptrPlane->setPlaneCorners(xmin, xmax, ymin, ymax);
		ptrPlane->assignImage(imgColor, imgTrans);

		if (showAxis) {
			opengl::CAxisPtr ptrAxis = (opengl::CAxisPtr) ptrScene->getByName(identifier + "_axis");
			ptrAxis->setAxisLimits(xmin, ymin, 0, xmax, ymax, 2);
		}

	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}

void GridMapVisualization::clear() {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CTexturedPlane::Ptr ptrPlane = std::dynamic_pointer_cast<opengl::CTexturedPlane>(ptrScene->getByName(identifier + "_ltm"));
		if(!ptrPlane){
			ptrPlane->setPlaneCorners(0, 1, 0, 1);
		} else {
			//std::cout<<" clear on NULL empty!"<<std::endl;
		}
		//mrpt::opengl::CText::Ptr gl_txt  = std::dynamic_pointer_cast<opengl::CText>(ptrScene->getByName(identifier + "_text"));
		//gl_txt->setString("map id = " + std::to_string(id));
	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CTexturedPlanePtr ptrPlane = (opengl::CTexturedPlanePtr) ptrScene->getByName(identifier + "_ltm");
		if(ptrPlane.null() == false){
			ptrPlane->setPlaneCorners(0, 0, 0, 0);
		} else {
			//std::cout<<" clear on NULL empty!"<<std::endl;
		}
	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}
