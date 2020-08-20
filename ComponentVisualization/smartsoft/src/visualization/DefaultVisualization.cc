/*--------------------------------------------------------------------------

 Copyright (C) 2017

 Created on: Oct 27, 2017
 Author    : Nayabrasul Shaik (shaik@hs-ulm.de)

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

#include "DefaultVisualization.hh"
#include <mrpt/opengl.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CAxis.h>


using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;
#ifdef WITH_MRPT_2_0_VERSION
#else
using namespace mrpt::utils;
#endif
//#include <cmath>
#include <ctime>
using namespace std;

DefaultVisualization::DefaultVisualization(CDisplayWindow3D& window3D, const std::string& identifier):
AbstractVisualization(window3D, identifier)
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
		{
			// coordinate axis
			opengl::CAxis::Ptr gl_axis = opengl::CAxis::Create(-10,-10, 0, 10, 10, 2, 1);
			gl_axis->setName(identifier + "_axis");
			ptrScene->insert( gl_axis );

			//2d grid
			mrpt::opengl::CGridPlaneXY::Ptr gl_grid = mrpt::opengl::CGridPlaneXY::Create();
			gl_grid->setName(identifier + "_grid");
			gl_grid->setColor(0.6,0.6,0.6);
			ptrScene->insert( gl_grid );

			//origin
			mrpt::opengl::CSetOfObjects::Ptr gl_corner = mrpt::opengl::stock_objects::CornerXYZ();
			gl_corner->setName(identifier + "_origin");
			gl_corner->setScale(0.5);
			ptrScene->insert(gl_corner);
		}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
		{


			// coordinate axis
			opengl::CAxisPtr gl_axis = opengl::CAxis::Create(-10,-10, 0, 10, 10, 2, 1);
			gl_axis->setName(identifier + "_axis");
			ptrScene->insert( gl_axis );

			//2d grid
			mrpt::opengl::CGridPlaneXYPtr gl_grid = mrpt::opengl::CGridPlaneXY::Create();
			gl_grid->setName(identifier + "_grid");
			gl_grid->setColor(0.6,0.6,0.6);
			ptrScene->insert( gl_grid );


			//origin
			mrpt::opengl::CSetOfObjectsPtr gl_corner = mrpt::opengl::stock_objects::CornerXYZ();
			gl_corner->setName(identifier + "_origin");
			gl_corner->setScale(0.5);
			ptrScene->insert(gl_corner);

		}
#endif
		window3D.unlockAccess3DScene();



}

DefaultVisualization::~DefaultVisualization() {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr & ptrScene = window3D.get3DSceneAndLock();
		{
			// remove axis
			opengl::CAxis::Ptr axis = std::dynamic_pointer_cast<opengl::CAxis>(ptrScene->getByName(identifier + "_axis"));
			ptrScene->removeObject(axis);

			// remove grid
			mrpt::opengl::CSetOfObjects::Ptr gl_grid = std::dynamic_pointer_cast<opengl::CSetOfObjects>(ptrScene->getByName(identifier + "_grid"));
			ptrScene->removeObject(gl_grid);

			// remove origin
			mrpt::opengl::CSetOfObjects::Ptr gl_corner = std::dynamic_pointer_cast<opengl::CSetOfObjects>(ptrScene->getByName(identifier + "_origin"));
			ptrScene->removeObject(gl_corner);

		}
#else
	opengl::COpenGLScenePtr & ptrScene = window3D.get3DSceneAndLock();
		{
			// remove axis
			opengl::CAxisPtr axis = (opengl::CAxisPtr) ptrScene->getByName(identifier + "_axis");
			ptrScene->removeObject(axis);

			// remove grid
			mrpt::opengl::CSetOfObjectsPtr gl_grid = (opengl::CSetOfObjectsPtr) ptrScene->getByName(identifier + "_grid");
			ptrScene->removeObject(gl_grid);

			// remove origin
			mrpt::opengl::CSetOfObjectsPtr gl_corner = (opengl::CSetOfObjectsPtr) ptrScene->getByName(identifier + "_origin");
			ptrScene->removeObject(gl_corner);

		}
#endif
		window3D.unlockAccess3DScene();
		window3D.forceRepaint();
}
