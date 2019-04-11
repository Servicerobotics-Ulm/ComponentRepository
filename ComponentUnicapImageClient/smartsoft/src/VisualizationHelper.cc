//------------------------------------------------------------------------
//
//  Copyright (C) 2011 Manuel Wopfner
//
//        wopfner@hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartColorToFObjectRecognition".
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
//--------------------------------------------------------------------------

#include "VisualizationHelper.hh"

VisualizationHelper::VisualizationHelper(uint32_t numberOfImageWindows, uint32_t numberOf3DWindows) :
	window2D(numberOfImageWindows), window3D(numberOf3DWindows), cloud3D(numberOf3DWindows), lines3D(numberOf3DWindows) {

	for (size_t i = 0; i < window2D.size(); ++i) {
		window2D[i] = new CDisplayWindow();
		window2D[i]->setPos(i * 215, 0);
	}

	for (size_t i = 0; i < window3D.size(); ++i) {
		window3D[i] = new CDisplayWindow3D();
		window3D[i]->setPos(i * 215, 260);

		window3D[i]->setCameraProjective(true);
		window3D[i]->setCameraElevationDeg(25.0f);
		window3D[i]->setCameraAzimuthDeg(160.0f);
		window3D[i]->setCameraPointingToPoint(0.5, 0, 0.7);
		window3D[i]->setCameraZoom(2.0f);

		cloud3D[i] = opengl::CPointCloudColoured::Create();
		cloud3D[i]->setName("cloud");
		cloud3D[i]->setColor(mrpt::utils::TColorf(1));
		cloud3D[i]->setPointSize(2.0);

		lines3D[i] = opengl::CSetOfLines::Create();
		lines3D[i]->setName("lines");

		COpenGLScenePtr &theScene = window3D[i]->get3DSceneAndLock();

		theScene->insert(cloud3D[i]);
		theScene->insert(lines3D[i]);

		window3D[i]->unlockAccess3DScene();
		window3D[i]->forceRepaint();

	}

}

uint32_t VisualizationHelper::addWindow() {

	size_t i = window2D.size() - 1;
	window2D.resize(i + 1);

	window2D[i] = new CDisplayWindow();
	window2D[i]->setPos(i * 215, 0);

	return i;
}

uint32_t VisualizationHelper::addWindow3D() {

	size_t i = window3D.size() - 1;
	window3D.resize(i + 1);
	cloud3D.resize(i + 1);
	lines3D.resize(i + 1);

	window3D[i] = new CDisplayWindow3D();
	window3D[i]->setPos(i * 215, 260);

	window3D[i]->setCameraProjective(true);
	window3D[i]->setCameraElevationDeg(25.0f);
	window3D[i]->setCameraAzimuthDeg(160.0f);
	window3D[i]->setCameraPointingToPoint(0.5, 0, 0.7);
	window3D[i]->setCameraZoom(2.0f);

	cloud3D[i] = opengl::CPointCloudColoured::Create();
	cloud3D[i]->setName("cloud");
	cloud3D[i]->setColor(mrpt::utils::TColorf(1));
	cloud3D[i]->setPointSize(2.0);

	lines3D[i] = opengl::CSetOfLines::Create();
	lines3D[i]->setName("lines");

	COpenGLScenePtr &theScene = window3D[i]->get3DSceneAndLock();

	theScene->insert(cloud3D[i]);
	theScene->insert(lines3D[i]);

	window3D[i]->unlockAccess3DScene();
	window3D[i]->forceRepaint();

	return i;
}


//void VisualizationHelper::showPointCloud(uint32_t windowId, PointCloud& cloud, float r, float g, float b) {
//	double x, y, z;
//
//	lockWindow3D(windowId);
//	opengl::CPointCloudColouredPtr glCloud = getCloud(windowId);
//	glCloud->clear();
//
//	for (size_t i = 0; i < cloud.size(); ++i) {
//		cloud.getPoint(i, x, y, z);
//		//std::cout <<"x y z"<<x<<" "<<y<<" "<<z<<"\n";
//		glCloud->push_back(x, y, z, r, g, b);
//	}
//
//	unlockWindow3D(windowId);
//}

void VisualizationHelper::show3DBox(uint32_t windowId, TPoint3D& point1,TPoint3D& point2, CPose3D* pose) {
	CPoint3D p1(point1);
	CPoint3D p2(point2);
	show3DBox(windowId, p1,p2, pose);
}

void VisualizationHelper::clearScene(uint32_t windowId) {
	COpenGLScenePtr &theScene = window3D[windowId]->get3DSceneAndLock();
	theScene->clear();

	cloud3D[windowId]->clear();
	theScene->insert(cloud3D[windowId]);
	theScene->insert(lines3D[windowId]);

	unlockWindow3D(windowId);
}

void VisualizationHelper::saveScene(uint32_t windowId, string filename) {
	COpenGLScenePtr &theScene = window3D[windowId]->get3DSceneAndLock();
	theScene->saveToFile(filename);
	unlockWindow3D(windowId);
}

void VisualizationHelper::showText(uint32_t windowId, std::string text, double x, double y, double z) {
	COpenGLScenePtr &theScene = window3D[windowId]->get3DSceneAndLock();

	opengl::CTextPtr txt = opengl::CText::Create(text);
	txt->setLocation(x, y, z);
	theScene->insert(txt);

	unlockWindow3D(windowId);
}

void VisualizationHelper::show3DBox(uint32_t windowId, CPoint3D& point1,CPoint3D& point2, CPose3D* pose) {

	COpenGLScenePtr &theScene = window3D[windowId]->get3DSceneAndLock();

	opengl::CBoxPtr glbox = opengl::CBox::Create(point1, point2, true);
	if(pose != NULL)
		glbox->setPose(*pose);
	theScene->insert(glbox);

	unlockWindow3D(windowId);
}

void VisualizationHelper::showArrow(uint32_t windowId, TPoint3D& point1,TPoint3D& point2) {
	COpenGLScenePtr &theScene = window3D[windowId]->get3DSceneAndLock();

	opengl::CArrowPtr glarrow = opengl::CArrow::Create(point1.x, point1.y, point1.z, point2.x, point2.y, point2.z, 0.02f, 0.005f, 0.02f);
	theScene->insert(glarrow);

	unlockWindow3D(windowId);
}

void VisualizationHelper::showArrow(uint32_t windowId, float x1, float y1, float z1, float x2, float y2, float z2) {
	COpenGLScenePtr &theScene = window3D[windowId]->get3DSceneAndLock();

	opengl::CArrowPtr glarrow = opengl::CArrow::Create(x1, y1, z1, x2, y2, z2, 0.02f, 0.005f, 0.02f);
	theScene->insert(glarrow);

	unlockWindow3D(windowId);
}

void VisualizationHelper::showLine(uint32_t windowId, float x1, float y1, float z1, float x2, float y2, float z2) {
	COpenGLScenePtr &theScene = window3D[windowId]->get3DSceneAndLock();

	opengl::CSimpleLinePtr glarrow = opengl::CSimpleLine::Create(x1, y1, z1, x2, y2, z2, 0.02f);
	theScene->insert(glarrow);

	unlockWindow3D(windowId);
}

void VisualizationHelper::showYZPlane(uint32_t windowID, float xmin, float xmax, float zmin, float zmax, float y, float frequency){

	COpenGLScenePtr &theScene = window3D[windowID]->get3DSceneAndLock();

	opengl::CGridPlaneXZPtr plane = opengl::CGridPlaneXZ::Create(xmin,xmax,zmin,zmax,y,frequency);
	plane->setPose(CPose3D(0,0,0,-M_PI_2,0,0));
	theScene->insert(plane);

	unlockWindow3D(windowID);
}

void VisualizationHelper::showXZPlane(uint32_t windowID, float xmin, float xmax, float zmin, float zmax, float x, float frequency){

	COpenGLScenePtr &theScene = window3D[windowID]->get3DSceneAndLock();

	opengl::CGridPlaneXZPtr plane = opengl::CGridPlaneXZ::Create(xmin,xmax,zmin,zmax,x,frequency);
	theScene->insert(plane);

	unlockWindow3D(windowID);
}

void VisualizationHelper::showXYPlane(uint32_t windowID, float xmin, float xmax, float ymin, float ymax, float z, float frequency ){

	COpenGLScenePtr &theScene = window3D[windowID]->get3DSceneAndLock();

	opengl::CGridPlaneXYPtr plane = opengl::CGridPlaneXY::Create();
	plane->setPlaneLimits(xmin,xmax,ymin,ymax);
	plane->setGridFrequency(frequency);
	plane->setPlaneZcoord(z);
	theScene->insert(plane);

	unlockWindow3D(windowID);
}


void VisualizationHelper::showBaseCoordinateSystem(uint32_t windowId, double size) {
	COpenGLScenePtr &theScene = window3D[windowId]->get3DSceneAndLock();

	// Add robot coordinate system
	opengl::CSimpleLinePtr xAxis = opengl::CSimpleLine::Create();
	xAxis->setName("coord_x");
	xAxis->setLineCoords(0, 0, 0, size, 0, 0);
	xAxis->setLineWidth(3.0);
	xAxis->setColor(mrpt::utils::TColorf(1, 0, 0));
	theScene->insert(xAxis);

	opengl::CSimpleLinePtr yAxis = opengl::CSimpleLine::Create();
	yAxis->setName("coord_y");
	yAxis->setLineCoords(0, 0, 0, 0, size, 0);
	yAxis->setLineWidth(3.0);
	yAxis->setColor(mrpt::utils::TColorf(0, 1, 0));
	theScene->insert(yAxis);

	opengl::CSimpleLinePtr zAxis = opengl::CSimpleLine::Create();
	zAxis->setName("coord_z");
	zAxis->setLineCoords(0, 0, 0, 0, 0, size);
	zAxis->setLineWidth(3.0);
	zAxis->setColor(mrpt::utils::TColorf(0, 0, 1));
	theScene->insert(zAxis);

	unlockWindow3D(windowId);
}


void VisualizationHelper::showCoordinateSystem(uint32_t windowId, CPose3D& pose, double size) {

	COpenGLScenePtr &theScene = window3D[windowId]->get3DSceneAndLock();


	opengl::CSetOfObjectsPtr corner = opengl::stock_objects::CornerXYZSimple(size,2.0);
	corner->setPose(pose);
	theScene->insert(corner);

	unlockWindow3D(windowId);
}





void VisualizationHelper::showCoordinateSystem(uint32_t windowId, const arma::mat& m, double size) {

//	m.print("m");

	arma::vec origin(4);
	origin(0) = origin(1) = origin(2) = 0;
	origin(3) = 1;

	arma::vec point(4);
	point(0) = point(1) = point(2) = 0;
	point(3) = 1;

	arma::vec o, p;

	COpenGLScenePtr &theScene = window3D[windowId]->get3DSceneAndLock();

	// Add robot coordinate system
	opengl::CSimpleLinePtr xAxis = opengl::CSimpleLine::Create();
	xAxis->setName("coord_x");
	o = m * origin;
	p = point;
	p(0) = size;
	p = m * p;
	xAxis->setLineCoords(o(0), o(1), o(2), p(0), p(1), p(2));
	xAxis->setLineWidth(3.0);
	xAxis->setColor(mrpt::utils::TColorf(1, 0, 0));
	theScene->insert(xAxis);

	opengl::CSimpleLinePtr yAxis = opengl::CSimpleLine::Create();
	yAxis->setName("coord_y");
	o = m * origin;
	p = point;
	p(1) = size;
	p = m * p;
	yAxis->setLineCoords(o(0), o(1), o(2), p(0), p(1), p(2));
	yAxis->setLineWidth(3.0);
	yAxis->setColor(mrpt::utils::TColorf(0, 1, 0));
	theScene->insert(yAxis);

	opengl::CSimpleLinePtr zAxis = opengl::CSimpleLine::Create();
	zAxis->setName("coord_z");
	o = m * origin;
	p = point;
	p(2) = size;
	p = m * p;
	zAxis->setLineCoords(o(0), o(1), o(2), p(0), p(1), p(2));
	zAxis->setLineWidth(3.0);
	zAxis->setColor(mrpt::utils::TColorf(0, 0, 1));
	theScene->insert(zAxis);

	unlockWindow3D(windowId);
}

VisualizationHelper::~VisualizationHelper() {

	for (size_t i = 0; i < window2D.size(); ++i) {
		delete window2D[i];
	}

	for (size_t i = 0; i < window3D.size(); ++i) {
		delete window3D[i];
	}
}

void VisualizationHelper::saveMatrixAsImage(const std::string& fileName, const CMatrixFloat& matrix) {
	CImage img;
	img.setFromMatrix(matrix);
	img.saveToFile(fileName);
}

//////////////////////////////////////////
//
//			Color
//
//////////////////////////////////////////
bool VisualizationHelper::Color::randomInitDone = false;

VisualizationHelper::Color::Color(int color) {
	switch (color) {
	case Color::Blue:
		red = 0;
		green = 0;
		blue = 255;
		break;

	case Color::DeepSkyBlue:
		red = 0;
		green = 191;
		blue = 255;
		break;

	case Color::DeepSkyBlue4:
		red = 0;
		green = 104;
		blue = 139;
		break;

	case Color::SlateBlue1:
		red = 131;
		green = 111;
		blue = 255;
		break;

	case Color::Cyan:
		red = 0;
		green = 255;
		blue = 255;
		break;

	case Color::Red:
		red = 255;
		green = 0;
		blue = 0;
		break;

	case Color::Green:
		red = 0;
		green = 255;
		blue = 0;
		break;

	case Color::Green4:
		red = 0;
		green = 139;
		blue = 0;
		break;

	case Color::DarkOrange:
		red = 255;
		green = 140;
		blue = 0;
		break;

	case Color::DeepPink:
		red = 255;
		green = 20;
		blue = 147;
		break;
	}
}

VisualizationHelper::Color::Colors VisualizationHelper::Color::randomColor() {
	if (!VisualizationHelper::Color::randomInitDone) {
		srand(time(NULL));
		VisualizationHelper::Color::randomInitDone = true;
	}
	return (VisualizationHelper::Color::Colors)(rand() % 7 + 3);

}
