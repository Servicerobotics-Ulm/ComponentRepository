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

#ifndef VISUALIZATIONHELPER_H_
#define VISUALIZATIONHELPER_H_

#include <mrpt/base.h>
#include <mrpt/gui.h>
#include <mrpt/opengl/include/mrpt/opengl.h>
#include <mrpt/base/include/mrpt/base.h>
#include <mrpt/base/include/mrpt/poses/CPose3D.h>
#include <mrpt/opengl/include/mrpt/opengl/COpenGLScene.h>
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::poses;
using namespace mrpt::opengl;

#include <armadillo.hh>
#include <OpenCVHelpers/OpenCVHelpers.hh>

//#include "PointCloud.hh"

#include <vector>
#include <string>


class VisualizationHelper {

private:
	std::vector<CDisplayWindow*> window2D;
	std::vector<CDisplayWindow3D*> window3D;
	std::vector<opengl::CPointCloudColouredPtr> cloud3D;
	std::vector<opengl::CSetOfLinesPtr> lines3D;

public:
	VisualizationHelper(uint32_t numberOfImageWindows = 0, uint32_t numberOf3DWindows = 0);
	//VisualizationHelper(uint32_t numberOfImageWindows = 3, uint32_t numberOf3DWindows = 1);

	virtual ~VisualizationHelper();

	void setWindowTitle(uint32_t windowId, const std::string& title) {
		window2D[windowId]->setWindowTitle(title);
	}

	void setWindow3DTitle(uint32_t windowId, const std::string& title) {
		window3D[windowId]->setWindowTitle(title);
	}

	void setWindow3DSize(uint32_t windowId, uint32_t width, uint32_t height) {
		window3D[windowId]->resize(width, height);
	}

	void showImage(uint32_t windowId, const CImage& image) {
		window2D[windowId]->showImage(image);
	}

	void showImage(uint32_t windowId, IplImage* image) {
		if(image != NULL) {
			CImage tmp;
			tmp.loadFromIplImage(image);
			window2D[windowId]->showImage(tmp);
		} else {
			std::cout << "ERROR: IMAGE NULL in showImage()" << std::endl;
		}
	}

	//TODO no float image in smartsoft
	//void showImage(uint32_t windowId, const CImageFloat& image) {
	//	window2D[windowId]->showImage(image);
	//}

	void lockWindow3D(uint32_t windowId) {
		window3D[windowId]->get3DSceneAndLock();
	}

	void unlockWindow3D(uint32_t windowId) {
		window3D[windowId]->unlockAccess3DScene();
		window3D[windowId]->forceRepaint();
	}

	opengl::CPointCloudColouredPtr getCloud(uint32_t windowId) {
		return cloud3D[windowId];
	}

	//void showPointCloud(uint32_t windowId, PointCloud& cloud, float r=1, float g=0, float b=0);
	void showText(uint32_t windowId, std::string text, double x, double y, double z);

	void show3DBox(uint32_t windowId, TPoint3D& point1,TPoint3D& point2, CPose3D* pose = NULL);
	void show3DBox(uint32_t windowId, CPoint3D& point1,CPoint3D& point2, CPose3D* pose = NULL);

	void showArrow(uint32_t windowId, TPoint3D& point1,TPoint3D& point2);
	void showArrow(uint32_t windowId, float x1, float y1, float z1, float x2, float y2, float z2);
	void showLine(uint32_t windowId, float x1, float y1, float z1, float x2, float y2, float z2);

	void showXYPlane(uint32_t windowID, float xmin, float xmax, float ymin, float ymax, float z, float frequency);

	void showYZPlane(uint32_t windowID, float xmin, float xmax, float zmin, float zmax, float x, float frequency);

	void showXZPlane(uint32_t windowID, float xmin, float xmax, float zmin, float zmax, float y, float frequency);

	void showBaseCoordinateSystem(uint32_t windowId, double size = 0.2);

	void showCoordinateSystem(uint32_t windowId, const arma::mat& m, double size = 0.1);
	void showCoordinateSystem(uint32_t windowId, CPose3D& pose, double size = 0.1);

	opengl::CSetOfLinesPtr getLines(uint32_t windowId) {
		return lines3D[windowId];
	}

	uint32_t addWindow();

	uint32_t addWindow3D();

	static void saveMatrixAsImage(const std::string& fileName, const CMatrixFloat& matrix);

	void clearScene(uint32_t windowId);
	void saveScene(uint32_t windowId, string filename);

public:
	class Color {
	public:
		enum Colors {
			Red = 0,
			Green = 1,
			Blue = 2,
			DeepPink = 3,
			DeepSkyBlue = 4,
			Green4 = 5,
			DeepSkyBlue4 = 6,
			DarkOrange = 7,
			SlateBlue1 = 8,
			Cyan = 9
		};

	private:
		static bool randomInitDone;

		int red;
		int green;
		int blue;

	public:
		Color(int color);

		double getGlRed() {
			return (double) red / 255;
		}

		double getGlGreen() {
			return (double) green / 255;
		}

		double getGlBlue() {
			return (double) blue / 255;
		}

		/**
		 * Red, Green and Blue are not used as random color.
		 */
		static Color::Colors randomColor();

	};

};

#endif /* VISUALIZATIONHELPERS_H_ */
