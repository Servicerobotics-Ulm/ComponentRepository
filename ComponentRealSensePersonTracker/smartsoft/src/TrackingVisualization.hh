/*--------------------------------------------------------------------------

 Copyright (C) 2011 

 Created on: Aug 28, 2017
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

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iomanip>
#include <iostream>

struct Rectangle
{
  Rectangle(cv::Point topLeft, cv::Point bottomRight) : topLeft(topLeft), bottomRight(bottomRight)
  {
    width = bottomRight.x - topLeft.x;
    height = bottomRight.y - topLeft.y;
  }
  cv::Point topLeft;
  cv::Point bottomRight;
  int width;
  int height;
};



const char COLOR_WINDOW_NAME[] = "Person Tracking Sample";
const char DEPTH_WINDOW_NAME[] = "Person Tracking Sample Depth";

class TrackingVisualization {
	bool m_showDepth;
	int mSummaryTextBottom = 0;
public:
	TrackingVisualization();
	virtual ~TrackingVisualization();
	void Init(bool showDepth);
	void ShowFrames(cv::Mat rgb, cv::Mat depth);
	void ShowDepth(cv::Mat depth);
	Rectangle setLabel(cv::Mat& image, const std::string& label, const cv::Point& origin, int thickness = 2, double scale = 0.5);
	void DrawPerson(cv::Mat image, int id, cv::Rect boundingBox, cv::Point com, cv::Point3f comWorld,  bool istracked);
	void DrawLineAtSummaryReport(cv::Mat image, std::string line);
	void DrawPersonSummaryReport(cv::Mat image, int id, cv::Point3f comWorld);
	void Reset();
};

#endif /* VISUALIZATION_H_ */
