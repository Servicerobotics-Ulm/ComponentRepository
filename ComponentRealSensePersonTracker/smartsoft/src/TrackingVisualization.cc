/*--------------------------------------------------------------------------

 Copyright (C) 2017

 Created on: Aug 28, 2017
 Author    : Nayabrasul Shaik (shaik@hs-ulm.de)

 ZAFH Servicerobotik Ulm
 University of Applied Sciences
 Prittwitzstr. 10
 D-89075 Ulm
 Germany

 Note: Visualization is inspired from ROS version of realsense person tracking

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

#include "TrackingVisualization.hh"

void TrackingVisualization::Init(bool showDepth) {

	m_showDepth=showDepth;

	cv::namedWindow(COLOR_WINDOW_NAME, cv::WINDOW_NORMAL);
	//cv::startWindowThread();
	if (m_showDepth)
	{
		cv::namedWindow(DEPTH_WINDOW_NAME, cv::WINDOW_NORMAL);
		//cv::startWindowThread();
	}

}
TrackingVisualization::TrackingVisualization() {
	m_showDepth=false;
	mSummaryTextBottom=0;
}
TrackingVisualization::~TrackingVisualization() {
	// TODO Auto-generated destructor stub
}

void TrackingVisualization::ShowFrames(cv::Mat rgb, cv::Mat depth)
{
  cv::imshow(COLOR_WINDOW_NAME, rgb);
  if (m_showDepth)
  {
	  cv::imshow(DEPTH_WINDOW_NAME, depth);
  }
  cv::waitKey(1);
}

Rectangle TrackingVisualization::setLabel(cv::Mat& image, const std::string& label, const cv::Point& origin, int thickness, double scale)
{
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  int baseline = 0;
  cv::Scalar SmartBLACK(0, 0, 0);
  cv::Scalar SmartWHITE(255, 255, 255);
  cv::Scalar SmartRED(0, 0, 255);
  cv::Scalar SmartBLUE(255, 0, 0);
  cv::Scalar SmartGREEN(0, 255, 0);
  cv::Scalar SmartYELLOW(0, 255, 255);

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);

  cv::Point topLeft = origin + cv::Point(-3, -3);
  cv::Point bottomRight = origin + cv::Point(text.width + 3, text.height + 3);

  cv::rectangle(image, topLeft, bottomRight, SmartWHITE, cv::FILLED);
  cv::putText(image, label, origin + cv::Point(0, text.height), fontface, scale, SmartBLACK, thickness, 8);

  return Rectangle(topLeft, bottomRight);
}

void TrackingVisualization::DrawLineAtSummaryReport(cv::Mat image, std::string line)
{
  Rectangle summaryTextRect = setLabel(image, line, cv::Point(0, mSummaryTextBottom), 2,  0.5);
  mSummaryTextBottom = summaryTextRect.bottomRight.y + 0.02 * image.rows;
}

void TrackingVisualization::DrawPersonSummaryReport(cv::Mat image, int id, cv::Point3f comWorld)
{
  std::stringstream summaryText;// summary text at at top left corner of image (center of mass, orientation etc.)

  //add center of mass (world coordinates)
  summaryText << id << ": " << std::fixed << std::setprecision(3) <<"(" << comWorld.x << "," << comWorld.y << "," <<comWorld.z << ")";

  DrawLineAtSummaryReport(image, summaryText.str());
}
void TrackingVisualization::DrawPerson(cv::Mat image, int id, cv::Rect boundingBox, cv::Point com, cv::Point3f comWorld, bool istracked)
{
	cv::Scalar SmartRED(0, 0, 255);
	cv::Scalar SmartGREEN(0, 255, 0);


	// center of mass point
	cv::Point centerMass = com;
	cv::circle(image, centerMass, 6, SmartRED, -1, 8, 0);


	cv::Point pt1(boundingBox.x, boundingBox.y);
	//    cv::Point pt2(boundingBox.x + boundingBox.width, boundingBox.y + boundingBox.height);

	if(istracked)
	cv::rectangle(image, boundingBox, SmartGREEN, 2, cv::LINE_8);
	else
	cv::rectangle(image, boundingBox, SmartRED, 2, cv::LINE_8);

	// person ID at top left corner of person rectangle
	int personId = id;
	std::string idText = "Person Id: " + std::to_string(personId);
	Rectangle personIdTextRect = setLabel(image, idText, pt1 + cv::Point(5, 5), 1, 0.4);

	DrawPersonSummaryReport(image, id, comWorld);


}
void TrackingVisualization::Reset()
{
	mSummaryTextBottom=0;
}


