#ifndef RECTANG_H_
#define RECTANG_H_

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mrpt/poses.h>

struct Rectang{
public:
	int id;
	std::vector<cv::Point> points; // four corners
	double aspect_ratio;
	double area;
	float depth; 	// in meter
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud;
	mrpt::poses::CPose3D pose; // with respect to the robot


	/*
	 * Defines MRPT rectangle to check, whether the given point is within this shape
	 * this is necessary to get rid of surrounding points of diagonal boxes
	 */
	bool isPointInRect(int x, int y){
		if(mrpt_rect.size() != 4){
			std::vector<mrpt::math::TPoint2D> mrpt_points;
			mrpt_points.push_back(mrpt::math::TPoint2D(points[0].x, points[0].y));
			mrpt_points.push_back(mrpt::math::TPoint2D(points[1].x, points[1].y));
			mrpt_points.push_back(mrpt::math::TPoint2D(points[2].x, points[2].y));
			mrpt_points.push_back(mrpt::math::TPoint2D(points[3].x, points[3].y));
			mrpt_rect = mrpt::math::TPolygon2D(mrpt_points);
		}

		return mrpt_rect.contains(mrpt::math::TPoint2D(x,y));
	}


	/*
	 * Calculates the center point of the rectangle.
	 */
	cv::Point getRectCenter() {
		return cv::Point((points[0].x + points[2].x) / 2,(points[0].y + points[2].y) / 2);
	}


	/*
	 * Returns the min and max row (Y) and column (X) in the RGB image.
	 * This is necessary, when a diagonal rectangle was detected.
	 */
	void getRectMinMaxValues (int &minX, int &maxX, int &minY, int &maxY){
		maxX = std::max(points[0].x, points[1].x);
		maxX = std::max(points[2].x, maxX);
		maxX = std::max(points[3].x, maxX);

		minX = std::min(points[0].x, points[1].x);
		minX = std::min(points[2].x, minX);
		minX = std::min(points[3].x, minX);

		maxY = std::max(points[0].y, points[1].y);
		maxY = std::max(points[2].y, maxY);
		maxY = std::max(points[3].y, maxY);

		minY = std::min(points[0].y, points[1].y);
		minY = std::min(points[2].y, minY);
		minY = std::min(points[3].y, minY);
	}

	/*
	 * Sorts the points to: top left, bottom left, bottom right, top right
	 */
	void sortPoints(){
		std::sort(points.begin(),points.end(),[](const cv::Point2d pt1, const cv::Point2d pt2) { return (pt1.x < pt2.x);});
		std::sort(points.begin(),points.begin() + 2, [](const cv::Point2d pt1, const cv::Point2d pt2) { return (pt1.y < pt2.y);});
		std::sort(points.begin() + 2,points.end(), [](const cv::Point2d pt1, const cv::Point2d pt2) { return (pt1.y > pt2.y);});
	}



private:
	mrpt::math::TPolygon2D mrpt_rect;

};

#endif /* RECTANG_H_ */
