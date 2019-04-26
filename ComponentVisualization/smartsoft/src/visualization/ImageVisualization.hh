#ifndef IMAGEVISUALIZATION_H_
#define IMAGEVISUALIZATION_H_

#include "AbstractVisualization.hh"
#include <DomainVision/CommVideoImage.hh>
#include "DomainVision/CommDepthImage.hh"


//#define USE_OPENCV

#include <mrpt/gui/CDisplayWindow.h>
//#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>

class ImageVisualization {
private:

	//const std::string image_window_label = "Image Visualization";
	//CDisplayWindow3D *imageWindow3D;
	mrpt::gui::CDisplayWindowPtr  m_image_window;



public:
	ImageVisualization();
	ImageVisualization(std::string window_name);
	virtual ~ImageVisualization();

	void displayImage(DomainVision::CommVideoImage& image);
	void displayDepthImage(DomainVision::CommDepthImage& image);
	void clear();
	void convert_depth_to_rgb(DomainVision::CommDepthImage& in_image, IplImage* rgb_image);
};

#endif /* IMAGEVISUALIZATION_H_*/
