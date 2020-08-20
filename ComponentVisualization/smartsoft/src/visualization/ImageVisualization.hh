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
#ifdef WITH_MRPT_2_0_VERSION
	mrpt::gui::CDisplayWindow::Ptr  m_image_window;
#else
	mrpt::gui::CDisplayWindowPtr  m_image_window;
#endif



public:
	ImageVisualization();
	ImageVisualization(std::string window_name);
	virtual ~ImageVisualization();

	void displayImage(DomainVision::CommVideoImage& image);
	void displayDepthImage(DomainVision::CommDepthImage& image);
	void clear();
//#ifdef 0 //WITH_MRPT_2_0_VERSION
//	void convert_depth_to_rgb(DomainVision::CommDepthImage& in_image, cv::Nat rgb_image);
//#else
	void convert_depth_to_rgb(DomainVision::CommDepthImage& in_image, IplImage* rgb_image);
//#endif
};

#endif /* IMAGEVISUALIZATION_H_*/
