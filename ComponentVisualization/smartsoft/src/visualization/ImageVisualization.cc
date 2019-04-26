#include "ImageVisualization.hh"

#include <cv.h>
#include "OpenCVHelpers/OpenCVHelpers.hh"

#include <cstring>


ImageVisualization::ImageVisualization(){


//	cv::namedWindow(image_window_label, CV_WINDOW_NORMAL);
//
//	imageWindow3D = new CDisplayWindow3D();
//	imageWindow3D->setWindowTitle("SmartVisualization Image");
//	imageWindow3D->resize(800,600);

//	m_win_range = mrpt::gui::CDisplayWindow::Create("Preview RANGE");
//	m_win_range->setPos(5,5);
//
//    m_win_int = mrpt::gui::CDisplayWindow::Create("Preview INTENSITY");
//    m_win_int->setPos(300,5);



}

ImageVisualization::ImageVisualization(std::string window_name){
	m_image_window = mrpt::gui::CDisplayWindow::Create(window_name);
	m_image_window->setPos(5,5);
}

ImageVisualization::~ImageVisualization() {


//	opengl::COpenGLScenePtr &ptrScene = imageWindow3D->get3DSceneAndLock();
//	{
//		mrpt::opengl::COpenGLViewportPtr view = ptrScene->getViewport("main");
//		view->setNormalMode();
//	}
//	imageWindow3D->unlockAccess3DScene();
//	imageWindow3D->forceRepaint();
//	delete(imageWindow3D);

}


IplImage* convertDataArrayToIplImage(DomainVision::CommVideoImage &query_image, CvSize size)
{
	IplImage* ipl_image = NULL;

	if (query_image.get_format() == DomainVision::FormatType::UYVY || query_image.get_format() == DomainVision::FormatType::RGB24)
	{
		unsigned char* arr_image = new unsigned char[query_image.get_size_as_rgb24()];
		query_image.get_as_rgb24(arr_image);

		ipl_image = OpenCVHelpers::copyRGBToIplImage(arr_image, query_image.get_height(), query_image.get_width());
		delete[] arr_image;

	} else if (query_image.get_format() == DomainVision::FormatType::GREY)
	{
		CvMat mat;
		cvInitMatHeader(&mat, size.height, size.width, CV_8UC1, const_cast<unsigned char *> (query_image.get_data()));
		ipl_image = cvCreateImage(size, IPL_DEPTH_8U, 1);

		// copy matrix data into image
		cvCopy(&mat, ipl_image);

	} else if (query_image.get_format() == DomainVision::FormatType::YUV422)
	{
		unsigned char* arr_image = new unsigned char[query_image.get_size_as_rgb24()];
		query_image.get_as_rgb24(arr_image);

		ipl_image = OpenCVHelpers::copyRGBToIplImage(arr_image, query_image.get_height(), query_image.get_width());
		delete[] arr_image;

	}
	else
	{
		std::cout << "Image Format: " << query_image.get_format() << " not supported!" << std::endl;
	}

	return ipl_image;


}

void ImageVisualization::displayImage(DomainVision::CommVideoImage& image) {

	IplImage* currentImage = NULL;

	currentImage = convertDataArrayToIplImage(image, cvSize(image.get_width(), image.get_height()));

	if (currentImage != NULL){
		{
			mrpt::utils::CImage img(currentImage);
			m_image_window->showImage(img);

			// show dimensions in the title
			std::stringstream str_dimension;
			str_dimension << "RGB Image : "<< image.get_width()<<" x "<<image.get_height();
			m_image_window->setWindowTitle(str_dimension.str());

		}

	}

}

void ImageVisualization::displayDepthImage(DomainVision::CommDepthImage& image) {

	IplImage* currentImage = NULL;
	mrpt::utils::CImage depthImage(image.getWidth(), image.getHeight());

	DomainVision::DepthFormatType depth_format = image.getFormat();

	if(depth_format==DomainVision::DepthFormatType::UINT16)
	{
		const uint16_t* depth_data_uint16;
		depth_data_uint16 = image.get_distances_uint16();
		for (uint32_t i = 0; i < image.getHeight(); i++)
		{
			for (uint32_t j = 0; j < image.getWidth(); j++)
			{
				const uint16_t pixel_uint16 = *(depth_data_uint16 + i * image.getWidth() + j);
				const float pixel = pixel_uint16/1000.0;

				uint8_t r = pixel / 8* 255 ;
				uint8_t g = pixel / 8* 255 ;
				uint8_t b = pixel / 8* 255 ;

				mrpt::utils::TColor color(r, g, b);
				depthImage.setPixel(j, i, color);
			}
		}


	}else if (depth_format==DomainVision::DepthFormatType::FLOAT)
	{
		const float* depth_data_float;
		depth_data_float = image.get_distances_float();
		for (uint32_t i = 0; i < image.getHeight(); i++)
		{
			for (uint32_t j = 0; j < image.getWidth(); j++)
			{

				const float* pixel = (depth_data_float + i * image.getWidth() + j);

				uint8_t r = pixel[0] / 8* 255 ;
				uint8_t g = pixel[0] / 8* 255 ;
				uint8_t b = pixel[0] / 8* 255 ;

				mrpt::utils::TColor color(r, g, b);
				depthImage.setPixel(j, i, color);
			}
		}

	}else
	{
		std::cout << " Only Uint16, float formats supported." <<std::endl;
	}

	std::stringstream str_dimension;
	str_dimension << "Depth Image : "<< image.getWidth()<<" x "<<image.getHeight();

	m_image_window->showImage(depthImage);
	m_image_window->setWindowTitle(str_dimension.str());

}
void ImageVisualization::clear() {

}
