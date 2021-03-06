//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//  
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#ifndef _LOCALIZATIONTHREAD_HH
#define _LOCALIZATIONTHREAD_HH

#include "LocalizationThreadCore.hh"
#include "SmartSoft2RtabMapConverter.hh"

#include "DomainVision/CommRGBDImage.hh"
#include "MapBuilder.hh"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <qt5/QtWidgets/qapplication.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/odometry/OdometryF2M.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/CameraModel.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>

class LocalizationThread  : public LocalizationThreadCore
{
private:
	Smart::StatusCode status;
	DomainVision::CommRGBDImage scan;

	rtabmap::OdometryF2M Odomfm;

	QApplication *app;
	MapBuilder *mapBuilder;
public:
	LocalizationThread(SmartACE::SmartComponent *comp);
	virtual ~LocalizationThread();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();
	rtabmap::Rtabmap rtabmap;
	ofstream write_log_file;
	Odomtype OdomType_;
};

#endif
