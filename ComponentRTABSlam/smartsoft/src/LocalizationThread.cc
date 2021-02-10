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
#include "LocalizationThread.hh"
#include "ComponentRTABSlam.hh"

#include "CommBasicObjects/CommBaseState.hh"

//#include <mrpt/poses/CPose3D.h>
//#include <mrpt/math.h>
#include <cassert>
#include <chrono>
#include <thread>

#include <iostream>
#include <fstream>

LocalizationThread::LocalizationThread(SmartACE::SmartComponent *comp) 
:	LocalizationThreadCore(comp)
{
	std::cout << "constructor LocalizationThread\n";
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kDebug);
}
LocalizationThread::~LocalizationThread() 
{
	std::cout << "destructor LocalizationThread\n";
}



int LocalizationThread::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	//Initialize rtabmap
	rtabmap::ParametersMap parameters;

	/*Only these 3 parameters differ between SLAM and localization mode*/
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpIncrementalDictionary(), "false"));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemSTMSize(), "1"));

	//To limit the distance of the extracted visual features used for visual odometry or loop closure detection
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxDepth(), "3"));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMinDepth(), "0.6"));

	std::string databasePath = UDirectory::homeDir() + "/"+ rtabmap::Parameters::getDefaultDatabaseName();



	//Open a logfile to store localization result from appearance based and laser based
	std::stringstream date_string;
	date_string << static_cast<unsigned long>(::time(0));
	std::string log_file_name(UDirectory::homeDir() + "/Smartlog_"+ date_string.str() + ".txt");

	//ofstream write_log_file;
	write_log_file.open(log_file_name);
	if(write_log_file.is_open())
	{
		std::cout << "Log file is opened at" <<log_file_name<<std::endl;
		int op_width = 15;

		// lx ly lz lr lp ly rx ry rz rr rp ry
		write_log_file <<std::setw(op_width)<< "base_x"<<std::setw(op_width)<< "base_y"<<std::setw(op_width)<< "base_z";
		write_log_file <<std::setw(op_width)<< "base_roll"<<std::setw(op_width)<< "base_pitch"<<std::setw(op_width)<< "base_yaw";

		write_log_file <<std::setw(op_width)<< "rtab_x"<<std::setw(op_width)<< "rtab_y"<<std::setw(op_width)<< "rtab_z";
		write_log_file <<std::setw(op_width)<< "rtab_roll"<<std::setw(op_width)<< "rtab_pitch"<<std::setw(op_width)<< "rtab_yaw" <<std::endl;

	}


	//rtabmap.init(parameters, databasePath);
		rtabmap.init(parameters,databasePath);

		int argc =0;
		char **argv = NULL;
		app = new QApplication(argc, argv);
		mapBuilder = new MapBuilder();

		mapBuilder->show();
		//QApplication::processEvents();


	std::cout << "using Rtabmap from Path ="<<databasePath<<std::endl;
	return 0;
}
int LocalizationThread::on_execute()
{

	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel
	
	try {

		status = COMP->stateSlave->acquire("nonneutral");

		if (status == Smart::SMART_OK)
		{

			// wait for scan (PushNewest)
			status = COMP->rgbd_client->getUpdateWait(scan);
			if (status != Smart::SMART_OK) {

				std::cout << "blocking wait  status " << Smart::StatusCodeConversion(status) << " not ok => retry ..." << std::endl;
			}
			else
			{
				//Process RealSense Data
				if(true!=scan.getIs_valid())
				{
					std::cout <<"RGBD Image data is INVALID" << std::endl;
				}else{

					DomainVision::CommVideoImage comm_rgb_image   = scan.getColor_image();
					DomainVision::CommDepthImage comm_depth_image = scan.getDepth_image();

					uint32_t rgb_frame_num = comm_rgb_image.getSeq_count();
					uint32_t depth_frame_num = comm_depth_image.getSeq_count();
					std::stringstream framenums;
					if(rgb_frame_num == depth_frame_num)
						framenums <<"RGB Frame Num: "<<rgb_frame_num <<", Depth Frame Num: "<<depth_frame_num;
					else
						framenums <<"?RGB Frame Num: "<<rgb_frame_num <<", ?Depth Frame Num: "<<depth_frame_num;


					cv::Mat rgb_mat(comm_rgb_image.get_height(), comm_rgb_image.get_width(), CV_8UC3);
					cv::Mat depth_mat(comm_depth_image.getHeight(), comm_depth_image.getWidth(), CV_16UC1);
					/*---------------------------------------------------------------------------------------------------------------------------------------------*/
					// copy rgb data into mat
					{
						const unsigned char* imageData = comm_rgb_image.get_data();
						for (int r = 0; r < rgb_mat.rows; r++)
						{
							for (int c = 0; c < rgb_mat.cols; c++)
							{
								const unsigned char* pixel = (imageData + r * 3* rgb_mat.cols + c * 3);

								rgb_mat.at<cv::Vec3b>(r,c)[0]=pixel[2];
								rgb_mat.at<cv::Vec3b>(r,c)[1]=pixel[1];
								rgb_mat.at<cv::Vec3b>(r,c)[2]=pixel[0];
							}
						}
					}
					/*---------------------------------------------------------------------------------------------------------------------------------------------*/

					// copy distance data into mat
					{
							const uint16_t* depth_data_uint16;
							const float* depth_data_float;
							DomainVision::DepthFormatType depth_format = comm_depth_image.getFormat();
							if(depth_format==DomainVision::DepthFormatType::UINT16)
							{
								//depth_data_uint16 = comm_depth_image.get_distances_data<const uint16_t*>();
								depth_data_uint16 = comm_depth_image.get_distances_uint16();

							} else if (depth_format==DomainVision::DepthFormatType::FLOAT)
							{
								depth_data_float = comm_depth_image.get_distances_float();

							}


							for (uint32_t depth_row = 0; depth_row < depth_mat.rows; ++depth_row) {//along y
								for (uint32_t depth_col = 0; depth_col < depth_mat.cols;++depth_col) {//along x-axis

									if(depth_format==DomainVision::DepthFormatType::UINT16)
									{
										const uint16_t pixel = *(depth_data_uint16 + depth_row * depth_mat.cols + depth_col);
										depth_mat.at<uint16_t>(depth_row, depth_col)=pixel;
									} else if (depth_format==DomainVision::DepthFormatType::FLOAT)
									{
										const float pixel = *(depth_data_float + depth_row * depth_mat.cols + depth_col);
										depth_mat.at<uint16_t>(depth_row, depth_col)=static_cast<uint16_t>(pixel*1000); // realsense requires uint16_t format with distance in mm
									}
								}
							}
						}
					/*---------------------------------------------------------------------------------------------------------------------------------------------*/

					assert(((depth_mat.cols = rgb_mat.cols) && (depth_mat.rows = rgb_mat.rows)));//, "Size of RGB and Depth image are not same");


					double current_time = 0;

					// localTransform is transform between Robot base and camera
					//rtabmap::Transform localTransform = transformFromSmartSoft(sensor_pose);
					rtabmap::Transform localTransform(0, 0, 1, 0,
							-1, 0, 0, 0,
							0,-1, 0, 0);

					rtabmap::CameraModel curr_cam_model(cameraModelFromSmartSoft(scan, localTransform));
					//rtabmap::SensorData data(rgb_mat, depth_mat, curr_cam_model, scan.get_rs_seq_count_rgb(), current_time);

					rtabmap::SensorData data(rgb_mat, depth_mat, curr_cam_model, 0, current_time);

					std::cout << "current rgbd frame num =" << rgb_frame_num<<std::endl;
					//std::cout << "SensorData  data.stamp() =" << data.stamp()<<std::endl;

					//smartodom_.processData(data, current_time);
					if(mapBuilder->isVisible())
					{


						rtabmap::Transform pose;
						rtabmap::OdometryInfo info;

						if(OdomType_ == Odomtype::VisualOdom)
						{
							pose = Odomfm.process(data, &info);
							std::cout << "VisualOdom: " <<pose.prettyPrint()<<std::endl;

						}
						else if(OdomType_==Odomtype::WheelOdom)
						{

							CommBasicObjects::CommBasePose Odom_pose = scan.getBase_state().get_base_raw_position();
							pose = transformFromSmartSoftBasePose(Odom_pose);
							//std::cout << "WheelOdom: " <<pose.prettyPrint()<<std::endl;
						}




						if(rtabmap.process(data, pose))
						{

							mapBuilder->processStatistics(rtabmap.getStatistics());

							//CommBasicObjects::CommBaseState base_state;
							//status = COMP->baseStatePushNewestClient->getUpdate(base_state);

							CommBasicObjects::CommBasePose base_pose = scan.getBase_state().get_base_position();
							double base_yaw = base_pose.get_base_azimuth(), base_pitch = base_pose.get_base_azimuth(), base_roll = base_pose.get_base_azimuth();
						    double base_x = base_pose.get_x() / 1000, base_y = base_pose.get_y() / 1000, base_z = base_pose.get_z() / 1000;
						    int op_width = 15;

						    // lx ly lz lr lp ly rx ry rz rr rp ry
						    write_log_file <<std::setw(op_width)<< base_x<<std::setw(op_width)<< base_y<<std::setw(op_width)<< base_z;
						    write_log_file <<std::setw(op_width)<< base_roll<<std::setw(op_width)<< base_pitch<<std::setw(op_width)<< base_yaw;



								if(rtabmap.getLoopClosureId() > 0)
								{

									printf("Localized Pose: %s\n", rtabmap.getLastLocalizationPose().prettyPrint().c_str());
									rtabmap::Transform rtab_pose = rtabmap.getLastLocalizationPose();


									float rtab_roll, rtab_pitch, rtab_yaw;
									float rtab_x, rtab_y, rtab_z;

									rtab_pose.getEulerAngles(rtab_roll, rtab_pitch, rtab_yaw);
									rtab_pose.getTranslation(rtab_x, rtab_y, rtab_z);

									write_log_file <<std::setw(op_width)<< rtab_x<<std::setw(op_width)<< rtab_y<<std::setw(op_width)<< rtab_z;
									write_log_file <<std::setw(op_width)<< rtab_roll<<std::setw(op_width)<< rtab_pitch<<std::setw(op_width)<< rtab_yaw <<std::endl;
								}
								else
								{
									printf("Not localized\n");
									double fail_=0.0;
									write_log_file <<std::setw(op_width)<< fail_<<std::setw(op_width)<< fail_<<std::setw(op_width)<< fail_;
									write_log_file <<std::setw(op_width)<< fail_<<std::setw(op_width)<< fail_<<std::setw(op_width)<< fail_<<std::endl;
								}



						}

						mapBuilder->processOdometry(data, pose, info);

						QApplication::processEvents();

						while(mapBuilder->isPaused() && mapBuilder->isVisible())
						{
							std::this_thread::sleep_for (std::chrono::milliseconds(10));
							QApplication::processEvents();
						}
					}

				}// Valid Data in the scan received


			}
		} // state
		status = COMP->stateSlave->release("nonneutral");



	}
	catch (std::exception &e)
	{
		std::cout << "exception: " << e.what() << std::endl;
	}



	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int LocalizationThread::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	write_log_file.close();

	delete mapBuilder;
	delete app;

	return 0;
}
