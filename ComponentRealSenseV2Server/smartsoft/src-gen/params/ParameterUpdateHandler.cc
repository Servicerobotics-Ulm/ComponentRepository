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
// Please do not modify this file. It will be re-generated
// running the code generator.
//--------------------------------------------------------------------------
#include "ParameterUpdateHandler.hh"

#include "ComponentRealSenseV2Server.hh"

SmartACE::CommParameterResponse ParamUpdateHandler::handleParameter(const SmartACE::CommParameterRequest& request)
{
	SmartACE::CommParameterResponse answer;
	
	if(request.getParameterDataMode() == SmartACE::ParameterDataMode::NAME){
		answer = handleParametersNamed(request);
	} else {
		answer = handleParametersSequence(request);
	}
	return answer;
}


SmartACE::CommParameterResponse ParamUpdateHandler::handleParametersNamed(const SmartACE::CommParameterRequest& request)
{
	SmartACE::CommParameterResponse answer;
	
	std::string tag = request.getTag();
	for (auto & c: tag) c = toupper(c);
	std::cout<<"PARAMETER: "<<tag<<std::endl;
	
	if (tag == "COMMIT")
	{
		answer.setResponse(globalState.handleCOMMIT(commitState));
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			globalStateLock.acquire();
			// change the content of the globalState, however change only the generated content
			// without affecting potential user member variables (which is more intuitive for the user)
			globalState.setContent(commitState);
			globalStateLock.release();
		} else {
			// the commit validation check returned != OK
			// the commit state is rejected and is not copied into the global state
		}
	}
	else
	{
		/////////////////////////////////////////////////////////////////////
		// default new
		std::cout<<"ERROR wrong Parameter!"<<std::endl;
		answer.setResponse(SmartACE::ParamResponseType::INVALID);
	}
	

	std::cout<<"[handleQuery] PARAMETER "<<tag<<" DONE\n\n";

	return answer;
}


SmartACE::CommParameterResponse ParamUpdateHandler::handleParametersSequence(const SmartACE::CommParameterRequest& request)
{
	SmartACE::CommParameterResponse answer;
	
	std::string tag = request.getTag();
	for (auto & c: tag) c = toupper(c);
	std::cout<<"PARAMETER: "<<tag<<std::endl;
	
	if (tag == "COMMIT")
	{
		answer.setResponse(globalState.handleCOMMIT(commitState));
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			globalStateLock.acquire();
			// change the content of the globalState, however change only the generated content
			// without affecting potential user member variables (which is more intuitive for the user)
			globalState.setContent(commitState);
			globalStateLock.release();
		} else {
			// the commit validation check returned != OK
			// the commit state is rejected and is not copied into the global state
		}
	}
	else
	{
		/////////////////////////////////////////////////////////////////////
		// default new
		std::cout<<"ERROR wrong Parameter!"<<std::endl;
		answer.setResponse(SmartACE::ParamResponseType::INVALID);
	}
	

	std::cout<<"[handleQuery] PARAMETER "<<tag<<" DONE\n\n";

	return answer;
}


ParameterStateStruct ParamUpdateHandler::getGlobalState() const{
	SmartACE::SmartGuard g(globalStateLock);
	return this->globalState;
}


void ParamUpdateHandler::loadParameter(SmartACE::SmartIniParameter &parameter)
{
	/*
	 Parameters can be specified via command line -filename=<filename>

	 With this parameter present:
	 - The component will look for the file in the current working directory,
	 a path relative to the current directory or any absolute path
	 - The component will use the default values if the file cannot be found

	 With this parameter absent:
	 - <Name of Component>.ini will be read from current working directory, if found there
	 - $SMART_ROOT/etc/<Name of Component>.ini will be read otherwise
	 - Default values will be used if neither found in working directory or /etc
	 */

	// load parameters
	try
	{
		// print all known parameters
		parameter.print();

		//
		// load internal parameters (if any)
		//
		// parameter Depth_config
		if(parameter.getInteger("Depth_config", "framerate", commitState.Depth_config.framerate))
		{
			globalState.Depth_config.framerate = commitState.Depth_config.framerate;
		}
		if(parameter.getInteger("Depth_config", "height", commitState.Depth_config.height))
		{
			globalState.Depth_config.height = commitState.Depth_config.height;
		}
		if(parameter.getInteger("Depth_config", "width", commitState.Depth_config.width))
		{
			globalState.Depth_config.width = commitState.Depth_config.width;
		}
		// parameter RGB_config
		if(parameter.getInteger("RGB_config", "framerate", commitState.RGB_config.framerate))
		{
			globalState.RGB_config.framerate = commitState.RGB_config.framerate;
		}
		if(parameter.getInteger("RGB_config", "height", commitState.RGB_config.height))
		{
			globalState.RGB_config.height = commitState.RGB_config.height;
		}
		if(parameter.getInteger("RGB_config", "width", commitState.RGB_config.width))
		{
			globalState.RGB_config.width = commitState.RGB_config.width;
		}
		// parameter base
		if(parameter.getDouble("base", "base_a", commitState.base.base_a))
		{
			globalState.base.base_a = commitState.base.base_a;
		}
		if(parameter.getBoolean("base", "on_base", commitState.base.on_base))
		{
			globalState.base.on_base = commitState.base.on_base;
		}
		if(parameter.getBoolean("base", "on_manipulator", commitState.base.on_manipulator))
		{
			globalState.base.on_manipulator = commitState.base.on_manipulator;
		}
		if(parameter.getBoolean("base", "on_ptu", commitState.base.on_ptu))
		{
			globalState.base.on_ptu = commitState.base.on_ptu;
		}
		if(parameter.getBoolean("base", "on_ur", commitState.base.on_ur))
		{
			globalState.base.on_ur = commitState.base.on_ur;
		}
		if(parameter.getDouble("base", "steer_a", commitState.base.steer_a))
		{
			globalState.base.steer_a = commitState.base.steer_a;
		}
		if(parameter.getInteger("base", "x", commitState.base.x))
		{
			globalState.base.x = commitState.base.x;
		}
		if(parameter.getInteger("base", "y", commitState.base.y))
		{
			globalState.base.y = commitState.base.y;
		}
		if(parameter.getInteger("base", "z", commitState.base.z))
		{
			globalState.base.z = commitState.base.z;
		}
		// parameter sensor_pose
		if(parameter.getDouble("sensor_pose", "azimuth", commitState.sensor_pose.azimuth))
		{
			globalState.sensor_pose.azimuth = commitState.sensor_pose.azimuth;
		}
		if(parameter.getDouble("sensor_pose", "elevation", commitState.sensor_pose.elevation))
		{
			globalState.sensor_pose.elevation = commitState.sensor_pose.elevation;
		}
		if(parameter.getDouble("sensor_pose", "roll", commitState.sensor_pose.roll))
		{
			globalState.sensor_pose.roll = commitState.sensor_pose.roll;
		}
		if(parameter.getDouble("sensor_pose", "x", commitState.sensor_pose.x))
		{
			globalState.sensor_pose.x = commitState.sensor_pose.x;
		}
		if(parameter.getDouble("sensor_pose", "y", commitState.sensor_pose.y))
		{
			globalState.sensor_pose.y = commitState.sensor_pose.y;
		}
		if(parameter.getDouble("sensor_pose", "z", commitState.sensor_pose.z))
		{
			globalState.sensor_pose.z = commitState.sensor_pose.z;
		}
		// parameter settings
		if(parameter.getBoolean("settings", "debug_info", commitState.settings.debug_info))
		{
			globalState.settings.debug_info = commitState.settings.debug_info;
		}
		if(parameter.getString("settings", "device_serial_number", commitState.settings.device_serial_number))
		{
			globalState.settings.device_serial_number = commitState.settings.device_serial_number;
		}
		if(parameter.getBoolean("settings", "post_processing", commitState.settings.post_processing))
		{
			globalState.settings.post_processing = commitState.settings.post_processing;
		}
		if(parameter.getBoolean("settings", "pushnewest_color_image", commitState.settings.pushnewest_color_image))
		{
			globalState.settings.pushnewest_color_image = commitState.settings.pushnewest_color_image;
		}
		if(parameter.getBoolean("settings", "pushnewest_depth_image", commitState.settings.pushnewest_depth_image))
		{
			globalState.settings.pushnewest_depth_image = commitState.settings.pushnewest_depth_image;
		}
		if(parameter.getBoolean("settings", "pushnewest_rgbd_image", commitState.settings.pushnewest_rgbd_image))
		{
			globalState.settings.pushnewest_rgbd_image = commitState.settings.pushnewest_rgbd_image;
		}
		if(parameter.getBoolean("settings", "undistort_image", commitState.settings.undistort_image))
		{
			globalState.settings.undistort_image = commitState.settings.undistort_image;
		}
		if(parameter.getDouble("settings", "valid_image_time", commitState.settings.valid_image_time))
		{
			globalState.settings.valid_image_time = commitState.settings.valid_image_time;
		}
		// parameter stereo
		if(parameter.getDouble("stereo", "baseline", commitState.stereo.baseline))
		{
			globalState.stereo.baseline = commitState.stereo.baseline;
		}
		
		//
		// load extended parameters (if any)
		//
		
		//
		// load instance parameters (if a parameter definition was instantiated in the model)
		//

	} catch (const SmartACE::IniParameterError & e)
	{
		std::cerr << "Exception from parameter handling: " << e << std::endl;
	} catch (const std::exception &ex)
	{
		std::cerr << "Uncaught std:: exception" << ex.what() << std::endl;
	} catch (...)
	{
		std::cerr << "Uncaught exception" << std::endl;
	}
}
