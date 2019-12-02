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

#include "ComponentKinectV1Server.hh"

SmartACE::CommParameterResponse ParamUpdateHandler::handleParameter(const SmartACE::CommParameterRequest& request)
{
	SmartACE::CommParameterResponse answer;

	std::string tag = request.getTag();
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
		// parameter base
		if(parameter.getDouble("base", "base_a", commitState.base.base_a))
		{
			globalState.base.base_a = commitState.base.base_a;
		}
		if(parameter.getBoolean("base", "on_base", commitState.base.on_base))
		{
			globalState.base.on_base = commitState.base.on_base;
		}
		if(parameter.getBoolean("base", "on_ptu", commitState.base.on_ptu))
		{
			globalState.base.on_ptu = commitState.base.on_ptu;
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
		// parameter hardware_properties
		if(parameter.getDouble("hardware_properties", "max_distance", commitState.hardware_properties.max_distance))
		{
			globalState.hardware_properties.max_distance = commitState.hardware_properties.max_distance;
		}
		if(parameter.getDouble("hardware_properties", "min_distance", commitState.hardware_properties.min_distance))
		{
			globalState.hardware_properties.min_distance = commitState.hardware_properties.min_distance;
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
		if(parameter.getInteger("settings", "depth_mode", commitState.settings.depth_mode))
		{
			globalState.settings.depth_mode = commitState.settings.depth_mode;
		}
		if(parameter.getBoolean("settings", "high_resolution", commitState.settings.high_resolution))
		{
			globalState.settings.high_resolution = commitState.settings.high_resolution;
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
		if(parameter.getInteger("settings", "rgb_mode", commitState.settings.rgb_mode))
		{
			globalState.settings.rgb_mode = commitState.settings.rgb_mode;
		}
		if(parameter.getBoolean("settings", "undistort_image", commitState.settings.undistort_image))
		{
			globalState.settings.undistort_image = commitState.settings.undistort_image;
		}
		if(parameter.getDouble("settings", "valid_image_time", commitState.settings.valid_image_time))
		{
			globalState.settings.valid_image_time = commitState.settings.valid_image_time;
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
