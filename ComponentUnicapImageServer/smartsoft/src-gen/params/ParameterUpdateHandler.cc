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

#include "ComponentUnicapImageServer.hh"

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
		// parameter Image
		if(parameter.getBoolean("Image", "debug_info", commitState.Image.debug_info))
		{
			globalState.Image.debug_info = commitState.Image.debug_info;
		}
		if(parameter.getString("Image", "smart_format", commitState.Image.smart_format))
		{
			globalState.Image.smart_format = commitState.Image.smart_format;
		}
		if(parameter.getDouble("Image", "valid_time_in_sec", commitState.Image.valid_time_in_sec))
		{
			globalState.Image.valid_time_in_sec = commitState.Image.valid_time_in_sec;
		}
		// parameter IntrinsicParams
		if(parameter.getInteger("IntrinsicParams", "calib_height", commitState.IntrinsicParams.calib_height))
		{
			globalState.IntrinsicParams.calib_height = commitState.IntrinsicParams.calib_height;
		}
		if(parameter.getInteger("IntrinsicParams", "calib_width", commitState.IntrinsicParams.calib_width))
		{
			globalState.IntrinsicParams.calib_width = commitState.IntrinsicParams.calib_width;
		}
		if(parameter.getDouble("IntrinsicParams", "cx", commitState.IntrinsicParams.cx))
		{
			globalState.IntrinsicParams.cx = commitState.IntrinsicParams.cx;
		}
		if(parameter.getDouble("IntrinsicParams", "cy", commitState.IntrinsicParams.cy))
		{
			globalState.IntrinsicParams.cy = commitState.IntrinsicParams.cy;
		}
		if(parameter.getDoubleList("IntrinsicParams", "distortion_coeffs", commitState.IntrinsicParams.distortion_coeffs))
		{
			globalState.IntrinsicParams.distortion_coeffs = commitState.IntrinsicParams.distortion_coeffs;
		}
		if(parameter.getDouble("IntrinsicParams", "fx", commitState.IntrinsicParams.fx))
		{
			globalState.IntrinsicParams.fx = commitState.IntrinsicParams.fx;
		}
		if(parameter.getDouble("IntrinsicParams", "fy", commitState.IntrinsicParams.fy))
		{
			globalState.IntrinsicParams.fy = commitState.IntrinsicParams.fy;
		}
		// parameter base
		if(parameter.getDouble("base", "azimuth", commitState.base.azimuth))
		{
			globalState.base.azimuth = commitState.base.azimuth;
		}
		if(parameter.getDouble("base", "elevation", commitState.base.elevation))
		{
			globalState.base.elevation = commitState.base.elevation;
		}
		if(parameter.getBoolean("base", "on_base", commitState.base.on_base))
		{
			globalState.base.on_base = commitState.base.on_base;
		}
		if(parameter.getBoolean("base", "on_ptu", commitState.base.on_ptu))
		{
			globalState.base.on_ptu = commitState.base.on_ptu;
		}
		if(parameter.getDouble("base", "roll", commitState.base.roll))
		{
			globalState.base.roll = commitState.base.roll;
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
		// parameter hardware
		if(parameter.getString("hardware", "camera_type", commitState.hardware.camera_type))
		{
			globalState.hardware.camera_type = commitState.hardware.camera_type;
		}
		if(parameter.getBoolean("hardware", "debug_info", commitState.hardware.debug_info))
		{
			globalState.hardware.debug_info = commitState.hardware.debug_info;
		}
		if(parameter.getString("hardware", "device", commitState.hardware.device))
		{
			globalState.hardware.device = commitState.hardware.device;
		}
		if(parameter.getString("hardware", "identifier", commitState.hardware.identifier))
		{
			globalState.hardware.identifier = commitState.hardware.identifier;
		}
		// parameter hardware_properties
		if(parameter.getDouble("hardware_properties", "auto_exposure", commitState.hardware_properties.auto_exposure))
		{
			globalState.hardware_properties.auto_exposure = commitState.hardware_properties.auto_exposure;
		}
		if(parameter.getBoolean("hardware_properties", "autoflag_shutter", commitState.hardware_properties.autoflag_shutter))
		{
			globalState.hardware_properties.autoflag_shutter = commitState.hardware_properties.autoflag_shutter;
		}
		if(parameter.getBoolean("hardware_properties", "autoflag_white_balance_mode", commitState.hardware_properties.autoflag_white_balance_mode))
		{
			globalState.hardware_properties.autoflag_white_balance_mode = commitState.hardware_properties.autoflag_white_balance_mode;
		}
		if(parameter.getDouble("hardware_properties", "brightness", commitState.hardware_properties.brightness))
		{
			globalState.hardware_properties.brightness = commitState.hardware_properties.brightness;
		}
		if(parameter.getString("hardware_properties", "format", commitState.hardware_properties.format))
		{
			globalState.hardware_properties.format = commitState.hardware_properties.format;
		}
		if(parameter.getDouble("hardware_properties", "framerate", commitState.hardware_properties.framerate))
		{
			globalState.hardware_properties.framerate = commitState.hardware_properties.framerate;
		}
		if(parameter.getDouble("hardware_properties", "gain", commitState.hardware_properties.gain))
		{
			globalState.hardware_properties.gain = commitState.hardware_properties.gain;
		}
		if(parameter.getDouble("hardware_properties", "gamma", commitState.hardware_properties.gamma))
		{
			globalState.hardware_properties.gamma = commitState.hardware_properties.gamma;
		}
		if(parameter.getInteger("hardware_properties", "height", commitState.hardware_properties.height))
		{
			globalState.hardware_properties.height = commitState.hardware_properties.height;
		}
		if(parameter.getDouble("hardware_properties", "hue", commitState.hardware_properties.hue))
		{
			globalState.hardware_properties.hue = commitState.hardware_properties.hue;
		}
		if(parameter.getDouble("hardware_properties", "saturation", commitState.hardware_properties.saturation))
		{
			globalState.hardware_properties.saturation = commitState.hardware_properties.saturation;
		}
		if(parameter.getDouble("hardware_properties", "sharpness", commitState.hardware_properties.sharpness))
		{
			globalState.hardware_properties.sharpness = commitState.hardware_properties.sharpness;
		}
		if(parameter.getDouble("hardware_properties", "shutter", commitState.hardware_properties.shutter))
		{
			globalState.hardware_properties.shutter = commitState.hardware_properties.shutter;
		}
		if(parameter.getInteger("hardware_properties", "trigger_mode", commitState.hardware_properties.trigger_mode))
		{
			globalState.hardware_properties.trigger_mode = commitState.hardware_properties.trigger_mode;
		}
		if(parameter.getInteger("hardware_properties", "trigger_polarity", commitState.hardware_properties.trigger_polarity))
		{
			globalState.hardware_properties.trigger_polarity = commitState.hardware_properties.trigger_polarity;
		}
		if(parameter.getDouble("hardware_properties", "white_balance_mode", commitState.hardware_properties.white_balance_mode))
		{
			globalState.hardware_properties.white_balance_mode = commitState.hardware_properties.white_balance_mode;
		}
		if(parameter.getDouble("hardware_properties", "white_balance_u", commitState.hardware_properties.white_balance_u))
		{
			globalState.hardware_properties.white_balance_u = commitState.hardware_properties.white_balance_u;
		}
		if(parameter.getDouble("hardware_properties", "white_balance_v", commitState.hardware_properties.white_balance_v))
		{
			globalState.hardware_properties.white_balance_v = commitState.hardware_properties.white_balance_v;
		}
		if(parameter.getInteger("hardware_properties", "width", commitState.hardware_properties.width))
		{
			globalState.hardware_properties.width = commitState.hardware_properties.width;
		}
		// parameter push_newest
		if(parameter.getBoolean("push_newest", "debug_info", commitState.push_newest.debug_info))
		{
			globalState.push_newest.debug_info = commitState.push_newest.debug_info;
		}
		// parameter push_timed
		if(parameter.getBoolean("push_timed", "debug_info", commitState.push_timed.debug_info))
		{
			globalState.push_timed.debug_info = commitState.push_timed.debug_info;
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
		if(parameter.getInteger("sensor_pose", "x", commitState.sensor_pose.x))
		{
			globalState.sensor_pose.x = commitState.sensor_pose.x;
		}
		if(parameter.getInteger("sensor_pose", "y", commitState.sensor_pose.y))
		{
			globalState.sensor_pose.y = commitState.sensor_pose.y;
		}
		if(parameter.getInteger("sensor_pose", "z", commitState.sensor_pose.z))
		{
			globalState.sensor_pose.z = commitState.sensor_pose.z;
		}
		// parameter settings
		if(parameter.getBoolean("settings", "debug_info", commitState.settings.debug_info))
		{
			globalState.settings.debug_info = commitState.settings.debug_info;
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
