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

#include "ComponentWebotsPTUServer.hh"

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
	else if (tag == "DOMAINPTU.PTUPARAMETER.ACCELERATION_PAN")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_acc = 0.0;
		if(request.getDouble("acc", temp_acc) == 0) {
			commitState.DomainPTU.PTUParameter.ACCELERATION_PAN.acc = temp_acc;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: acc request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.ACCELERATION_TILT")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_acc = 0.0;
		if(request.getDouble("acc", temp_acc) == 0) {
			commitState.DomainPTU.PTUParameter.ACCELERATION_TILT.acc = temp_acc;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: acc request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.RESET")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		bool temp_reset = false;
		if(request.getBoolean("reset", temp_reset) == 0) {
			commitState.DomainPTU.PTUParameter.RESET.reset = temp_reset;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: reset request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.SENSOR_OFFSET")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_x = 0.0;
		if(request.getDouble("x", temp_x) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.x = temp_x;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: x request: "<<request<<std::endl;
		}
		double temp_y = 0.0;
		if(request.getDouble("y", temp_y) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.y = temp_y;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: y request: "<<request<<std::endl;
		}
		double temp_z = 0.0;
		if(request.getDouble("z", temp_z) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.z = temp_z;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: z request: "<<request<<std::endl;
		}
		double temp_azimuth = 0.0;
		if(request.getDouble("azimuth", temp_azimuth) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.azimuth = temp_azimuth;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: azimuth request: "<<request<<std::endl;
		}
		double temp_elevation = 0.0;
		if(request.getDouble("elevation", temp_elevation) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.elevation = temp_elevation;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: elevation request: "<<request<<std::endl;
		}
		double temp_roll = 0.0;
		if(request.getDouble("roll", temp_roll) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.roll = temp_roll;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: roll request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.SPEED_LIMIT_PAN")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_min = 0.0;
		if(request.getDouble("min", temp_min) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_LIMIT_PAN.min = temp_min;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: min request: "<<request<<std::endl;
		}
		double temp_max = 0.0;
		if(request.getDouble("max", temp_max) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_LIMIT_PAN.max = temp_max;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: max request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.SPEED_LIMIT_TILT")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_min = 0.0;
		if(request.getDouble("min", temp_min) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_LIMIT_TILT.min = temp_min;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: min request: "<<request<<std::endl;
		}
		double temp_max = 0.0;
		if(request.getDouble("max", temp_max) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_LIMIT_TILT.max = temp_max;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: max request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.SPEED_PAN")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_speed = 0.0;
		if(request.getDouble("speed", temp_speed) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_PAN.speed = temp_speed;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.SPEED_TILT")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_speed = 0.0;
		if(request.getDouble("speed", temp_speed) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_TILT.speed = temp_speed;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.START_UP_SPEED_PAN")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_speed = 0.0;
		if(request.getDouble("speed", temp_speed) == 0) {
			commitState.DomainPTU.PTUParameter.START_UP_SPEED_PAN.speed = temp_speed;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.START_UP_SPEED_TILT")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_speed = 0.0;
		if(request.getDouble("speed", temp_speed) == 0) {
			commitState.DomainPTU.PTUParameter.START_UP_SPEED_TILT.speed = temp_speed;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
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
	else if (tag == "DOMAINPTU.PTUPARAMETER.ACCELERATION_PAN")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_acc = 0.0;
		if(request.getDouble("1", temp_acc) == 0) {
			commitState.DomainPTU.PTUParameter.ACCELERATION_PAN.acc = temp_acc;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: acc request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.ACCELERATION_TILT")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_acc = 0.0;
		if(request.getDouble("1", temp_acc) == 0) {
			commitState.DomainPTU.PTUParameter.ACCELERATION_TILT.acc = temp_acc;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: acc request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.RESET")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		bool temp_reset = false;
		if(request.getBoolean("1", temp_reset) == 0) {
			commitState.DomainPTU.PTUParameter.RESET.reset = temp_reset;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: reset request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.SENSOR_OFFSET")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_x = 0.0;
		if(request.getDouble("1", temp_x) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.x = temp_x;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: x request: "<<request<<std::endl;
		}
		double temp_y = 0.0;
		if(request.getDouble("2", temp_y) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.y = temp_y;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: y request: "<<request<<std::endl;
		}
		double temp_z = 0.0;
		if(request.getDouble("3", temp_z) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.z = temp_z;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: z request: "<<request<<std::endl;
		}
		double temp_azimuth = 0.0;
		if(request.getDouble("4", temp_azimuth) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.azimuth = temp_azimuth;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: azimuth request: "<<request<<std::endl;
		}
		double temp_elevation = 0.0;
		if(request.getDouble("5", temp_elevation) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.elevation = temp_elevation;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: elevation request: "<<request<<std::endl;
		}
		double temp_roll = 0.0;
		if(request.getDouble("6", temp_roll) == 0) {
			commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.roll = temp_roll;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: roll request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.SPEED_LIMIT_PAN")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_min = 0.0;
		if(request.getDouble("1", temp_min) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_LIMIT_PAN.min = temp_min;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: min request: "<<request<<std::endl;
		}
		double temp_max = 0.0;
		if(request.getDouble("2", temp_max) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_LIMIT_PAN.max = temp_max;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: max request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.SPEED_LIMIT_TILT")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_min = 0.0;
		if(request.getDouble("1", temp_min) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_LIMIT_TILT.min = temp_min;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: min request: "<<request<<std::endl;
		}
		double temp_max = 0.0;
		if(request.getDouble("2", temp_max) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_LIMIT_TILT.max = temp_max;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: max request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.SPEED_PAN")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_speed = 0.0;
		if(request.getDouble("1", temp_speed) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_PAN.speed = temp_speed;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.SPEED_TILT")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_speed = 0.0;
		if(request.getDouble("1", temp_speed) == 0) {
			commitState.DomainPTU.PTUParameter.SPEED_TILT.speed = temp_speed;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.START_UP_SPEED_PAN")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_speed = 0.0;
		if(request.getDouble("1", temp_speed) == 0) {
			commitState.DomainPTU.PTUParameter.START_UP_SPEED_PAN.speed = temp_speed;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
	}
	else if (tag == "DOMAINPTU.PTUPARAMETER.START_UP_SPEED_TILT")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		double temp_speed = 0.0;
		if(request.getDouble("1", temp_speed) == 0) {
			commitState.DomainPTU.PTUParameter.START_UP_SPEED_TILT.speed = temp_speed;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
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
		// parameter Base
		if(parameter.getDouble("Base", "base_a", commitState.Base.base_a))
		{
			globalState.Base.base_a = commitState.Base.base_a;
		}
		if(parameter.getBoolean("Base", "on_base", commitState.Base.on_base))
		{
			globalState.Base.on_base = commitState.Base.on_base;
		}
		if(parameter.getDouble("Base", "steer_a", commitState.Base.steer_a))
		{
			globalState.Base.steer_a = commitState.Base.steer_a;
		}
		if(parameter.getDouble("Base", "turret_a", commitState.Base.turret_a))
		{
			globalState.Base.turret_a = commitState.Base.turret_a;
		}
		if(parameter.getInteger("Base", "x", commitState.Base.x))
		{
			globalState.Base.x = commitState.Base.x;
		}
		if(parameter.getInteger("Base", "y", commitState.Base.y))
		{
			globalState.Base.y = commitState.Base.y;
		}
		if(parameter.getInteger("Base", "z", commitState.Base.z))
		{
			globalState.Base.z = commitState.Base.z;
		}
		// parameter PTU
		if(parameter.getDouble("PTU", "azimuth", commitState.PTU.azimuth))
		{
			globalState.PTU.azimuth = commitState.PTU.azimuth;
		}
		if(parameter.getString("PTU", "device", commitState.PTU.device))
		{
			globalState.PTU.device = commitState.PTU.device;
		}
		if(parameter.getDouble("PTU", "elevation", commitState.PTU.elevation))
		{
			globalState.PTU.elevation = commitState.PTU.elevation;
		}
		if(parameter.getDouble("PTU", "roll", commitState.PTU.roll))
		{
			globalState.PTU.roll = commitState.PTU.roll;
		}
		if(parameter.getBoolean("PTU", "verbose", commitState.PTU.verbose))
		{
			globalState.PTU.verbose = commitState.PTU.verbose;
		}
		if(parameter.getInteger("PTU", "x", commitState.PTU.x))
		{
			globalState.PTU.x = commitState.PTU.x;
		}
		if(parameter.getInteger("PTU", "y", commitState.PTU.y))
		{
			globalState.PTU.y = commitState.PTU.y;
		}
		if(parameter.getInteger("PTU", "z", commitState.PTU.z))
		{
			globalState.PTU.z = commitState.PTU.z;
		}
		// parameter webots
		if(parameter.getString("webots", "robotName", commitState.webots.robotName))
		{
			globalState.webots.robotName = commitState.webots.robotName;
		}
		
		//
		// load extended parameters (if any)
		//
		
		//
		// load instance parameters (if a parameter definition was instantiated in the model)
		//
		// parameter DomainPTU.PTUParameter.ACCELERATION_PAN
		if(parameter.getDouble("DomainPTU.PTUParameter.ACCELERATION_PAN", "acc", commitState.DomainPTU.PTUParameter.ACCELERATION_PAN.acc))
		{
			globalState.DomainPTU.PTUParameter.ACCELERATION_PAN.acc = commitState.DomainPTU.PTUParameter.ACCELERATION_PAN.acc;
		}
		// parameter DomainPTU.PTUParameter.ACCELERATION_TILT
		if(parameter.getDouble("DomainPTU.PTUParameter.ACCELERATION_TILT", "acc", commitState.DomainPTU.PTUParameter.ACCELERATION_TILT.acc))
		{
			globalState.DomainPTU.PTUParameter.ACCELERATION_TILT.acc = commitState.DomainPTU.PTUParameter.ACCELERATION_TILT.acc;
		}
		// parameter DomainPTU.PTUParameter.RESET
		if(parameter.getBoolean("DomainPTU.PTUParameter.RESET", "reset", commitState.DomainPTU.PTUParameter.RESET.reset))
		{
			globalState.DomainPTU.PTUParameter.RESET.reset = commitState.DomainPTU.PTUParameter.RESET.reset;
		}
		// parameter DomainPTU.PTUParameter.SENSOR_OFFSET
		if(parameter.getDouble("DomainPTU.PTUParameter.SENSOR_OFFSET", "azimuth", commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.azimuth))
		{
			globalState.DomainPTU.PTUParameter.SENSOR_OFFSET.azimuth = commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.azimuth;
		}
		if(parameter.getDouble("DomainPTU.PTUParameter.SENSOR_OFFSET", "elevation", commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.elevation))
		{
			globalState.DomainPTU.PTUParameter.SENSOR_OFFSET.elevation = commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.elevation;
		}
		if(parameter.getDouble("DomainPTU.PTUParameter.SENSOR_OFFSET", "roll", commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.roll))
		{
			globalState.DomainPTU.PTUParameter.SENSOR_OFFSET.roll = commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.roll;
		}
		if(parameter.getDouble("DomainPTU.PTUParameter.SENSOR_OFFSET", "x", commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.x))
		{
			globalState.DomainPTU.PTUParameter.SENSOR_OFFSET.x = commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.x;
		}
		if(parameter.getDouble("DomainPTU.PTUParameter.SENSOR_OFFSET", "y", commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.y))
		{
			globalState.DomainPTU.PTUParameter.SENSOR_OFFSET.y = commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.y;
		}
		if(parameter.getDouble("DomainPTU.PTUParameter.SENSOR_OFFSET", "z", commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.z))
		{
			globalState.DomainPTU.PTUParameter.SENSOR_OFFSET.z = commitState.DomainPTU.PTUParameter.SENSOR_OFFSET.z;
		}
		// parameter DomainPTU.PTUParameter.SPEED_LIMIT_PAN
		if(parameter.getDouble("DomainPTU.PTUParameter.SPEED_LIMIT_PAN", "max", commitState.DomainPTU.PTUParameter.SPEED_LIMIT_PAN.max))
		{
			globalState.DomainPTU.PTUParameter.SPEED_LIMIT_PAN.max = commitState.DomainPTU.PTUParameter.SPEED_LIMIT_PAN.max;
		}
		if(parameter.getDouble("DomainPTU.PTUParameter.SPEED_LIMIT_PAN", "min", commitState.DomainPTU.PTUParameter.SPEED_LIMIT_PAN.min))
		{
			globalState.DomainPTU.PTUParameter.SPEED_LIMIT_PAN.min = commitState.DomainPTU.PTUParameter.SPEED_LIMIT_PAN.min;
		}
		// parameter DomainPTU.PTUParameter.SPEED_LIMIT_TILT
		if(parameter.getDouble("DomainPTU.PTUParameter.SPEED_LIMIT_TILT", "max", commitState.DomainPTU.PTUParameter.SPEED_LIMIT_TILT.max))
		{
			globalState.DomainPTU.PTUParameter.SPEED_LIMIT_TILT.max = commitState.DomainPTU.PTUParameter.SPEED_LIMIT_TILT.max;
		}
		if(parameter.getDouble("DomainPTU.PTUParameter.SPEED_LIMIT_TILT", "min", commitState.DomainPTU.PTUParameter.SPEED_LIMIT_TILT.min))
		{
			globalState.DomainPTU.PTUParameter.SPEED_LIMIT_TILT.min = commitState.DomainPTU.PTUParameter.SPEED_LIMIT_TILT.min;
		}
		// parameter DomainPTU.PTUParameter.SPEED_PAN
		if(parameter.getDouble("DomainPTU.PTUParameter.SPEED_PAN", "speed", commitState.DomainPTU.PTUParameter.SPEED_PAN.speed))
		{
			globalState.DomainPTU.PTUParameter.SPEED_PAN.speed = commitState.DomainPTU.PTUParameter.SPEED_PAN.speed;
		}
		// parameter DomainPTU.PTUParameter.SPEED_TILT
		if(parameter.getDouble("DomainPTU.PTUParameter.SPEED_TILT", "speed", commitState.DomainPTU.PTUParameter.SPEED_TILT.speed))
		{
			globalState.DomainPTU.PTUParameter.SPEED_TILT.speed = commitState.DomainPTU.PTUParameter.SPEED_TILT.speed;
		}
		// parameter DomainPTU.PTUParameter.START_UP_SPEED_PAN
		if(parameter.getDouble("DomainPTU.PTUParameter.START_UP_SPEED_PAN", "speed", commitState.DomainPTU.PTUParameter.START_UP_SPEED_PAN.speed))
		{
			globalState.DomainPTU.PTUParameter.START_UP_SPEED_PAN.speed = commitState.DomainPTU.PTUParameter.START_UP_SPEED_PAN.speed;
		}
		// parameter DomainPTU.PTUParameter.START_UP_SPEED_TILT
		if(parameter.getDouble("DomainPTU.PTUParameter.START_UP_SPEED_TILT", "speed", commitState.DomainPTU.PTUParameter.START_UP_SPEED_TILT.speed))
		{
			globalState.DomainPTU.PTUParameter.START_UP_SPEED_TILT.speed = commitState.DomainPTU.PTUParameter.START_UP_SPEED_TILT.speed;
		}

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