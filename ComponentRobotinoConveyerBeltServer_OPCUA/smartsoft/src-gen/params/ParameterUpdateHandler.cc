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

#include "ComponentRobotinoConveyerBeltServer_OPCUA.hh"

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
	else if (tag == "COMMROBOTINOOBJECTS.ROBOTINOCONVEYERPARAMETER.SETSTATIONID")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		short temp_id = 0;
		if(request.getInteger("id", temp_id) == 0) {
			commitState.CommRobotinoObjects.RobotinoConveyerParameter.SetStationID.id = temp_id;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: id request: "<<request<<std::endl;
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
	else if (tag == "COMMROBOTINOOBJECTS.ROBOTINOCONVEYERPARAMETER.SETSTATIONID")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		short temp_id = 0;
		if(request.getInteger("1", temp_id) == 0) {
			commitState.CommRobotinoObjects.RobotinoConveyerParameter.SetStationID.id = temp_id;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: id request: "<<request<<std::endl;
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
		// parameter OPCUAstatic
		if(parameter.getString("OPCUAstatic", "object_name", commitState.OPCUAstatic.object_name))
		{
			globalState.OPCUAstatic.object_name = commitState.OPCUAstatic.object_name;
		}
		if(parameter.getStringList("OPCUAstatic", "server_address", commitState.OPCUAstatic.server_address))
		{
			globalState.OPCUAstatic.server_address = commitState.OPCUAstatic.server_address;
		}
		// parameter Robot
		if(parameter.getInteger("Robot", "FallingEdge_PW_Load_mSec", commitState.Robot.FallingEdge_PW_Load_mSec))
		{
			globalState.Robot.FallingEdge_PW_Load_mSec = commitState.Robot.FallingEdge_PW_Load_mSec;
		}
		if(parameter.getInteger("Robot", "FallingEdge_PW_Unload_mSec", commitState.Robot.FallingEdge_PW_Unload_mSec))
		{
			globalState.Robot.FallingEdge_PW_Unload_mSec = commitState.Robot.FallingEdge_PW_Unload_mSec;
		}
		if(parameter.getInteger("Robot", "LoadProcess_TimeOutSec", commitState.Robot.LoadProcess_TimeOutSec))
		{
			globalState.Robot.LoadProcess_TimeOutSec = commitState.Robot.LoadProcess_TimeOutSec;
		}
		if(parameter.getInteger("Robot", "NoOf_FallingEdge_Load", commitState.Robot.NoOf_FallingEdge_Load))
		{
			globalState.Robot.NoOf_FallingEdge_Load = commitState.Robot.NoOf_FallingEdge_Load;
		}
		if(parameter.getInteger("Robot", "NoOf_FallingEdge_Unload", commitState.Robot.NoOf_FallingEdge_Unload))
		{
			globalState.Robot.NoOf_FallingEdge_Unload = commitState.Robot.NoOf_FallingEdge_Unload;
		}
		if(parameter.getInteger("Robot", "UnloadProcess_TimeOutSec", commitState.Robot.UnloadProcess_TimeOutSec))
		{
			globalState.Robot.UnloadProcess_TimeOutSec = commitState.Robot.UnloadProcess_TimeOutSec;
		}
		if(parameter.getInteger("Robot", "ack_pressed_din", commitState.Robot.ack_pressed_din))
		{
			globalState.Robot.ack_pressed_din = commitState.Robot.ack_pressed_din;
		}
		if(parameter.getInteger("Robot", "belt_time_out_sec", commitState.Robot.belt_time_out_sec))
		{
			globalState.Robot.belt_time_out_sec = commitState.Robot.belt_time_out_sec;
		}
		if(parameter.getInteger("Robot", "box_present_din", commitState.Robot.box_present_din))
		{
			globalState.Robot.box_present_din = commitState.Robot.box_present_din;
		}
		if(parameter.getInteger("Robot", "dock_complete_dout", commitState.Robot.dock_complete_dout))
		{
			globalState.Robot.dock_complete_dout = commitState.Robot.dock_complete_dout;
		}
		if(parameter.getBoolean("Robot", "ignore_station_communication", commitState.Robot.ignore_station_communication))
		{
			globalState.Robot.ignore_station_communication = commitState.Robot.ignore_station_communication;
		}
		if(parameter.getInteger("Robot", "ignore_station_communication_unload_time_sec", commitState.Robot.ignore_station_communication_unload_time_sec))
		{
			globalState.Robot.ignore_station_communication_unload_time_sec = commitState.Robot.ignore_station_communication_unload_time_sec;
		}
		if(parameter.getDouble("Robot", "load_motor_direction", commitState.Robot.load_motor_direction))
		{
			globalState.Robot.load_motor_direction = commitState.Robot.load_motor_direction;
		}
		if(parameter.getInteger("Robot", "manual_load_dout", commitState.Robot.manual_load_dout))
		{
			globalState.Robot.manual_load_dout = commitState.Robot.manual_load_dout;
		}
		if(parameter.getInteger("Robot", "signal_error_dout", commitState.Robot.signal_error_dout))
		{
			globalState.Robot.signal_error_dout = commitState.Robot.signal_error_dout;
		}
		if(parameter.getInteger("Robot", "signal_loading_dout", commitState.Robot.signal_loading_dout))
		{
			globalState.Robot.signal_loading_dout = commitState.Robot.signal_loading_dout;
		}
		if(parameter.getInteger("Robot", "signal_unloading_dout", commitState.Robot.signal_unloading_dout))
		{
			globalState.Robot.signal_unloading_dout = commitState.Robot.signal_unloading_dout;
		}
		if(parameter.getInteger("Robot", "station_communication_delay_sec", commitState.Robot.station_communication_delay_sec))
		{
			globalState.Robot.station_communication_delay_sec = commitState.Robot.station_communication_delay_sec;
		}
		if(parameter.getInteger("Robot", "station_communication_delay_usec", commitState.Robot.station_communication_delay_usec))
		{
			globalState.Robot.station_communication_delay_usec = commitState.Robot.station_communication_delay_usec;
		}
		if(parameter.getInteger("Robot", "trans_read_din", commitState.Robot.trans_read_din))
		{
			globalState.Robot.trans_read_din = commitState.Robot.trans_read_din;
		}
		if(parameter.getDouble("Robot", "unload_motor_direction", commitState.Robot.unload_motor_direction))
		{
			globalState.Robot.unload_motor_direction = commitState.Robot.unload_motor_direction;
		}
		
		//
		// load extended parameters (if any)
		//
		
		//
		// load instance parameters (if a parameter definition was instantiated in the model)
		//
		// parameter CommRobotinoObjects.RobotinoConveyerParameter.SetStationID
		if(parameter.getInteger("CommRobotinoObjects.RobotinoConveyerParameter.SetStationID", "id", commitState.CommRobotinoObjects.RobotinoConveyerParameter.SetStationID.id))
		{
			globalState.CommRobotinoObjects.RobotinoConveyerParameter.SetStationID.id = commitState.CommRobotinoObjects.RobotinoConveyerParameter.SetStationID.id;
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