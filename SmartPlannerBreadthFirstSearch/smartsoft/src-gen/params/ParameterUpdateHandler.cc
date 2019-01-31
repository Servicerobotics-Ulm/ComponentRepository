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

#include "SmartPlannerBreadthFirstSearch.hh"

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
	else if (tag == "COMMNAVIGATIONOBJECTS.PLANNERPARAMS.DELETEGOAL")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommNavigationObjects_PlannerParams_DELETEGOALCore(
			);
		}
	}
	else if (tag == "COMMNAVIGATIONOBJECTS.PLANNERPARAMS.ID")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK); // TODO: this should be decided according to validation checks defined in the model (not yet implemented)
		
		unsigned int temp_id = 0;
		if(request.getInteger("1", temp_id) == 0) {
			commitState.CommNavigationObjects.PlannerParams.ID.id = temp_id;
		} else {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
		}
		
	}
	else if (tag == "COMMNAVIGATIONOBJECTS.PLANNERPARAMS.SETDESTINATIONCIRCLE")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		int temp_x = 0.0;
		if(request.getDouble("1", temp_x) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
		}
		int temp_y = 0.0;
		if(request.getDouble("2", temp_y) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
		}
		unsigned int temp_r = 0;
		if(request.getInteger("3", temp_r) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommNavigationObjects_PlannerParams_SETDESTINATIONCIRCLECore(
			temp_x, 
			temp_y, 
			temp_r
			);
		}
	}
	else if (tag == "COMMNAVIGATIONOBJECTS.PLANNERPARAMS.SETDESTINATIONLINE")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		int temp_x1 = 0.0;
		if(request.getDouble("1", temp_x1) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
		}
		int temp_y1 = 0.0;
		if(request.getDouble("2", temp_y1) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
		}
		int temp_x2 = 0.0;
		if(request.getDouble("3", temp_x2) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
		}
		int temp_y2 = 0.0;
		if(request.getDouble("4", temp_y2) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommNavigationObjects_PlannerParams_SETDESTINATIONLINECore(
			temp_x1, 
			temp_y1, 
			temp_x2, 
			temp_y2
			);
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
		// parameter Settings
		if(parameter.getDouble("Settings", "no_path_event_timeout", commitState.Settings.no_path_event_timeout))
		{
			globalState.Settings.no_path_event_timeout = commitState.Settings.no_path_event_timeout;
		}
		
		//
		// load extended parameters (if any)
		//
		
		//
		// load instance parameters (if a parameter definition was instantiated in the model)
		//
		// parameter CommNavigationObjects.PlannerParams.ID
		if(parameter.getInteger("CommNavigationObjects.PlannerParams.ID", "id", commitState.CommNavigationObjects.PlannerParams.ID.id))
		{
			globalState.CommNavigationObjects.PlannerParams.ID.id = commitState.CommNavigationObjects.PlannerParams.ID.id;
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