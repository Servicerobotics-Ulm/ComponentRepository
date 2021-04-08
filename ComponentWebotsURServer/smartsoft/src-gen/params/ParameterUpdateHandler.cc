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

#include "ComponentWebotsURServer.hh"

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
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.CLEAR_PCS")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_CLEAR_PCSCore(
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.LOAD_PROGRAM")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		std::string temp_name = "";
		if(request.getString("name", temp_name) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: name request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_LOAD_PROGRAMCore(
			temp_name
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.MOVE_CIRCULAR")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		double temp_viaX = 0.0;
		if(request.getDouble("viaX", temp_viaX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaX request: "<<request<<std::endl;
		}
		double temp_viaY = 0.0;
		if(request.getDouble("viaY", temp_viaY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaY request: "<<request<<std::endl;
		}
		double temp_viaZ = 0.0;
		if(request.getDouble("viaZ", temp_viaZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaZ request: "<<request<<std::endl;
		}
		double temp_viaRX = 0.0;
		if(request.getDouble("viaRX", temp_viaRX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaRX request: "<<request<<std::endl;
		}
		double temp_viaRY = 0.0;
		if(request.getDouble("viaRY", temp_viaRY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaRY request: "<<request<<std::endl;
		}
		double temp_viaRZ = 0.0;
		if(request.getDouble("viaRZ", temp_viaRZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaRZ request: "<<request<<std::endl;
		}
		double temp_goalX = 0.0;
		if(request.getDouble("goalX", temp_goalX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalX request: "<<request<<std::endl;
		}
		double temp_goalY = 0.0;
		if(request.getDouble("goalY", temp_goalY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalY request: "<<request<<std::endl;
		}
		double temp_goalZ = 0.0;
		if(request.getDouble("goalZ", temp_goalZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalZ request: "<<request<<std::endl;
		}
		double temp_goalRX = 0.0;
		if(request.getDouble("goalRX", temp_goalRX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRX request: "<<request<<std::endl;
		}
		double temp_goalRY = 0.0;
		if(request.getDouble("goalRY", temp_goalRY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRY request: "<<request<<std::endl;
		}
		double temp_goalRZ = 0.0;
		if(request.getDouble("goalRZ", temp_goalRZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRZ request: "<<request<<std::endl;
		}
		double temp_acc = 0.0;
		if(request.getDouble("acc", temp_acc) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: acc request: "<<request<<std::endl;
		}
		double temp_speed = 0.0;
		if(request.getDouble("speed", temp_speed) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULARCore(
			temp_viaX, 
			temp_viaY, 
			temp_viaZ, 
			temp_viaRX, 
			temp_viaRY, 
			temp_viaRZ, 
			temp_goalX, 
			temp_goalY, 
			temp_goalZ, 
			temp_goalRX, 
			temp_goalRY, 
			temp_goalRZ, 
			temp_acc, 
			temp_speed
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.MOVE_LINEAR")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		double temp_goalX = 0.0;
		if(request.getDouble("goalX", temp_goalX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalX request: "<<request<<std::endl;
		}
		double temp_goalY = 0.0;
		if(request.getDouble("goalY", temp_goalY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalY request: "<<request<<std::endl;
		}
		double temp_goalZ = 0.0;
		if(request.getDouble("goalZ", temp_goalZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalZ request: "<<request<<std::endl;
		}
		double temp_goalRX = 0.0;
		if(request.getDouble("goalRX", temp_goalRX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRX request: "<<request<<std::endl;
		}
		double temp_goalRY = 0.0;
		if(request.getDouble("goalRY", temp_goalRY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRY request: "<<request<<std::endl;
		}
		double temp_goalRZ = 0.0;
		if(request.getDouble("goalRZ", temp_goalRZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRZ request: "<<request<<std::endl;
		}
		double temp_acc = 0.0;
		if(request.getDouble("acc", temp_acc) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: acc request: "<<request<<std::endl;
		}
		double temp_speed = 0.0;
		if(request.getDouble("speed", temp_speed) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_MOVE_LINEARCore(
			temp_goalX, 
			temp_goalY, 
			temp_goalZ, 
			temp_goalRX, 
			temp_goalRY, 
			temp_goalRZ, 
			temp_acc, 
			temp_speed
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.MOVE_PATH")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		std::string temp_pathID = "";
		if(request.getString("pathID", temp_pathID) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: pathID request: "<<request<<std::endl;
		}
		bool temp_overwriteSpeed = false;
		if(request.getBoolean("overwriteSpeed", temp_overwriteSpeed) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: overwriteSpeed request: "<<request<<std::endl;
		}
		double temp_acc = 0.0;
		if(request.getDouble("acc", temp_acc) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: acc request: "<<request<<std::endl;
		}
		double temp_speed = 0.0;
		if(request.getDouble("speed", temp_speed) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_MOVE_PATHCore(
			temp_pathID, 
			temp_overwriteSpeed, 
			temp_acc, 
			temp_speed
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.SET_PCS")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		double temp_x = 0.0;
		if(request.getDouble("x", temp_x) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: x request: "<<request<<std::endl;
		}
		double temp_y = 0.0;
		if(request.getDouble("y", temp_y) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: y request: "<<request<<std::endl;
		}
		double temp_z = 0.0;
		if(request.getDouble("z", temp_z) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: z request: "<<request<<std::endl;
		}
		double temp_rX = 0.0;
		if(request.getDouble("rX", temp_rX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rX request: "<<request<<std::endl;
		}
		double temp_rY = 0.0;
		if(request.getDouble("rY", temp_rY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rY request: "<<request<<std::endl;
		}
		double temp_rZ = 0.0;
		if(request.getDouble("rZ", temp_rZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rZ request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_SET_PCSCore(
			temp_x, 
			temp_y, 
			temp_z, 
			temp_rX, 
			temp_rY, 
			temp_rZ
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.SET_TCP")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		double temp_x = 0.0;
		if(request.getDouble("x", temp_x) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: x request: "<<request<<std::endl;
		}
		double temp_y = 0.0;
		if(request.getDouble("y", temp_y) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: y request: "<<request<<std::endl;
		}
		double temp_z = 0.0;
		if(request.getDouble("z", temp_z) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: z request: "<<request<<std::endl;
		}
		double temp_rX = 0.0;
		if(request.getDouble("rX", temp_rX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rX request: "<<request<<std::endl;
		}
		double temp_rY = 0.0;
		if(request.getDouble("rY", temp_rY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rY request: "<<request<<std::endl;
		}
		double temp_rZ = 0.0;
		if(request.getDouble("rZ", temp_rZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rZ request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_SET_TCPCore(
			temp_x, 
			temp_y, 
			temp_z, 
			temp_rX, 
			temp_rY, 
			temp_rZ
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.START_PROGRAM")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_START_PROGRAMCore(
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
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.CLEAR_PCS")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_CLEAR_PCSCore(
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.LOAD_PROGRAM")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		std::string temp_name = "";
		if(request.getString("1", temp_name) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: name request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_LOAD_PROGRAMCore(
			temp_name
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.MOVE_CIRCULAR")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		double temp_viaX = 0.0;
		if(request.getDouble("1", temp_viaX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaX request: "<<request<<std::endl;
		}
		double temp_viaY = 0.0;
		if(request.getDouble("2", temp_viaY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaY request: "<<request<<std::endl;
		}
		double temp_viaZ = 0.0;
		if(request.getDouble("3", temp_viaZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaZ request: "<<request<<std::endl;
		}
		double temp_viaRX = 0.0;
		if(request.getDouble("4", temp_viaRX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaRX request: "<<request<<std::endl;
		}
		double temp_viaRY = 0.0;
		if(request.getDouble("5", temp_viaRY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaRY request: "<<request<<std::endl;
		}
		double temp_viaRZ = 0.0;
		if(request.getDouble("6", temp_viaRZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: viaRZ request: "<<request<<std::endl;
		}
		double temp_goalX = 0.0;
		if(request.getDouble("7", temp_goalX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalX request: "<<request<<std::endl;
		}
		double temp_goalY = 0.0;
		if(request.getDouble("8", temp_goalY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalY request: "<<request<<std::endl;
		}
		double temp_goalZ = 0.0;
		if(request.getDouble("9", temp_goalZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalZ request: "<<request<<std::endl;
		}
		double temp_goalRX = 0.0;
		if(request.getDouble("10", temp_goalRX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRX request: "<<request<<std::endl;
		}
		double temp_goalRY = 0.0;
		if(request.getDouble("11", temp_goalRY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRY request: "<<request<<std::endl;
		}
		double temp_goalRZ = 0.0;
		if(request.getDouble("12", temp_goalRZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRZ request: "<<request<<std::endl;
		}
		double temp_acc = 0.0;
		if(request.getDouble("13", temp_acc) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: acc request: "<<request<<std::endl;
		}
		double temp_speed = 0.0;
		if(request.getDouble("14", temp_speed) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_MOVE_CIRCULARCore(
			temp_viaX, 
			temp_viaY, 
			temp_viaZ, 
			temp_viaRX, 
			temp_viaRY, 
			temp_viaRZ, 
			temp_goalX, 
			temp_goalY, 
			temp_goalZ, 
			temp_goalRX, 
			temp_goalRY, 
			temp_goalRZ, 
			temp_acc, 
			temp_speed
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.MOVE_LINEAR")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		double temp_goalX = 0.0;
		if(request.getDouble("1", temp_goalX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalX request: "<<request<<std::endl;
		}
		double temp_goalY = 0.0;
		if(request.getDouble("2", temp_goalY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalY request: "<<request<<std::endl;
		}
		double temp_goalZ = 0.0;
		if(request.getDouble("3", temp_goalZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalZ request: "<<request<<std::endl;
		}
		double temp_goalRX = 0.0;
		if(request.getDouble("4", temp_goalRX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRX request: "<<request<<std::endl;
		}
		double temp_goalRY = 0.0;
		if(request.getDouble("5", temp_goalRY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRY request: "<<request<<std::endl;
		}
		double temp_goalRZ = 0.0;
		if(request.getDouble("6", temp_goalRZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: goalRZ request: "<<request<<std::endl;
		}
		double temp_acc = 0.0;
		if(request.getDouble("7", temp_acc) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: acc request: "<<request<<std::endl;
		}
		double temp_speed = 0.0;
		if(request.getDouble("8", temp_speed) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_MOVE_LINEARCore(
			temp_goalX, 
			temp_goalY, 
			temp_goalZ, 
			temp_goalRX, 
			temp_goalRY, 
			temp_goalRZ, 
			temp_acc, 
			temp_speed
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.MOVE_PATH")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		std::string temp_pathID = "";
		if(request.getString("1", temp_pathID) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: pathID request: "<<request<<std::endl;
		}
		bool temp_overwriteSpeed = false;
		if(request.getBoolean("2", temp_overwriteSpeed) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: overwriteSpeed request: "<<request<<std::endl;
		}
		double temp_acc = 0.0;
		if(request.getDouble("3", temp_acc) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: acc request: "<<request<<std::endl;
		}
		double temp_speed = 0.0;
		if(request.getDouble("4", temp_speed) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: speed request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_MOVE_PATHCore(
			temp_pathID, 
			temp_overwriteSpeed, 
			temp_acc, 
			temp_speed
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.SET_PCS")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		double temp_x = 0.0;
		if(request.getDouble("1", temp_x) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: x request: "<<request<<std::endl;
		}
		double temp_y = 0.0;
		if(request.getDouble("2", temp_y) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: y request: "<<request<<std::endl;
		}
		double temp_z = 0.0;
		if(request.getDouble("3", temp_z) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: z request: "<<request<<std::endl;
		}
		double temp_rX = 0.0;
		if(request.getDouble("4", temp_rX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rX request: "<<request<<std::endl;
		}
		double temp_rY = 0.0;
		if(request.getDouble("5", temp_rY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rY request: "<<request<<std::endl;
		}
		double temp_rZ = 0.0;
		if(request.getDouble("6", temp_rZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rZ request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_SET_PCSCore(
			temp_x, 
			temp_y, 
			temp_z, 
			temp_rX, 
			temp_rY, 
			temp_rZ
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.SET_TCP")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		double temp_x = 0.0;
		if(request.getDouble("1", temp_x) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: x request: "<<request<<std::endl;
		}
		double temp_y = 0.0;
		if(request.getDouble("2", temp_y) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: y request: "<<request<<std::endl;
		}
		double temp_z = 0.0;
		if(request.getDouble("3", temp_z) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: z request: "<<request<<std::endl;
		}
		double temp_rX = 0.0;
		if(request.getDouble("4", temp_rX) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rX request: "<<request<<std::endl;
		}
		double temp_rY = 0.0;
		if(request.getDouble("5", temp_rY) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rY request: "<<request<<std::endl;
		}
		double temp_rZ = 0.0;
		if(request.getDouble("6", temp_rZ) != 0) {
			answer.setResponse(SmartACE::ParamResponseType::INVALID);
			std::cout<<"ParamUpdateHandler - error parsing value: rZ request: "<<request<<std::endl;
		}
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_SET_TCPCore(
			temp_x, 
			temp_y, 
			temp_z, 
			temp_rX, 
			temp_rY, 
			temp_rZ
			);
		}
	}
	else if (tag == "COMMMANIPULATOROBJECTS.MANIPULATORPARAMETER.START_PROGRAM")
	{
		answer.setResponse(SmartACE::ParamResponseType::OK);
		
		
		if(answer.getResponse() == SmartACE::ParamResponseType::OK) {
			triggerHandler.handleCommManipulatorObjects_ManipulatorParameter_START_PROGRAMCore(
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
		// parameter base
		if(parameter.getDouble("base", "base_a", commitState.base.base_a))
		{
			globalState.base.base_a = commitState.base.base_a;
		}
		if(parameter.getBoolean("base", "on_base", commitState.base.on_base))
		{
			globalState.base.on_base = commitState.base.on_base;
		}
		if(parameter.getDouble("base", "steer_a", commitState.base.steer_a))
		{
			globalState.base.steer_a = commitState.base.steer_a;
		}
		if(parameter.getDouble("base", "turret_a", commitState.base.turret_a))
		{
			globalState.base.turret_a = commitState.base.turret_a;
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
		// parameter manipulator
		if(parameter.getDouble("manipulator", "azimuth", commitState.manipulator.azimuth))
		{
			globalState.manipulator.azimuth = commitState.manipulator.azimuth;
		}
		if(parameter.getDouble("manipulator", "elevation", commitState.manipulator.elevation))
		{
			globalState.manipulator.elevation = commitState.manipulator.elevation;
		}
		if(parameter.getDouble("manipulator", "roll", commitState.manipulator.roll))
		{
			globalState.manipulator.roll = commitState.manipulator.roll;
		}
		if(parameter.getBoolean("manipulator", "verbose", commitState.manipulator.verbose))
		{
			globalState.manipulator.verbose = commitState.manipulator.verbose;
		}
		if(parameter.getInteger("manipulator", "x", commitState.manipulator.x))
		{
			globalState.manipulator.x = commitState.manipulator.x;
		}
		if(parameter.getInteger("manipulator", "y", commitState.manipulator.y))
		{
			globalState.manipulator.y = commitState.manipulator.y;
		}
		if(parameter.getInteger("manipulator", "z", commitState.manipulator.z))
		{
			globalState.manipulator.z = commitState.manipulator.z;
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
