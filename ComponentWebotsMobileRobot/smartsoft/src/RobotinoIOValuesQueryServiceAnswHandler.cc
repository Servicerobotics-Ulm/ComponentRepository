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
#include "RobotinoIOValuesQueryServiceAnswHandler.hh"
#include "ComponentWebotsMobileRobot.hh"

RobotinoIOValuesQueryServiceAnswHandler::RobotinoIOValuesQueryServiceAnswHandler(Smart::IQueryServerPattern<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues>* server)
:	RobotinoIOValuesQueryServiceAnswHandlerCore(server)
{
	
}

RobotinoIOValuesQueryServiceAnswHandler::~RobotinoIOValuesQueryServiceAnswHandler()
{
	
}


void RobotinoIOValuesQueryServiceAnswHandler::handleQuery(const Smart::QueryIdPtr &id, const CommBasicObjects::CommIOValues& request)
{
	CommBasicObjects::CommIOValues answer;
	
	// implement your query handling logic here and fill in the answer object
	


	if(request.getDigitalOutputValuesSize()>0){
		for(unsigned int i=0;i<request.getDigitalOutputValuesSize();i++){
			CommBasicObjects::CommDigitalOutputRequest value = request.getDigitalOutputValuesElemAtPos(i);
			std::cout<<__FUNCTION__<<": Digital Output number: "<<value.getOutputNumber()<<" value: "<<value.getOutputValue()<<std::endl;
			COMP->robot->setDigitalOutput(value.getOutputNumber(), (bool)value.getOutputValue());
		}

	}

	if(request.getAnalogOutputValuesSize()>0){
		for(unsigned int i=0;i<request.getAnalogOutputValuesSize();i++){
			CommBasicObjects::CommAnalogOutputRequest value = request.getAnalogOutputValuesElemAtPos(i);
					COMP->robot->setAnalogOutput(value.getOutputNumber(), value.getOutputValue());
				}
	}

	std::vector<bool> digitalIn = COMP->robot->getDigitalInputArray();
	std::vector<float> analogIn = COMP->robot->getAnalogInputArray();

	answer.resizeDigitalInputValues(digitalIn.size());
	for(unsigned int i=0;i<digitalIn.size();i++){
		answer.setDigitalInputValuesElemAtPos(i,digitalIn[i]);
	}

	for(unsigned int i=0;i<analogIn.size();i++){
		answer.getAnalogInputValuesRef().push_back(analogIn[i]);
	}

	this->server->answer(id, answer);
}
