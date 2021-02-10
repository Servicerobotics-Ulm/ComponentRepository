#include "ComponentTCLSequencer.hh"

	std::string queryParam(const std::string& server, const std::string& param){
		return COMP->queryParam(COMP->paramMaster, server, param);

	}
	std::string setState(const std::string& server, const std::string& state){
		return COMP->setState(COMP->stateMaster,server, state);
	}

	std::string getState(const std::string& server){
		return COMP->getState(COMP->stateMaster,server);
	}

	std::string waitForLifeCycleState(const std::string& server, const std::string& state){
		return COMP->waitForLifeCycleState(COMP->stateMaster,server, state);
	}

	int appedEvent(const std::string& event){
		return COMP->eventInterface->append(event);
	}
