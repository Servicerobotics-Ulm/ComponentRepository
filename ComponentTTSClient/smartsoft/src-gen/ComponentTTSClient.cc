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
#include "ComponentTTSClient.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentTTSClientAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentTTSClient* ComponentTTSClient::_componentTTSClient = 0;

// constructor
ComponentTTSClient::ComponentTTSClient()
{
	std::cout << "constructor of ComponentTTSClient\n";
	
	// set all pointer members to NULL
	//componentTTSClientParams = NULL;
	consoleTask = NULL;
	consoleTaskTrigger = NULL;
	//coordinationPort = NULL;
	speechQueryServiceReq = NULL;
	speechSendServiceOut = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentTTSClient";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.speechQueryServiceReq.initialConnect = false;
	connections.speechQueryServiceReq.wiringName = "SpeechQueryServiceReq";
	connections.speechQueryServiceReq.serverName = "unknown";
	connections.speechQueryServiceReq.serviceName = "unknown";
	connections.speechQueryServiceReq.interval = 1;
	connections.speechQueryServiceReq.roboticMiddleware = "ACE_SmartSoft";
	connections.speechSendServiceOut.initialConnect = false;
	connections.speechSendServiceOut.wiringName = "SpeechSendServiceOut";
	connections.speechSendServiceOut.serverName = "unknown";
	connections.speechSendServiceOut.serviceName = "unknown";
	connections.speechSendServiceOut.interval = 1;
	connections.speechSendServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.consoleTask.minActFreq = 0.0;
	connections.consoleTask.maxActFreq = 0.0;
	// scheduling default parameters
	connections.consoleTask.scheduler = "DEFAULT";
	connections.consoleTask.priority = -1;
	connections.consoleTask.cpuAffinity = -1;
	
	// initialize members of ComponentTTSClientROSExtension
	
	// initialize members of PlainOpcUaComponentTTSClientExtension
	
}

void ComponentTTSClient::addPortFactory(const std::string &name, ComponentTTSClientPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentTTSClient::addExtension(ComponentTTSClientExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentTTSClient::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentTTSClient::connectSpeechQueryServiceReq(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.speechQueryServiceReq.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = speechQueryServiceReq->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->speechQueryServiceReq->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	return status;
}
Smart::StatusCode ComponentTTSClient::connectSpeechSendServiceOut(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.speechSendServiceOut.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = speechSendServiceOut->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->speechSendServiceOut->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentTTSClient::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectSpeechQueryServiceReq(connections.speechQueryServiceReq.serverName, connections.speechQueryServiceReq.serviceName);
	if(status != Smart::SMART_OK) return status;
	status = connectSpeechSendServiceOut(connections.speechSendServiceOut.serverName, connections.speechSendServiceOut.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentTTSClient::startAllTasks() {
	// start task ConsoleTask
	if(connections.consoleTask.scheduler != "DEFAULT") {
		ACE_Sched_Params consoleTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.consoleTask.scheduler == "FIFO") {
			consoleTask_SchedParams.policy(ACE_SCHED_FIFO);
			consoleTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.consoleTask.scheduler == "RR") {
			consoleTask_SchedParams.policy(ACE_SCHED_RR);
			consoleTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		consoleTask->start(consoleTask_SchedParams, connections.consoleTask.cpuAffinity);
	} else {
		consoleTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentTTSClient::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentTTSClient::getInputTaskTriggerFromString(const std::string &client)
{
	
	return NULL;
}


void ComponentTTSClient::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		// print out the actual parameters which are used to initialize the component
		std::cout << " \nComponentDefinition Initial-Parameters:\n" << COMP->getGlobalState() << std::endl;
		
		// initializations of ComponentTTSClientROSExtension
		
		// initializations of PlainOpcUaComponentTTSClientExtension
		
		
		// initialize all registered port-factories
		for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
		{
			portFactory->second->initialize(this, argc, argv);
		}
		
		// initialize all registered component-extensions
		for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
		{
			extension->second->initialize(this, argc, argv);
		}
		
		ComponentTTSClientPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentTTSClientAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentTTSClient is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		
		// create client ports
		speechQueryServiceReq = portFactoryRegistry[connections.speechQueryServiceReq.roboticMiddleware]->createSpeechQueryServiceReq();
		speechSendServiceOut = portFactoryRegistry[connections.speechSendServiceOut.roboticMiddleware]->createSpeechSendServiceOut();
		
		// create InputTaskTriggers and UpcallManagers
		
		// create input-handler
		
		// create request-handlers
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateSlave = new SmartACE::StateSlave(component, stateChangeHandler);
		status = stateSlave->setUpInitialState(connections.component.initialComponentMode);
		if (status != Smart::SMART_OK) std::cerr << status << "; failed setting initial ComponentMode: " << connections.component.initialComponentMode << std::endl;
		// activate state slave
		status = stateSlave->activate();
		if(status != Smart::SMART_OK) std::cerr << "ERROR: activate state" << std::endl;
		
		wiringSlave = new SmartACE::WiringSlave(component);
		// add client port to wiring slave
		if(connections.speechQueryServiceReq.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::QueryClient<DomainSpeech::CommSpeechOutputMessage, CommBasicObjects::CommPropertySet>*>(speechQueryServiceReq)->add(wiringSlave, connections.speechQueryServiceReq.wiringName);
		}
		if(connections.speechSendServiceOut.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::SendClient<DomainSpeech::CommSpeechOutputMessage>*>(speechSendServiceOut)->add(wiringSlave, connections.speechSendServiceOut.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task ConsoleTask
		consoleTask = new ConsoleTask(component);
		// configure input-links
		// configure task-trigger (if task is configurable)
		if(connections.consoleTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.consoleTask.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(consoleTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				consoleTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task ConsoleTask" << std::endl;
			}
		} else if(connections.consoleTask.trigger == "DataTriggered") {
			consoleTaskTrigger = getInputTaskTriggerFromString(connections.consoleTask.inPortRef);
			if(consoleTaskTrigger != NULL) {
				consoleTaskTrigger->attach(consoleTask, connections.consoleTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.consoleTask.inPortRef << " as activation source for Task ConsoleTask" << std::endl;
			}
		} 
		
		
		// link observers with subjects
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std exception" << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}

// run the component
void ComponentTTSClient::run()
{
	stateSlave->acquire("init");
	// startup all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->onStartup();
	}
	
	// startup all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->onStartup();
	}
	stateSlave->release("init");
	
	// do not call this handler within the init state (see above) as this handler internally calls setStartupFinished() (this should be fixed in future)
	compHandler.onStartup();
	
	// this call blocks until the component is commanded to shutdown
	stateSlave->acquire("shutdown");
	
	// shutdown all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->onShutdown();
	}
	
	// shutdown all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->onShutdown();
	}
	
	if(connections.component.useLogger == true) {
		//FIXME: use logging
		//Smart::LOGGER->stopLogging();
	}
	
	compHandler.onShutdown();
	
	stateSlave->release("shutdown");
}

// clean-up component's resources
void ComponentTTSClient::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	consoleTaskTrigger->detach(consoleTask);
	delete consoleTask;

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers

	// destroy client ports
	delete speechQueryServiceReq;
	delete speechSendServiceOut;

	// destroy server ports
	// destroy event-test handlers (if needed)
	
	// destroy request-handlers
	
	delete stateSlave;
	// destroy state-change-handler
	delete stateChangeHandler;
	
	// destroy all master/slave ports
	delete wiringSlave;
	delete param;
	

	// destroy all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->destroy();
	}

	// destroy all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->destroy();
	}
	
	// destruction of ComponentTTSClientROSExtension
	
	// destruction of PlainOpcUaComponentTTSClientExtension
	
}

void ComponentTTSClient::loadParameter(int argc, char *argv[])
{
	/*
	 Parameters can be specified via command line --filename=<filename> or -f <filename>

	 With this parameter present:
	   - The component will look for the file in the current working directory,
	     a path relative to the current directory or any absolute path
	   - The component will use the default values if the file cannot be found

	 With this parameter absent:
	   - <Name of Component>.ini will be read from current working directory, if found there
	   - $SMART_ROOT/etc/<Name of Component>.ini will be read otherwise
	   - Default values will be used if neither found in working directory or /etc
	 */
	SmartACE::SmartIniParameter parameter;
	std::ifstream parameterfile;
	bool parameterFileFound = false;

	// load parameters
	try
	{
		// if paramfile is given as argument
		if(parameter.tryAddFileFromArgs(argc,argv,"filename", 'f'))
		{
			parameterFileFound = true;
			std::cout << "parameter file is loaded from an argv argument \n";
		} else if(parameter.searchFile("ComponentTTSClient.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentTTSClient.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentTTSClient.ini parameter file not found! (using default values or command line arguments)\n";
		}
		
		// add command line arguments to allow overwriting of parameters
		// from file
		parameter.addCommandLineArgs(argc,argv,"component");
		
		// initialize the naming service using the command line parameters parsed in the
		// SmartIniParameter class. The naming service parameters are expected to be in
		// the "component" parameter group.
		SmartACE::NAMING::instance()->checkForHelpArg(argc,argv);
		if(parameterFileFound) 
		{
			if(SmartACE::NAMING::instance()->init(parameter.getAllParametersFromGroup("component")) != 0) {
				// initialization of naming service failed
				throw std::logic_error( "<NamingService> Service initialization failed!\nPossible causes could be:\n-> Erroneous configuration.\n-> Naming service not reachable.\n" );
			}
		} else {
			if(SmartACE::NAMING::instance()->init(argc, argv) != 0) {
				// initialization of naming service failed
				throw std::logic_error( "<NamingService> Service initialization failed!\nPossible causes could be:\n-> Erroneous configuration.\n-> Naming service not reachable.\n" );
			}
		}
			
		// print all known parameters
		// parameter.print();
		
		//--- server port // client port // other parameter ---
		// load parameter
		parameter.getString("component", "name", connections.component.name);
		parameter.getString("component", "initialComponentMode", connections.component.initialComponentMode);
		if(parameter.checkIfParameterExists("component", "defaultScheduler")) {
			parameter.getString("component", "defaultScheduler", connections.component.defaultScheduler);
		}
		if(parameter.checkIfParameterExists("component", "useLogger")) {
			parameter.getBoolean("component", "useLogger", connections.component.useLogger);
		}
		
		// load parameters for client SpeechQueryServiceReq
		parameter.getBoolean("SpeechQueryServiceReq", "initialConnect", connections.speechQueryServiceReq.initialConnect);
		parameter.getString("SpeechQueryServiceReq", "serviceName", connections.speechQueryServiceReq.serviceName);
		parameter.getString("SpeechQueryServiceReq", "serverName", connections.speechQueryServiceReq.serverName);
		parameter.getString("SpeechQueryServiceReq", "wiringName", connections.speechQueryServiceReq.wiringName);
		if(parameter.checkIfParameterExists("SpeechQueryServiceReq", "roboticMiddleware")) {
			parameter.getString("SpeechQueryServiceReq", "roboticMiddleware", connections.speechQueryServiceReq.roboticMiddleware);
		}
		// load parameters for client SpeechSendServiceOut
		parameter.getBoolean("SpeechSendServiceOut", "initialConnect", connections.speechSendServiceOut.initialConnect);
		parameter.getString("SpeechSendServiceOut", "serviceName", connections.speechSendServiceOut.serviceName);
		parameter.getString("SpeechSendServiceOut", "serverName", connections.speechSendServiceOut.serverName);
		parameter.getString("SpeechSendServiceOut", "wiringName", connections.speechSendServiceOut.wiringName);
		if(parameter.checkIfParameterExists("SpeechSendServiceOut", "roboticMiddleware")) {
			parameter.getString("SpeechSendServiceOut", "roboticMiddleware", connections.speechSendServiceOut.roboticMiddleware);
		}
		
		
		// load parameters for task ConsoleTask
		parameter.getDouble("ConsoleTask", "minActFreqHz", connections.consoleTask.minActFreq);
		parameter.getDouble("ConsoleTask", "maxActFreqHz", connections.consoleTask.maxActFreq);
		parameter.getString("ConsoleTask", "triggerType", connections.consoleTask.trigger);
		if(connections.consoleTask.trigger == "PeriodicTimer") {
			parameter.getDouble("ConsoleTask", "periodicActFreqHz", connections.consoleTask.periodicActFreq);
		} else if(connections.consoleTask.trigger == "DataTriggered") {
			parameter.getString("ConsoleTask", "inPortRef", connections.consoleTask.inPortRef);
			parameter.getInteger("ConsoleTask", "prescale", connections.consoleTask.prescale);
		}
		if(parameter.checkIfParameterExists("ConsoleTask", "scheduler")) {
			parameter.getString("ConsoleTask", "scheduler", connections.consoleTask.scheduler);
		}
		if(parameter.checkIfParameterExists("ConsoleTask", "priority")) {
			parameter.getInteger("ConsoleTask", "priority", connections.consoleTask.priority);
		}
		if(parameter.checkIfParameterExists("ConsoleTask", "cpuAffinity")) {
			parameter.getInteger("ConsoleTask", "cpuAffinity", connections.consoleTask.cpuAffinity);
		}
		
		// load parameters for ComponentTTSClientROSExtension
		
		// load parameters for PlainOpcUaComponentTTSClientExtension
		
		
		// load parameters for all registered component-extensions
		for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
		{
			extension->second->loadParameters(parameter);
		}
		
		paramHandler.loadParameter(parameter);
	
	} catch (const SmartACE::IniParameterError & e) {
		std::cerr << e.what() << std::endl;
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std::exception: " << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}