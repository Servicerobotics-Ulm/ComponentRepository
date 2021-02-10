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
#include "ComponentQtRobotConsole.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentQtRobotConsoleAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentQtRobotConsole* ComponentQtRobotConsole::_componentQtRobotConsole = 0;

// constructor
ComponentQtRobotConsole::ComponentQtRobotConsole()
{
	std::cout << "constructor of ComponentQtRobotConsole\n";
	
	// set all pointer members to NULL
	consoleTask = NULL;
	consoleTaskTrigger = NULL;
	//coordinationMaster = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	stateMaster = NULL;
	paramMaster = NULL;
	wiringMaster = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentQtRobotConsole";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.consoleTask.minActFreq = 0.0;
	connections.consoleTask.maxActFreq = 0.0;
	// scheduling default parameters
	connections.consoleTask.scheduler = "DEFAULT";
	connections.consoleTask.priority = -1;
	connections.consoleTask.cpuAffinity = -1;
	
	// initialize members of ComponentQtRobotConsoleROS1InterfacesExtension
	
	// initialize members of ComponentQtRobotConsoleROSExtension
	
	// initialize members of ComponentQtRobotConsoleRestInterfacesExtension
	
	// initialize members of OpcUaBackendComponentGeneratorExtension
	
	// initialize members of PlainOpcUaComponentQtRobotConsoleExtension
	
}

void ComponentQtRobotConsole::addPortFactory(const std::string &name, ComponentQtRobotConsolePortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentQtRobotConsole::addExtension(ComponentQtRobotConsoleExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentQtRobotConsole::getComponentImpl()
{
	return dynamic_cast<ComponentQtRobotConsoleAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentQtRobotConsole::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}




/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentQtRobotConsole::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentQtRobotConsole::startAllTasks() {
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
void ComponentQtRobotConsole::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentQtRobotConsole::getInputTaskTriggerFromString(const std::string &client)
{
	
	return NULL;
}


void ComponentQtRobotConsole::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		
		// initializations of ComponentQtRobotConsoleROS1InterfacesExtension
		
		// initializations of ComponentQtRobotConsoleROSExtension
		
		// initializations of ComponentQtRobotConsoleRestInterfacesExtension
		
		// initializations of OpcUaBackendComponentGeneratorExtension
		
		// initializations of PlainOpcUaComponentQtRobotConsoleExtension
		
		
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
		
		ComponentQtRobotConsolePortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentQtRobotConsoleAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentQtRobotConsole is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		
		// create client ports
		
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
		
		
		// create master ports
		stateMaster = new SmartACE::StateMaster(component);
		paramMaster = new SmartACE::ParameterMaster(component);
		wiringMaster = new SmartACE::WiringMaster(component);
		
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
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
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
void ComponentQtRobotConsole::run()
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
void ComponentQtRobotConsole::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	if(consoleTaskTrigger != NULL){
		consoleTaskTrigger->detach(consoleTask);
		delete consoleTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers

	// destroy client ports

	// destroy server ports
	// destroy event-test handlers (if needed)
	
	// destroy request-handlers
	
	delete stateSlave;
	// destroy state-change-handler
	delete stateChangeHandler;
	
	// destroy all master/slave ports
	delete wiringSlave;
	
	// destroy master ports
	delete stateMaster;
	delete paramMaster;
	delete wiringMaster;

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
	
	// destruction of ComponentQtRobotConsoleROS1InterfacesExtension
	
	// destruction of ComponentQtRobotConsoleROSExtension
	
	// destruction of ComponentQtRobotConsoleRestInterfacesExtension
	
	// destruction of OpcUaBackendComponentGeneratorExtension
	
	// destruction of PlainOpcUaComponentQtRobotConsoleExtension
	
}

void ComponentQtRobotConsole::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentQtRobotConsole.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentQtRobotConsole.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentQtRobotConsole.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for ComponentQtRobotConsoleROS1InterfacesExtension
		
		// load parameters for ComponentQtRobotConsoleROSExtension
		
		// load parameters for ComponentQtRobotConsoleRestInterfacesExtension
		
		// load parameters for OpcUaBackendComponentGeneratorExtension
		
		// load parameters for PlainOpcUaComponentQtRobotConsoleExtension
		
		
		// load parameters for all registered component-extensions
		for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
		{
			extension->second->loadParameters(parameter);
		}
		
	
	} catch (const SmartACE::IniParameterError & e) {
		std::cerr << e.what() << std::endl;
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std::exception: " << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}
