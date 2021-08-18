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
#include "ComponentKeyboardNavigation.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentKeyboardNavigationAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentKeyboardNavigation* ComponentKeyboardNavigation::_componentKeyboardNavigation = 0;

// constructor
ComponentKeyboardNavigation::ComponentKeyboardNavigation()
{
	std::cout << "constructor of ComponentKeyboardNavigation\n";
	
	// set all pointer members to NULL
	//componentKeyboardNavigationParams = NULL;
	//coordinationPort = NULL;
	keyboardInputTask = NULL;
	keyboardInputTaskTrigger = NULL;
	navVelSendClient = NULL;
	navVelSendClientWrapper = NULL;
	stateChangeHandler = NULL;
	stateActivityManager = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentKeyboardNavigation";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.navVelSendClient.initialConnect = false;
	connections.navVelSendClient.wiringName = "navVelSendClient";
	connections.navVelSendClient.serverName = "unknown";
	connections.navVelSendClient.serviceName = "unknown";
	connections.navVelSendClient.interval = 1;
	connections.navVelSendClient.roboticMiddleware = "ACE_SmartSoft";
	connections.keyboardInputTask.minActFreq = 0.0;
	connections.keyboardInputTask.maxActFreq = 0.0;
	connections.keyboardInputTask.trigger = "PeriodicTimer";
	connections.keyboardInputTask.periodicActFreq = 2.0;
	// scheduling default parameters
	connections.keyboardInputTask.scheduler = "DEFAULT";
	connections.keyboardInputTask.priority = -1;
	connections.keyboardInputTask.cpuAffinity = -1;
	
}

void ComponentKeyboardNavigation::addPortFactory(const std::string &name, ComponentKeyboardNavigationPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentKeyboardNavigation::addExtension(ComponentKeyboardNavigationExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentKeyboardNavigation::getComponentImpl()
{
	return dynamic_cast<ComponentKeyboardNavigationAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentKeyboardNavigation::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentKeyboardNavigation::connectNavVelSendClient(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.navVelSendClient.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = navVelSendClient->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->navVelSendClient->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentKeyboardNavigation::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectNavVelSendClient(connections.navVelSendClient.serverName, connections.navVelSendClient.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentKeyboardNavigation::startAllTasks() {
	// start task KeyboardInputTask
	if(connections.keyboardInputTask.scheduler != "DEFAULT") {
		ACE_Sched_Params keyboardInputTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.keyboardInputTask.scheduler == "FIFO") {
			keyboardInputTask_SchedParams.policy(ACE_SCHED_FIFO);
			#if defined(ACE_HAS_PTHREADS)
				keyboardInputTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				keyboardInputTask_SchedParams.priority(THREAD_PRIORITY_IDLE);
			#endif
		} else if(connections.keyboardInputTask.scheduler == "RR") {
			keyboardInputTask_SchedParams.policy(ACE_SCHED_RR);
			#if defined(ACE_HAS_PTHREADS)
				keyboardInputTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				keyboardInputTask_SchedParams.priority(THREAD_PRIORITY_IDLE);
			#endif
		}
		keyboardInputTask->start(keyboardInputTask_SchedParams, connections.keyboardInputTask.cpuAffinity);
	} else {
		keyboardInputTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentKeyboardNavigation::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentKeyboardNavigation::getInputTaskTriggerFromString(const std::string &client)
{
	
	return NULL;
}


void ComponentKeyboardNavigation::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		// print out the actual parameters which are used to initialize the component
		std::cout << " \nComponentDefinition Initial-Parameters:\n" << COMP->getParameters() << std::endl;
		
		
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
		
		ComponentKeyboardNavigationPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentKeyboardNavigationAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentKeyboardNavigation is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		
		// create client ports
		navVelSendClient = portFactoryRegistry[connections.navVelSendClient.roboticMiddleware]->createNavVelSendClient();
		navVelSendClientWrapper = new NavVelSendClientWrapper(navVelSendClient);
		
		// create InputTaskTriggers and UpcallManagers
		
		// create input-handler
		
		// create request-handlers
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateActivityManager = new StateActivityManager(stateChangeHandler);
		stateSlave = new SmartACE::StateSlave(component, stateActivityManager);
		status = stateSlave->setUpInitialState(connections.component.initialComponentMode);
		if (status != Smart::SMART_OK) std::cerr << status << "; failed setting initial ComponentMode: " << connections.component.initialComponentMode << std::endl;
		// activate state slave
		status = stateSlave->activate();
		if(status != Smart::SMART_OK) std::cerr << "ERROR: activate state" << std::endl;
		
		wiringSlave = new SmartACE::WiringSlave(component);
		// add client port to wiring slave
		if(connections.navVelSendClient.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::SendClient<CommBasicObjects::CommNavigationVelocity>*>(navVelSendClient)->add(wiringSlave, connections.navVelSendClient.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task KeyboardInputTask
		keyboardInputTask = new KeyboardInputTask(component);
		// configure input-links
		// configure task-trigger (if task is configurable)
		if(connections.keyboardInputTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = (int)(1000.0*1000.0 / connections.keyboardInputTask.periodicActFreq);
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(keyboardInputTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				keyboardInputTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task KeyboardInputTask" << std::endl;
			}
		} else if(connections.keyboardInputTask.trigger == "DataTriggered") {
			keyboardInputTaskTrigger = getInputTaskTriggerFromString(connections.keyboardInputTask.inPortRef);
			if(keyboardInputTaskTrigger != NULL) {
				keyboardInputTaskTrigger->attach(keyboardInputTask, connections.keyboardInputTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.keyboardInputTask.inPortRef << " as activation source for Task KeyboardInputTask" << std::endl;
			}
		} else
		{
			// setup default task-trigger as PeriodicTimer
			Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
			int microseconds = (int)(1000.0*1000.0 / 2.0);
			if(microseconds > 0) {
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				triggerPtr->attach(keyboardInputTask);
				// store trigger in class member
				keyboardInputTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task KeyboardInputTask" << std::endl;
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
void ComponentKeyboardNavigation::run()
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
void ComponentKeyboardNavigation::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	if(keyboardInputTaskTrigger != NULL){
		keyboardInputTaskTrigger->detach(keyboardInputTask);
		delete keyboardInputTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers

	// destroy client ports
	delete navVelSendClientWrapper;
	delete navVelSendClient;

	// destroy request-handlers

	// destroy server ports
	
	// destroy event-test handlers (if needed)
	
	delete stateSlave;
	delete stateActivityManager;
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
	
}

void ComponentKeyboardNavigation::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentKeyboardNavigation.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentKeyboardNavigation.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentKeyboardNavigation.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for client navVelSendClient
		parameter.getBoolean("navVelSendClient", "initialConnect", connections.navVelSendClient.initialConnect);
		parameter.getString("navVelSendClient", "serviceName", connections.navVelSendClient.serviceName);
		parameter.getString("navVelSendClient", "serverName", connections.navVelSendClient.serverName);
		parameter.getString("navVelSendClient", "wiringName", connections.navVelSendClient.wiringName);
		if(parameter.checkIfParameterExists("navVelSendClient", "roboticMiddleware")) {
			parameter.getString("navVelSendClient", "roboticMiddleware", connections.navVelSendClient.roboticMiddleware);
		}
		
		
		// load parameters for task KeyboardInputTask
		parameter.getDouble("KeyboardInputTask", "minActFreqHz", connections.keyboardInputTask.minActFreq);
		parameter.getDouble("KeyboardInputTask", "maxActFreqHz", connections.keyboardInputTask.maxActFreq);
		parameter.getString("KeyboardInputTask", "triggerType", connections.keyboardInputTask.trigger);
		if(connections.keyboardInputTask.trigger == "PeriodicTimer") {
			parameter.getDouble("KeyboardInputTask", "periodicActFreqHz", connections.keyboardInputTask.periodicActFreq);
		} else if(connections.keyboardInputTask.trigger == "DataTriggered") {
			parameter.getString("KeyboardInputTask", "inPortRef", connections.keyboardInputTask.inPortRef);
			parameter.getInteger("KeyboardInputTask", "prescale", connections.keyboardInputTask.prescale);
		}
		if(parameter.checkIfParameterExists("KeyboardInputTask", "scheduler")) {
			parameter.getString("KeyboardInputTask", "scheduler", connections.keyboardInputTask.scheduler);
		}
		if(parameter.checkIfParameterExists("KeyboardInputTask", "priority")) {
			parameter.getInteger("KeyboardInputTask", "priority", connections.keyboardInputTask.priority);
		}
		if(parameter.checkIfParameterExists("KeyboardInputTask", "cpuAffinity")) {
			parameter.getInteger("KeyboardInputTask", "cpuAffinity", connections.keyboardInputTask.cpuAffinity);
		}
		
		
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
