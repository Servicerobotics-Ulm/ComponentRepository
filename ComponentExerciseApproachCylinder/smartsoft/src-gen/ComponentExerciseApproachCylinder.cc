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
#include "ComponentExerciseApproachCylinder.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentExerciseApproachCylinderAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentExerciseApproachCylinder* ComponentExerciseApproachCylinder::_componentExerciseApproachCylinder = 0;

// constructor
ComponentExerciseApproachCylinder::ComponentExerciseApproachCylinder()
{
	std::cout << "constructor of ComponentExerciseApproachCylinder\n";
	
	// set all pointer members to NULL
	laserServiceIn = NULL;
	laserServiceInInputTaskTrigger = NULL;
	laserServiceInUpcallManager = NULL;
	laserServiceInInputCollector = NULL;
	navigationVelocityServiceOut = NULL;
	navigationVelocityServiceOutWrapper = NULL;
	robotTask = NULL;
	robotTaskTrigger = NULL;
	stateChangeHandler = NULL;
	stateActivityManager = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentExerciseApproachCylinder";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.laserServiceIn.initialConnect = false;
	connections.laserServiceIn.wiringName = "LaserServiceIn";
	connections.laserServiceIn.serverName = "unknown";
	connections.laserServiceIn.serviceName = "unknown";
	connections.laserServiceIn.interval = 1;
	connections.laserServiceIn.roboticMiddleware = "ACE_SmartSoft";
	connections.navigationVelocityServiceOut.initialConnect = false;
	connections.navigationVelocityServiceOut.wiringName = "NavigationVelocityServiceOut";
	connections.navigationVelocityServiceOut.serverName = "unknown";
	connections.navigationVelocityServiceOut.serviceName = "unknown";
	connections.navigationVelocityServiceOut.interval = 1;
	connections.navigationVelocityServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.robotTask.minActFreq = 0.0;
	connections.robotTask.maxActFreq = 0.0;
	connections.robotTask.trigger = "PeriodicTimer";
	connections.robotTask.periodicActFreq = 10.0;
	// scheduling default parameters
	connections.robotTask.scheduler = "DEFAULT";
	connections.robotTask.priority = -1;
	connections.robotTask.cpuAffinity = -1;
	
}

void ComponentExerciseApproachCylinder::addPortFactory(const std::string &name, ComponentExerciseApproachCylinderPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentExerciseApproachCylinder::addExtension(ComponentExerciseApproachCylinderExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentExerciseApproachCylinder::getComponentImpl()
{
	return dynamic_cast<ComponentExerciseApproachCylinderAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentExerciseApproachCylinder::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentExerciseApproachCylinder::connectLaserServiceIn(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.laserServiceIn.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = laserServiceIn->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->laserServiceIn->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	laserServiceIn->subscribe(connections.laserServiceIn.interval);
	return status;
}
Smart::StatusCode ComponentExerciseApproachCylinder::connectNavigationVelocityServiceOut(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.navigationVelocityServiceOut.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = navigationVelocityServiceOut->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->navigationVelocityServiceOut->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentExerciseApproachCylinder::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectLaserServiceIn(connections.laserServiceIn.serverName, connections.laserServiceIn.serviceName);
	if(status != Smart::SMART_OK) return status;
	status = connectNavigationVelocityServiceOut(connections.navigationVelocityServiceOut.serverName, connections.navigationVelocityServiceOut.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentExerciseApproachCylinder::startAllTasks() {
	// start task RobotTask
	if(connections.robotTask.scheduler != "DEFAULT") {
		ACE_Sched_Params robotTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.robotTask.scheduler == "FIFO") {
			robotTask_SchedParams.policy(ACE_SCHED_FIFO);
			#if defined(ACE_HAS_PTHREADS)
				robotTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				robotTask_SchedParams.priority(THREAD_PRIORITY_IDLE);
			#endif
		} else if(connections.robotTask.scheduler == "RR") {
			robotTask_SchedParams.policy(ACE_SCHED_RR);
			#if defined(ACE_HAS_PTHREADS)
				robotTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				robotTask_SchedParams.priority(THREAD_PRIORITY_IDLE);
			#endif
		}
		robotTask->start(robotTask_SchedParams, connections.robotTask.cpuAffinity);
	} else {
		robotTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentExerciseApproachCylinder::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentExerciseApproachCylinder::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "LaserServiceIn") return laserServiceInInputTaskTrigger;
	
	return NULL;
}


void ComponentExerciseApproachCylinder::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		
		
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
		
		ComponentExerciseApproachCylinderPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentExerciseApproachCylinderAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentExerciseApproachCylinder is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		
		// create client ports
		laserServiceIn = portFactoryRegistry[connections.laserServiceIn.roboticMiddleware]->createLaserServiceIn();
		navigationVelocityServiceOut = portFactoryRegistry[connections.navigationVelocityServiceOut.roboticMiddleware]->createNavigationVelocityServiceOut();
		navigationVelocityServiceOutWrapper = new NavigationVelocityServiceOutWrapper(navigationVelocityServiceOut);
		
		// create InputTaskTriggers and UpcallManagers
		laserServiceInInputCollector = new LaserServiceInInputCollector(laserServiceIn);
		laserServiceInInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommMobileLaserScan>(laserServiceInInputCollector);
		laserServiceInUpcallManager = new LaserServiceInUpcallManager(laserServiceInInputCollector);
		
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
		if(connections.laserServiceIn.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<CommBasicObjects::CommMobileLaserScan>*>(laserServiceIn)->add(wiringSlave, connections.laserServiceIn.wiringName);
		}
		if(connections.navigationVelocityServiceOut.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::SendClient<CommBasicObjects::CommNavigationVelocity>*>(navigationVelocityServiceOut)->add(wiringSlave, connections.navigationVelocityServiceOut.wiringName);
		}
		
		
		
		// create Task RobotTask
		robotTask = new RobotTask(component);
		// configure input-links
		laserServiceInUpcallManager->attach(robotTask);
		// configure task-trigger (if task is configurable)
		if(connections.robotTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = (int)(1000.0*1000.0 / connections.robotTask.periodicActFreq);
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(robotTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				robotTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task RobotTask" << std::endl;
			}
		} else if(connections.robotTask.trigger == "DataTriggered") {
			robotTaskTrigger = getInputTaskTriggerFromString(connections.robotTask.inPortRef);
			if(robotTaskTrigger != NULL) {
				robotTaskTrigger->attach(robotTask, connections.robotTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.robotTask.inPortRef << " as activation source for Task RobotTask" << std::endl;
			}
		} else
		{
			// setup default task-trigger as PeriodicTimer
			Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
			int microseconds = (int)(1000.0*1000.0 / 10.0);
			if(microseconds > 0) {
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				triggerPtr->attach(robotTask);
				// store trigger in class member
				robotTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task RobotTask" << std::endl;
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
void ComponentExerciseApproachCylinder::run()
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
void ComponentExerciseApproachCylinder::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	laserServiceInUpcallManager->detach(robotTask);
	// unlink the TaskTrigger
	if(robotTaskTrigger != NULL){
		robotTaskTrigger->detach(robotTask);
		delete robotTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers
	delete laserServiceInInputTaskTrigger;
	delete laserServiceInUpcallManager;
	delete laserServiceInInputCollector;

	// destroy client ports
	delete laserServiceIn;
	delete navigationVelocityServiceOutWrapper;
	delete navigationVelocityServiceOut;

	// destroy request-handlers

	// destroy server ports
	
	// destroy event-test handlers (if needed)
	
	delete stateSlave;
	delete stateActivityManager;
	// destroy state-change-handler
	delete stateChangeHandler;
	
	// destroy all master/slave ports
	delete wiringSlave;
	

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

void ComponentExerciseApproachCylinder::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentExerciseApproachCylinder.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentExerciseApproachCylinder.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentExerciseApproachCylinder.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for client LaserServiceIn
		parameter.getBoolean("LaserServiceIn", "initialConnect", connections.laserServiceIn.initialConnect);
		parameter.getString("LaserServiceIn", "serviceName", connections.laserServiceIn.serviceName);
		parameter.getString("LaserServiceIn", "serverName", connections.laserServiceIn.serverName);
		parameter.getString("LaserServiceIn", "wiringName", connections.laserServiceIn.wiringName);
		parameter.getInteger("LaserServiceIn", "interval", connections.laserServiceIn.interval);
		if(parameter.checkIfParameterExists("LaserServiceIn", "roboticMiddleware")) {
			parameter.getString("LaserServiceIn", "roboticMiddleware", connections.laserServiceIn.roboticMiddleware);
		}
		// load parameters for client NavigationVelocityServiceOut
		parameter.getBoolean("NavigationVelocityServiceOut", "initialConnect", connections.navigationVelocityServiceOut.initialConnect);
		parameter.getString("NavigationVelocityServiceOut", "serviceName", connections.navigationVelocityServiceOut.serviceName);
		parameter.getString("NavigationVelocityServiceOut", "serverName", connections.navigationVelocityServiceOut.serverName);
		parameter.getString("NavigationVelocityServiceOut", "wiringName", connections.navigationVelocityServiceOut.wiringName);
		if(parameter.checkIfParameterExists("NavigationVelocityServiceOut", "roboticMiddleware")) {
			parameter.getString("NavigationVelocityServiceOut", "roboticMiddleware", connections.navigationVelocityServiceOut.roboticMiddleware);
		}
		
		
		// load parameters for task RobotTask
		parameter.getDouble("RobotTask", "minActFreqHz", connections.robotTask.minActFreq);
		parameter.getDouble("RobotTask", "maxActFreqHz", connections.robotTask.maxActFreq);
		parameter.getString("RobotTask", "triggerType", connections.robotTask.trigger);
		if(connections.robotTask.trigger == "PeriodicTimer") {
			parameter.getDouble("RobotTask", "periodicActFreqHz", connections.robotTask.periodicActFreq);
		} else if(connections.robotTask.trigger == "DataTriggered") {
			parameter.getString("RobotTask", "inPortRef", connections.robotTask.inPortRef);
			parameter.getInteger("RobotTask", "prescale", connections.robotTask.prescale);
		}
		if(parameter.checkIfParameterExists("RobotTask", "scheduler")) {
			parameter.getString("RobotTask", "scheduler", connections.robotTask.scheduler);
		}
		if(parameter.checkIfParameterExists("RobotTask", "priority")) {
			parameter.getInteger("RobotTask", "priority", connections.robotTask.priority);
		}
		if(parameter.checkIfParameterExists("RobotTask", "cpuAffinity")) {
			parameter.getInteger("RobotTask", "cpuAffinity", connections.robotTask.cpuAffinity);
		}
		
		
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
