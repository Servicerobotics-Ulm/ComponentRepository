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
#include "ComponentWebotsMpsDocking.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentWebotsMpsDockingAcePortFactory.hh"

#include "RobotDockingEventServiceOutEventTestHandler.hh"

// initialize static singleton pointer to zero
ComponentWebotsMpsDocking* ComponentWebotsMpsDocking::_componentWebotsMpsDocking = 0;

// constructor
ComponentWebotsMpsDocking::ComponentWebotsMpsDocking()
{
	std::cout << "constructor of ComponentWebotsMpsDocking\n";
	
	// set all pointer members to NULL
	baseStateServiceIn = NULL;
	baseStateServiceInInputTaskTrigger = NULL;
	baseStateServiceInUpcallManager = NULL;
	baseStateServiceInInputCollector = NULL;
	//componentWebotsMpsDocking = NULL;
	//coordinationPort = NULL;
	dockingTask = NULL;
	dockingTaskTrigger = NULL;
	laserServiceIn = NULL;
	laserServiceInInputTaskTrigger = NULL;
	laserServiceInUpcallManager = NULL;
	laserServiceInInputCollector = NULL;
	navigationVelocityServiceOut = NULL;
	navigationVelocityServiceOutWrapper = NULL;
	robotDockingEventServiceOut = NULL;
	robotDockingEventServiceOutWrapper = NULL;
	robotDockingEventServiceOutEventTestHandler = nullptr; 
	trafficLightsServiceOut = NULL;
	trafficLightsServiceOutWrapper = NULL;
	stateChangeHandler = NULL;
	stateActivityManager = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentWebotsMpsDocking";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.robotDockingEventServiceOut.serviceName = "RobotDockingEventServiceOut";
	connections.robotDockingEventServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.trafficLightsServiceOut.serviceName = "TrafficLightsServiceOut";
	connections.trafficLightsServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.baseStateServiceIn.initialConnect = false;
	connections.baseStateServiceIn.wiringName = "BaseStateServiceIn";
	connections.baseStateServiceIn.serverName = "unknown";
	connections.baseStateServiceIn.serviceName = "unknown";
	connections.baseStateServiceIn.interval = 1;
	connections.baseStateServiceIn.roboticMiddleware = "ACE_SmartSoft";
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
	connections.dockingTask.minActFreq = 0.0;
	connections.dockingTask.maxActFreq = 0.0;
	// scheduling default parameters
	connections.dockingTask.scheduler = "DEFAULT";
	connections.dockingTask.priority = -1;
	connections.dockingTask.cpuAffinity = -1;
	
}

void ComponentWebotsMpsDocking::addPortFactory(const std::string &name, ComponentWebotsMpsDockingPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentWebotsMpsDocking::addExtension(ComponentWebotsMpsDockingExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentWebotsMpsDocking::getComponentImpl()
{
	return dynamic_cast<ComponentWebotsMpsDockingAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentWebotsMpsDocking::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentWebotsMpsDocking::connectBaseStateServiceIn(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.baseStateServiceIn.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = baseStateServiceIn->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->baseStateServiceIn->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	baseStateServiceIn->subscribe(connections.baseStateServiceIn.interval);
	return status;
}
Smart::StatusCode ComponentWebotsMpsDocking::connectLaserServiceIn(const std::string &serverName, const std::string &serviceName) {
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
Smart::StatusCode ComponentWebotsMpsDocking::connectNavigationVelocityServiceOut(const std::string &serverName, const std::string &serviceName) {
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
Smart::StatusCode ComponentWebotsMpsDocking::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectBaseStateServiceIn(connections.baseStateServiceIn.serverName, connections.baseStateServiceIn.serviceName);
	if(status != Smart::SMART_OK) return status;
	status = connectLaserServiceIn(connections.laserServiceIn.serverName, connections.laserServiceIn.serviceName);
	if(status != Smart::SMART_OK) return status;
	status = connectNavigationVelocityServiceOut(connections.navigationVelocityServiceOut.serverName, connections.navigationVelocityServiceOut.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentWebotsMpsDocking::startAllTasks() {
	// start task DockingTask
	if(connections.dockingTask.scheduler != "DEFAULT") {
		ACE_Sched_Params dockingTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.dockingTask.scheduler == "FIFO") {
			dockingTask_SchedParams.policy(ACE_SCHED_FIFO);
			#if defined(ACE_HAS_PTHREADS)
				dockingTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				dockingTask_SchedParams.priority(THREAD_PRIORITY_IDLE);
			#endif
		} else if(connections.dockingTask.scheduler == "RR") {
			dockingTask_SchedParams.policy(ACE_SCHED_RR);
			#if defined(ACE_HAS_PTHREADS)
				dockingTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				dockingTask_SchedParams.priority(THREAD_PRIORITY_IDLE);
			#endif
		}
		dockingTask->start(dockingTask_SchedParams, connections.dockingTask.cpuAffinity);
	} else {
		dockingTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentWebotsMpsDocking::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentWebotsMpsDocking::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "BaseStateServiceIn") return baseStateServiceInInputTaskTrigger;
	if(client == "LaserServiceIn") return laserServiceInInputTaskTrigger;
	
	return NULL;
}


void ComponentWebotsMpsDocking::init(int argc, char *argv[])
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
		
		ComponentWebotsMpsDockingPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentWebotsMpsDockingAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentWebotsMpsDocking is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		robotDockingEventServiceOutEventTestHandler = std::make_shared<RobotDockingEventServiceOutEventTestHandler>();
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		robotDockingEventServiceOutEventTestHandler = std::make_shared<RobotDockingEventServiceOutEventTestHandler>();
		robotDockingEventServiceOut = portFactoryRegistry[connections.robotDockingEventServiceOut.roboticMiddleware]->createRobotDockingEventServiceOut(connections.robotDockingEventServiceOut.serviceName, robotDockingEventServiceOutEventTestHandler);
		robotDockingEventServiceOutWrapper = new RobotDockingEventServiceOutWrapper(robotDockingEventServiceOut);
		trafficLightsServiceOut = portFactoryRegistry[connections.trafficLightsServiceOut.roboticMiddleware]->createTrafficLightsServiceOut(connections.trafficLightsServiceOut.serviceName);
		trafficLightsServiceOutWrapper = new TrafficLightsServiceOutWrapper(trafficLightsServiceOut);
		
		// create client ports
		baseStateServiceIn = portFactoryRegistry[connections.baseStateServiceIn.roboticMiddleware]->createBaseStateServiceIn();
		laserServiceIn = portFactoryRegistry[connections.laserServiceIn.roboticMiddleware]->createLaserServiceIn();
		navigationVelocityServiceOut = portFactoryRegistry[connections.navigationVelocityServiceOut.roboticMiddleware]->createNavigationVelocityServiceOut();
		navigationVelocityServiceOutWrapper = new NavigationVelocityServiceOutWrapper(navigationVelocityServiceOut);
		
		// create InputTaskTriggers and UpcallManagers
		baseStateServiceInInputCollector = new BaseStateServiceInInputCollector(baseStateServiceIn);
		baseStateServiceInInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommBaseState>(baseStateServiceInInputCollector);
		baseStateServiceInUpcallManager = new BaseStateServiceInUpcallManager(baseStateServiceInInputCollector);
		laserServiceInInputCollector = new LaserServiceInInputCollector(laserServiceIn);
		laserServiceInInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommMobileLaserScan>(laserServiceInInputCollector);
		laserServiceInUpcallManager = new LaserServiceInUpcallManager(laserServiceInInputCollector);
		
		// create input-handler
		
		// create request-handlers
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateActivityManager = new StateActivityManager(stateChangeHandler);
		stateSlave = new SmartACE::StateSlave(component, stateActivityManager);
		if (stateSlave->defineStates("IrDocking" ,"irDocking") != Smart::SMART_OK) std::cerr << "ERROR: defining state combinaion IrDocking.irDocking" << std::endl;
		if (stateSlave->defineStates("UnDocking" ,"unDocking") != Smart::SMART_OK) std::cerr << "ERROR: defining state combinaion UnDocking.unDocking" << std::endl;
		if (stateSlave->defineStates("LaserDocking" ,"laserDocking") != Smart::SMART_OK) std::cerr << "ERROR: defining state combinaion LaserDocking.laserDocking" << std::endl;
		status = stateSlave->setUpInitialState(connections.component.initialComponentMode);
		if (status != Smart::SMART_OK) std::cerr << status << "; failed setting initial ComponentMode: " << connections.component.initialComponentMode << std::endl;
		// activate state slave
		status = stateSlave->activate();
		if(status != Smart::SMART_OK) std::cerr << "ERROR: activate state" << std::endl;
		
		wiringSlave = new SmartACE::WiringSlave(component);
		// add client port to wiring slave
		if(connections.baseStateServiceIn.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<CommBasicObjects::CommBaseState>*>(baseStateServiceIn)->add(wiringSlave, connections.baseStateServiceIn.wiringName);
		}
		if(connections.laserServiceIn.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<CommBasicObjects::CommMobileLaserScan>*>(laserServiceIn)->add(wiringSlave, connections.laserServiceIn.wiringName);
		}
		if(connections.navigationVelocityServiceOut.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::SendClient<CommBasicObjects::CommNavigationVelocity>*>(navigationVelocityServiceOut)->add(wiringSlave, connections.navigationVelocityServiceOut.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task DockingTask
		dockingTask = new DockingTask(component);
		// configure input-links
		baseStateServiceInUpcallManager->attach(dockingTask);
		laserServiceInUpcallManager->attach(dockingTask);
		// configure task-trigger (if task is configurable)
		if(connections.dockingTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = (int)(1000.0*1000.0 / connections.dockingTask.periodicActFreq);
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(dockingTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				dockingTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task DockingTask" << std::endl;
			}
		} else if(connections.dockingTask.trigger == "DataTriggered") {
			dockingTaskTrigger = getInputTaskTriggerFromString(connections.dockingTask.inPortRef);
			if(dockingTaskTrigger != NULL) {
				dockingTaskTrigger->attach(dockingTask, connections.dockingTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.dockingTask.inPortRef << " as activation source for Task DockingTask" << std::endl;
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
void ComponentWebotsMpsDocking::run()
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
void ComponentWebotsMpsDocking::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	baseStateServiceInUpcallManager->detach(dockingTask);
	laserServiceInUpcallManager->detach(dockingTask);
	// unlink the TaskTrigger
	if(dockingTaskTrigger != NULL){
		dockingTaskTrigger->detach(dockingTask);
		delete dockingTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers
	delete baseStateServiceInInputTaskTrigger;
	delete baseStateServiceInUpcallManager;
	delete baseStateServiceInInputCollector;
	delete laserServiceInInputTaskTrigger;
	delete laserServiceInUpcallManager;
	delete laserServiceInInputCollector;

	// destroy client ports
	delete baseStateServiceIn;
	delete laserServiceIn;
	delete navigationVelocityServiceOutWrapper;
	delete navigationVelocityServiceOut;

	// destroy request-handlers

	// destroy server ports
	delete robotDockingEventServiceOutWrapper;
	delete robotDockingEventServiceOut;
	delete trafficLightsServiceOutWrapper;
	delete trafficLightsServiceOut;
	
	// destroy event-test handlers (if needed)
	robotDockingEventServiceOutEventTestHandler;
	
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

void ComponentWebotsMpsDocking::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentWebotsMpsDocking.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentWebotsMpsDocking.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentWebotsMpsDocking.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for client BaseStateServiceIn
		parameter.getBoolean("BaseStateServiceIn", "initialConnect", connections.baseStateServiceIn.initialConnect);
		parameter.getString("BaseStateServiceIn", "serviceName", connections.baseStateServiceIn.serviceName);
		parameter.getString("BaseStateServiceIn", "serverName", connections.baseStateServiceIn.serverName);
		parameter.getString("BaseStateServiceIn", "wiringName", connections.baseStateServiceIn.wiringName);
		parameter.getInteger("BaseStateServiceIn", "interval", connections.baseStateServiceIn.interval);
		if(parameter.checkIfParameterExists("BaseStateServiceIn", "roboticMiddleware")) {
			parameter.getString("BaseStateServiceIn", "roboticMiddleware", connections.baseStateServiceIn.roboticMiddleware);
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
		
		// load parameters for server RobotDockingEventServiceOut
		parameter.getString("RobotDockingEventServiceOut", "serviceName", connections.robotDockingEventServiceOut.serviceName);
		if(parameter.checkIfParameterExists("RobotDockingEventServiceOut", "roboticMiddleware")) {
			parameter.getString("RobotDockingEventServiceOut", "roboticMiddleware", connections.robotDockingEventServiceOut.roboticMiddleware);
		}
		// load parameters for server TrafficLightsServiceOut
		parameter.getString("TrafficLightsServiceOut", "serviceName", connections.trafficLightsServiceOut.serviceName);
		if(parameter.checkIfParameterExists("TrafficLightsServiceOut", "roboticMiddleware")) {
			parameter.getString("TrafficLightsServiceOut", "roboticMiddleware", connections.trafficLightsServiceOut.roboticMiddleware);
		}
		
		// load parameters for task DockingTask
		parameter.getDouble("DockingTask", "minActFreqHz", connections.dockingTask.minActFreq);
		parameter.getDouble("DockingTask", "maxActFreqHz", connections.dockingTask.maxActFreq);
		parameter.getString("DockingTask", "triggerType", connections.dockingTask.trigger);
		if(connections.dockingTask.trigger == "PeriodicTimer") {
			parameter.getDouble("DockingTask", "periodicActFreqHz", connections.dockingTask.periodicActFreq);
		} else if(connections.dockingTask.trigger == "DataTriggered") {
			parameter.getString("DockingTask", "inPortRef", connections.dockingTask.inPortRef);
			parameter.getInteger("DockingTask", "prescale", connections.dockingTask.prescale);
		}
		if(parameter.checkIfParameterExists("DockingTask", "scheduler")) {
			parameter.getString("DockingTask", "scheduler", connections.dockingTask.scheduler);
		}
		if(parameter.checkIfParameterExists("DockingTask", "priority")) {
			parameter.getInteger("DockingTask", "priority", connections.dockingTask.priority);
		}
		if(parameter.checkIfParameterExists("DockingTask", "cpuAffinity")) {
			parameter.getInteger("DockingTask", "cpuAffinity", connections.dockingTask.cpuAffinity);
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
