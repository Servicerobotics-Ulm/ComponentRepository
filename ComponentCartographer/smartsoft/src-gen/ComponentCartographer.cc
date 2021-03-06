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
#include "ComponentCartographer.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentCartographerAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentCartographer* ComponentCartographer::_componentCartographer = 0;

// constructor
ComponentCartographer::ComponentCartographer()
{
	std::cout << "constructor of ComponentCartographer\n";
	
	// set all pointer members to NULL
	cartographerTask = NULL;
	cartographerTaskTrigger = NULL;
	//coordinationPort = NULL;
	//coordinationPort = NULL;
	gridMapPushServiceOut = NULL;
	gridMapPushServiceOutWrapper = NULL;
	laserServiceIn = NULL;
	laserServiceInInputTaskTrigger = NULL;
	laserServiceInUpcallManager = NULL;
	laserServiceInInputCollector = NULL;
	localized_robot_pose = NULL;
	localized_robot_poseWrapper = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentCartographer";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.gridMapPushServiceOut.serviceName = "gridMapPushServiceOut";
	connections.gridMapPushServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.laserServiceIn.initialConnect = false;
	connections.laserServiceIn.wiringName = "laserServiceIn";
	connections.laserServiceIn.serverName = "unknown";
	connections.laserServiceIn.serviceName = "unknown";
	connections.laserServiceIn.interval = 1;
	connections.laserServiceIn.roboticMiddleware = "ACE_SmartSoft";
	connections.localized_robot_pose.initialConnect = false;
	connections.localized_robot_pose.wiringName = "localized_robot_pose";
	connections.localized_robot_pose.serverName = "unknown";
	connections.localized_robot_pose.serviceName = "unknown";
	connections.localized_robot_pose.interval = 1;
	connections.localized_robot_pose.roboticMiddleware = "ACE_SmartSoft";
	connections.cartographerTask.minActFreq = 0.0;
	connections.cartographerTask.maxActFreq = 0.0;
	// scheduling default parameters
	connections.cartographerTask.scheduler = "DEFAULT";
	connections.cartographerTask.priority = -1;
	connections.cartographerTask.cpuAffinity = -1;
	
}

void ComponentCartographer::addPortFactory(const std::string &name, ComponentCartographerPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentCartographer::addExtension(ComponentCartographerExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentCartographer::getComponentImpl()
{
	return dynamic_cast<ComponentCartographerAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentCartographer::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentCartographer::connectLaserServiceIn(const std::string &serverName, const std::string &serviceName) {
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
Smart::StatusCode ComponentCartographer::connectLocalized_robot_pose(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.localized_robot_pose.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = localized_robot_pose->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->localized_robot_pose->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentCartographer::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectLaserServiceIn(connections.laserServiceIn.serverName, connections.laserServiceIn.serviceName);
	if(status != Smart::SMART_OK) return status;
	status = connectLocalized_robot_pose(connections.localized_robot_pose.serverName, connections.localized_robot_pose.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentCartographer::startAllTasks() {
	// start task CartographerTask
	if(connections.cartographerTask.scheduler != "DEFAULT") {
		ACE_Sched_Params cartographerTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.cartographerTask.scheduler == "FIFO") {
			cartographerTask_SchedParams.policy(ACE_SCHED_FIFO);
			cartographerTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.cartographerTask.scheduler == "RR") {
			cartographerTask_SchedParams.policy(ACE_SCHED_RR);
			cartographerTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		cartographerTask->start(cartographerTask_SchedParams, connections.cartographerTask.cpuAffinity);
	} else {
		cartographerTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentCartographer::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentCartographer::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "laserServiceIn") return laserServiceInInputTaskTrigger;
	
	return NULL;
}


void ComponentCartographer::init(int argc, char *argv[])
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
		
		ComponentCartographerPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentCartographerAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentCartographer is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		gridMapPushServiceOut = portFactoryRegistry[connections.gridMapPushServiceOut.roboticMiddleware]->createGridMapPushServiceOut(connections.gridMapPushServiceOut.serviceName);
		gridMapPushServiceOutWrapper = new GridMapPushServiceOutWrapper(gridMapPushServiceOut);
		
		// create client ports
		laserServiceIn = portFactoryRegistry[connections.laserServiceIn.roboticMiddleware]->createLaserServiceIn();
		localized_robot_pose = portFactoryRegistry[connections.localized_robot_pose.roboticMiddleware]->createLocalized_robot_pose();
		localized_robot_poseWrapper = new Localized_robot_poseWrapper(localized_robot_pose);
		
		// create InputTaskTriggers and UpcallManagers
		laserServiceInInputCollector = new LaserServiceInInputCollector(laserServiceIn);
		laserServiceInInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommMobileLaserScan>(laserServiceInInputCollector);
		laserServiceInUpcallManager = new LaserServiceInUpcallManager(laserServiceInInputCollector);
		
		// create input-handler
		
		// create request-handlers
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateSlave = new SmartACE::StateSlave(component, stateChangeHandler);
		if (stateSlave->defineStates("Active" ,"active") != Smart::SMART_OK) std::cerr << "ERROR: defining state combinaion Active.active" << std::endl;
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
		if(connections.localized_robot_pose.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::SendClient<CommBasicObjects::CommBasePositionUpdate>*>(localized_robot_pose)->add(wiringSlave, connections.localized_robot_pose.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task CartographerTask
		cartographerTask = new CartographerTask(component);
		// configure input-links
		// configure task-trigger (if task is configurable)
		if(connections.cartographerTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.cartographerTask.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(cartographerTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				cartographerTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task CartographerTask" << std::endl;
			}
		} else if(connections.cartographerTask.trigger == "DataTriggered") {
			cartographerTaskTrigger = getInputTaskTriggerFromString(connections.cartographerTask.inPortRef);
			if(cartographerTaskTrigger != NULL) {
				cartographerTaskTrigger->attach(cartographerTask, connections.cartographerTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.cartographerTask.inPortRef << " as activation source for Task CartographerTask" << std::endl;
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
void ComponentCartographer::run()
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
void ComponentCartographer::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	if(cartographerTaskTrigger != NULL){
		cartographerTaskTrigger->detach(cartographerTask);
		delete cartographerTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers
	delete laserServiceInInputTaskTrigger;
	delete laserServiceInUpcallManager;
	delete laserServiceInInputCollector;

	// destroy client ports
	delete laserServiceIn;
	delete localized_robot_poseWrapper;
	delete localized_robot_pose;

	// destroy server ports
	delete gridMapPushServiceOutWrapper;
	delete gridMapPushServiceOut;
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
	
}

void ComponentCartographer::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentCartographer.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentCartographer.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentCartographer.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for client laserServiceIn
		parameter.getBoolean("laserServiceIn", "initialConnect", connections.laserServiceIn.initialConnect);
		parameter.getString("laserServiceIn", "serviceName", connections.laserServiceIn.serviceName);
		parameter.getString("laserServiceIn", "serverName", connections.laserServiceIn.serverName);
		parameter.getString("laserServiceIn", "wiringName", connections.laserServiceIn.wiringName);
		parameter.getInteger("laserServiceIn", "interval", connections.laserServiceIn.interval);
		if(parameter.checkIfParameterExists("laserServiceIn", "roboticMiddleware")) {
			parameter.getString("laserServiceIn", "roboticMiddleware", connections.laserServiceIn.roboticMiddleware);
		}
		// load parameters for client localized_robot_pose
		parameter.getBoolean("localized_robot_pose", "initialConnect", connections.localized_robot_pose.initialConnect);
		parameter.getString("localized_robot_pose", "serviceName", connections.localized_robot_pose.serviceName);
		parameter.getString("localized_robot_pose", "serverName", connections.localized_robot_pose.serverName);
		parameter.getString("localized_robot_pose", "wiringName", connections.localized_robot_pose.wiringName);
		if(parameter.checkIfParameterExists("localized_robot_pose", "roboticMiddleware")) {
			parameter.getString("localized_robot_pose", "roboticMiddleware", connections.localized_robot_pose.roboticMiddleware);
		}
		
		// load parameters for server gridMapPushServiceOut
		parameter.getString("gridMapPushServiceOut", "serviceName", connections.gridMapPushServiceOut.serviceName);
		if(parameter.checkIfParameterExists("gridMapPushServiceOut", "roboticMiddleware")) {
			parameter.getString("gridMapPushServiceOut", "roboticMiddleware", connections.gridMapPushServiceOut.roboticMiddleware);
		}
		
		// load parameters for task CartographerTask
		parameter.getDouble("CartographerTask", "minActFreqHz", connections.cartographerTask.minActFreq);
		parameter.getDouble("CartographerTask", "maxActFreqHz", connections.cartographerTask.maxActFreq);
		parameter.getString("CartographerTask", "triggerType", connections.cartographerTask.trigger);
		if(connections.cartographerTask.trigger == "PeriodicTimer") {
			parameter.getDouble("CartographerTask", "periodicActFreqHz", connections.cartographerTask.periodicActFreq);
		} else if(connections.cartographerTask.trigger == "DataTriggered") {
			parameter.getString("CartographerTask", "inPortRef", connections.cartographerTask.inPortRef);
			parameter.getInteger("CartographerTask", "prescale", connections.cartographerTask.prescale);
		}
		if(parameter.checkIfParameterExists("CartographerTask", "scheduler")) {
			parameter.getString("CartographerTask", "scheduler", connections.cartographerTask.scheduler);
		}
		if(parameter.checkIfParameterExists("CartographerTask", "priority")) {
			parameter.getInteger("CartographerTask", "priority", connections.cartographerTask.priority);
		}
		if(parameter.checkIfParameterExists("CartographerTask", "cpuAffinity")) {
			parameter.getInteger("CartographerTask", "cpuAffinity", connections.cartographerTask.cpuAffinity);
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
