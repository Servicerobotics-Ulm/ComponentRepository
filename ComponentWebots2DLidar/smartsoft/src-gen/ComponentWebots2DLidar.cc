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
#include "ComponentWebots2DLidar.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentWebots2DLidarAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentWebots2DLidar* ComponentWebots2DLidar::_componentWebots2DLidar = 0;

// constructor
ComponentWebots2DLidar::ComponentWebots2DLidar()
{
	std::cout << "constructor of ComponentWebots2DLidar\n";
	
	// set all pointer members to NULL
	baseStateServiceIn = NULL;
	baseStateServiceInInputTaskTrigger = NULL;
	baseStateServiceInUpcallManager = NULL;
	baseStateServiceInInputCollector = NULL;
	//coordinationPort = NULL;
	//coordinationPort = NULL;
	laserQueryServiceAnsw = NULL;
	laserQueryServiceAnswInputTaskTrigger = NULL;
	laserQueryServiceAnswHandler = NULL;
	laserServiceOut = NULL;
	laserServiceOutWrapper = NULL;
	laserTask = NULL;
	laserTaskTrigger = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentWebots2DLidar";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.laserQueryServiceAnsw.serviceName = "LaserQueryServiceAnsw";
	connections.laserQueryServiceAnsw.roboticMiddleware = "ACE_SmartSoft";
	connections.laserServiceOut.serviceName = "LaserServiceOut";
	connections.laserServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.baseStateServiceIn.initialConnect = false;
	connections.baseStateServiceIn.wiringName = "BaseStateServiceIn";
	connections.baseStateServiceIn.serverName = "unknown";
	connections.baseStateServiceIn.serviceName = "unknown";
	connections.baseStateServiceIn.interval = 1;
	connections.baseStateServiceIn.roboticMiddleware = "ACE_SmartSoft";
	connections.laserTask.minActFreq = 33.0;
	connections.laserTask.maxActFreq = 40.0;
	// scheduling default parameters
	connections.laserTask.scheduler = "DEFAULT";
	connections.laserTask.priority = -1;
	connections.laserTask.cpuAffinity = -1;
	
	// initialize members of ComponentWebots2DLidarROS1InterfacesExtension
	
	// initialize members of ComponentWebots2DLidarROSExtension
	
	// initialize members of ComponentWebots2DLidarRestInterfacesExtension
	
	// initialize members of OpcUaBackendComponentGeneratorExtension
	
	// initialize members of PlainOpcUaComponentWebots2DLidarExtension
	
}

void ComponentWebots2DLidar::addPortFactory(const std::string &name, ComponentWebots2DLidarPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentWebots2DLidar::addExtension(ComponentWebots2DLidarExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentWebots2DLidar::getComponentImpl()
{
	return dynamic_cast<ComponentWebots2DLidarAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentWebots2DLidar::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentWebots2DLidar::connectBaseStateServiceIn(const std::string &serverName, const std::string &serviceName) {
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


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentWebots2DLidar::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectBaseStateServiceIn(connections.baseStateServiceIn.serverName, connections.baseStateServiceIn.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentWebots2DLidar::startAllTasks() {
	// start task LaserTask
	if(connections.laserTask.scheduler != "DEFAULT") {
		ACE_Sched_Params laserTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.laserTask.scheduler == "FIFO") {
			laserTask_SchedParams.policy(ACE_SCHED_FIFO);
			laserTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.laserTask.scheduler == "RR") {
			laserTask_SchedParams.policy(ACE_SCHED_RR);
			laserTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		laserTask->start(laserTask_SchedParams, connections.laserTask.cpuAffinity);
	} else {
		laserTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentWebots2DLidar::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentWebots2DLidar::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "BaseStateServiceIn") return baseStateServiceInInputTaskTrigger;
	
	return NULL;
}


void ComponentWebots2DLidar::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		// print out the actual parameters which are used to initialize the component
		std::cout << " \nComponentDefinition Initial-Parameters:\n" << COMP->getParameters() << std::endl;
		
		// initializations of ComponentWebots2DLidarROS1InterfacesExtension
		
		// initializations of ComponentWebots2DLidarROSExtension
		
		// initializations of ComponentWebots2DLidarRestInterfacesExtension
		
		// initializations of OpcUaBackendComponentGeneratorExtension
		
		// initializations of PlainOpcUaComponentWebots2DLidarExtension
		
		
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
		
		ComponentWebots2DLidarPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentWebots2DLidarAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentWebots2DLidar is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		laserQueryServiceAnsw = portFactoryRegistry[connections.laserQueryServiceAnsw.roboticMiddleware]->createLaserQueryServiceAnsw(connections.laserQueryServiceAnsw.serviceName);
		laserQueryServiceAnswInputTaskTrigger = new Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, CommBasicObjects::CommMobileLaserScan>(laserQueryServiceAnsw);
		laserServiceOut = portFactoryRegistry[connections.laserServiceOut.roboticMiddleware]->createLaserServiceOut(connections.laserServiceOut.serviceName);
		laserServiceOutWrapper = new LaserServiceOutWrapper(laserServiceOut);
		
		// create client ports
		baseStateServiceIn = portFactoryRegistry[connections.baseStateServiceIn.roboticMiddleware]->createBaseStateServiceIn();
		
		// create InputTaskTriggers and UpcallManagers
		baseStateServiceInInputCollector = new BaseStateServiceInInputCollector(baseStateServiceIn);
		baseStateServiceInInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommBaseState>(baseStateServiceInInputCollector);
		baseStateServiceInUpcallManager = new BaseStateServiceInUpcallManager(baseStateServiceInInputCollector);
		
		// create input-handler
		
		// create request-handlers
		laserQueryServiceAnswHandler = new LaserQueryServiceAnswHandler(laserQueryServiceAnsw);
		
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
		if(connections.baseStateServiceIn.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<CommBasicObjects::CommBaseState>*>(baseStateServiceIn)->add(wiringSlave, connections.baseStateServiceIn.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task LaserTask
		laserTask = new LaserTask(component);
		// configure input-links
		baseStateServiceInUpcallManager->attach(laserTask);
		// configure task-trigger (if task is configurable)
		if(connections.laserTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.laserTask.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(laserTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				laserTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task LaserTask" << std::endl;
			}
		} else if(connections.laserTask.trigger == "DataTriggered") {
			laserTaskTrigger = getInputTaskTriggerFromString(connections.laserTask.inPortRef);
			if(laserTaskTrigger != NULL) {
				laserTaskTrigger->attach(laserTask, connections.laserTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.laserTask.inPortRef << " as activation source for Task LaserTask" << std::endl;
			}
		} 
		
		
		// link observers with subjects
		laserTask->attach_interaction_observer(laserQueryServiceAnswHandler);
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std exception" << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}

// run the component
void ComponentWebots2DLidar::run()
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
void ComponentWebots2DLidar::fini()
{
	// unlink all observers
	laserTask->detach_interaction_observer(laserQueryServiceAnswHandler);
	
	// destroy all task instances
	// unlink all UpcallManagers
	baseStateServiceInUpcallManager->detach(laserTask);
	// unlink the TaskTrigger
	if(laserTaskTrigger != NULL){
		laserTaskTrigger->detach(laserTask);
		delete laserTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers
	delete baseStateServiceInInputTaskTrigger;
	delete baseStateServiceInUpcallManager;
	delete baseStateServiceInInputCollector;

	// destroy client ports
	delete baseStateServiceIn;

	// destroy server ports
	delete laserQueryServiceAnsw;
	delete laserQueryServiceAnswInputTaskTrigger;
	delete laserServiceOutWrapper;
	delete laserServiceOut;
	// destroy event-test handlers (if needed)
	
	// destroy request-handlers
	delete laserQueryServiceAnswHandler;
	
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
	
	// destruction of ComponentWebots2DLidarROS1InterfacesExtension
	
	// destruction of ComponentWebots2DLidarROSExtension
	
	// destruction of ComponentWebots2DLidarRestInterfacesExtension
	
	// destruction of OpcUaBackendComponentGeneratorExtension
	
	// destruction of PlainOpcUaComponentWebots2DLidarExtension
	
}

void ComponentWebots2DLidar::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentWebots2DLidar.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentWebots2DLidar.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentWebots2DLidar.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for server LaserQueryServiceAnsw
		parameter.getString("LaserQueryServiceAnsw", "serviceName", connections.laserQueryServiceAnsw.serviceName);
		if(parameter.checkIfParameterExists("LaserQueryServiceAnsw", "roboticMiddleware")) {
			parameter.getString("LaserQueryServiceAnsw", "roboticMiddleware", connections.laserQueryServiceAnsw.roboticMiddleware);
		}
		// load parameters for server LaserServiceOut
		parameter.getString("LaserServiceOut", "serviceName", connections.laserServiceOut.serviceName);
		if(parameter.checkIfParameterExists("LaserServiceOut", "roboticMiddleware")) {
			parameter.getString("LaserServiceOut", "roboticMiddleware", connections.laserServiceOut.roboticMiddleware);
		}
		
		// load parameters for task LaserTask
		parameter.getDouble("LaserTask", "minActFreqHz", connections.laserTask.minActFreq);
		parameter.getDouble("LaserTask", "maxActFreqHz", connections.laserTask.maxActFreq);
		parameter.getString("LaserTask", "triggerType", connections.laserTask.trigger);
		if(connections.laserTask.trigger == "PeriodicTimer") {
			parameter.getDouble("LaserTask", "periodicActFreqHz", connections.laserTask.periodicActFreq);
		} else if(connections.laserTask.trigger == "DataTriggered") {
			parameter.getString("LaserTask", "inPortRef", connections.laserTask.inPortRef);
			parameter.getInteger("LaserTask", "prescale", connections.laserTask.prescale);
		}
		if(parameter.checkIfParameterExists("LaserTask", "scheduler")) {
			parameter.getString("LaserTask", "scheduler", connections.laserTask.scheduler);
		}
		if(parameter.checkIfParameterExists("LaserTask", "priority")) {
			parameter.getInteger("LaserTask", "priority", connections.laserTask.priority);
		}
		if(parameter.checkIfParameterExists("LaserTask", "cpuAffinity")) {
			parameter.getInteger("LaserTask", "cpuAffinity", connections.laserTask.cpuAffinity);
		}
		
		// load parameters for ComponentWebots2DLidarROS1InterfacesExtension
		
		// load parameters for ComponentWebots2DLidarROSExtension
		
		// load parameters for ComponentWebots2DLidarRestInterfacesExtension
		
		// load parameters for OpcUaBackendComponentGeneratorExtension
		
		// load parameters for PlainOpcUaComponentWebots2DLidarExtension
		
		
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