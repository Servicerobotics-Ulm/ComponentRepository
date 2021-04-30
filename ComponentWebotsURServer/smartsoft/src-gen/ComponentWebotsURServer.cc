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
#include "ComponentWebotsURServer.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentWebotsURServerAcePortFactory.hh"

#include "IoEventServerEventTestHandler.hh"
#include "ManipulatorEventServiceOutEventTestHandler.hh"

// initialize static singleton pointer to zero
ComponentWebotsURServer* ComponentWebotsURServer::_componentWebotsURServer = 0;

// constructor
ComponentWebotsURServer::ComponentWebotsURServer()
{
	std::cout << "constructor of ComponentWebotsURServer\n";
	
	// set all pointer members to NULL
	//coordinationPort = NULL;
	//coordinationPort = NULL;
	ioQueryServerHandler = NULL;
	poseQueryServerHandler = NULL;
	poseUpdateActivity = NULL;
	poseUpdateActivityTrigger = NULL;
	trajectorySendServerHandler = NULL;
	baseStateServiceIn = NULL;
	baseStateServiceInInputTaskTrigger = NULL;
	baseStateServiceInUpcallManager = NULL;
	baseStateServiceInInputCollector = NULL;
	ioEventServer = NULL;
	ioEventServerWrapper = NULL;
	ioEventServerEventTestHandler = nullptr; 
	ioQueryServer = NULL;
	ioQueryServerInputTaskTrigger = NULL;
	manipulatorEventServiceOut = NULL;
	manipulatorEventServiceOutWrapper = NULL;
	manipulatorEventServiceOutEventTestHandler = nullptr; 
	posePushServer = NULL;
	posePushServerWrapper = NULL;
	poseQueryServer = NULL;
	poseQueryServerInputTaskTrigger = NULL;
	programQuery = NULL;
	programQueryInputTaskTrigger = NULL;
	programQueryHandler = NULL;
	trajectorySendServer = NULL;
	trajectorySendServerInputTaskTrigger = NULL;
	trajectorySendServerUpcallManager = NULL;
	trajectorySendServerInputCollector = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentWebotsURServer";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.ioEventServer.serviceName = "ioEventServer";
	connections.ioEventServer.roboticMiddleware = "ACE_SmartSoft";
	connections.ioQueryServer.serviceName = "ioQueryServer";
	connections.ioQueryServer.roboticMiddleware = "ACE_SmartSoft";
	connections.manipulatorEventServiceOut.serviceName = "manipulatorEventServiceOut";
	connections.manipulatorEventServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.posePushServer.serviceName = "posePushServer";
	connections.posePushServer.roboticMiddleware = "ACE_SmartSoft";
	connections.poseQueryServer.serviceName = "poseQueryServer";
	connections.poseQueryServer.roboticMiddleware = "ACE_SmartSoft";
	connections.programQuery.serviceName = "programQuery";
	connections.programQuery.roboticMiddleware = "ACE_SmartSoft";
	connections.trajectorySendServer.serviceName = "trajectorySendServer";
	connections.trajectorySendServer.roboticMiddleware = "ACE_SmartSoft";
	connections.baseStateServiceIn.initialConnect = false;
	connections.baseStateServiceIn.wiringName = "baseStateServiceIn";
	connections.baseStateServiceIn.serverName = "unknown";
	connections.baseStateServiceIn.serviceName = "unknown";
	connections.baseStateServiceIn.interval = 1;
	connections.baseStateServiceIn.roboticMiddleware = "ACE_SmartSoft";
	connections.poseUpdateActivity.minActFreq = 0.0;
	connections.poseUpdateActivity.maxActFreq = 0.0;
	// scheduling default parameters
	connections.poseUpdateActivity.scheduler = "DEFAULT";
	connections.poseUpdateActivity.priority = -1;
	connections.poseUpdateActivity.cpuAffinity = -1;
	connections.trajectorySendServerHandler.prescale = 1;
	
}

void ComponentWebotsURServer::addPortFactory(const std::string &name, ComponentWebotsURServerPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentWebotsURServer::addExtension(ComponentWebotsURServerExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentWebotsURServer::getComponentImpl()
{
	return dynamic_cast<ComponentWebotsURServerAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentWebotsURServer::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentWebotsURServer::connectBaseStateServiceIn(const std::string &serverName, const std::string &serviceName) {
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
Smart::StatusCode ComponentWebotsURServer::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectBaseStateServiceIn(connections.baseStateServiceIn.serverName, connections.baseStateServiceIn.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentWebotsURServer::startAllTasks() {
	// start task PoseUpdateActivity
	if(connections.poseUpdateActivity.scheduler != "DEFAULT") {
		ACE_Sched_Params poseUpdateActivity_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.poseUpdateActivity.scheduler == "FIFO") {
			poseUpdateActivity_SchedParams.policy(ACE_SCHED_FIFO);
			poseUpdateActivity_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.poseUpdateActivity.scheduler == "RR") {
			poseUpdateActivity_SchedParams.policy(ACE_SCHED_RR);
			poseUpdateActivity_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		poseUpdateActivity->start(poseUpdateActivity_SchedParams, connections.poseUpdateActivity.cpuAffinity);
	} else {
		poseUpdateActivity->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentWebotsURServer::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentWebotsURServer::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "baseStateServiceIn") return baseStateServiceInInputTaskTrigger;
	if(client == "trajectorySendServer") return trajectorySendServerInputTaskTrigger;
	
	return NULL;
}


void ComponentWebotsURServer::init(int argc, char *argv[])
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
		
		ComponentWebotsURServerPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentWebotsURServerAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentWebotsURServer is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		ioEventServerEventTestHandler = std::make_shared<IoEventServerEventTestHandler>();
		manipulatorEventServiceOutEventTestHandler = std::make_shared<ManipulatorEventServiceOutEventTestHandler>();
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		ioEventServerEventTestHandler = std::make_shared<IoEventServerEventTestHandler>();
		ioEventServer = portFactoryRegistry[connections.ioEventServer.roboticMiddleware]->createIoEventServer(connections.ioEventServer.serviceName, ioEventServerEventTestHandler);
		ioEventServerWrapper = new IoEventServerWrapper(ioEventServer);
		ioQueryServer = portFactoryRegistry[connections.ioQueryServer.roboticMiddleware]->createIoQueryServer(connections.ioQueryServer.serviceName);
		ioQueryServerInputTaskTrigger = new Smart::QueryServerTaskTrigger<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues>(ioQueryServer);
		manipulatorEventServiceOutEventTestHandler = std::make_shared<ManipulatorEventServiceOutEventTestHandler>();
		manipulatorEventServiceOut = portFactoryRegistry[connections.manipulatorEventServiceOut.roboticMiddleware]->createManipulatorEventServiceOut(connections.manipulatorEventServiceOut.serviceName, manipulatorEventServiceOutEventTestHandler);
		manipulatorEventServiceOutWrapper = new ManipulatorEventServiceOutWrapper(manipulatorEventServiceOut);
		posePushServer = portFactoryRegistry[connections.posePushServer.roboticMiddleware]->createPosePushServer(connections.posePushServer.serviceName);
		posePushServerWrapper = new PosePushServerWrapper(posePushServer);
		poseQueryServer = portFactoryRegistry[connections.poseQueryServer.roboticMiddleware]->createPoseQueryServer(connections.poseQueryServer.serviceName);
		poseQueryServerInputTaskTrigger = new Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState>(poseQueryServer);
		programQuery = portFactoryRegistry[connections.programQuery.roboticMiddleware]->createProgramQuery(connections.programQuery.serviceName);
		programQueryInputTaskTrigger = new Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorPrograms>(programQuery);
		trajectorySendServer = portFactoryRegistry[connections.trajectorySendServer.roboticMiddleware]->createTrajectorySendServer(connections.trajectorySendServer.serviceName);
		
		// create client ports
		baseStateServiceIn = portFactoryRegistry[connections.baseStateServiceIn.roboticMiddleware]->createBaseStateServiceIn();
		
		// create InputTaskTriggers and UpcallManagers
		baseStateServiceInInputCollector = new BaseStateServiceInInputCollector(baseStateServiceIn);
		baseStateServiceInInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommBaseState>(baseStateServiceInInputCollector);
		baseStateServiceInUpcallManager = new BaseStateServiceInUpcallManager(baseStateServiceInInputCollector);
		trajectorySendServerInputCollector = new TrajectorySendServerInputCollector(trajectorySendServer);
		trajectorySendServerInputTaskTrigger = new Smart::InputTaskTrigger<CommManipulatorObjects::CommManipulatorTrajectory>(trajectorySendServerInputCollector);
		trajectorySendServerUpcallManager = new TrajectorySendServerUpcallManager(trajectorySendServerInputCollector);
		
		// create input-handler
		trajectorySendServerHandler = new TrajectorySendServerHandler(trajectorySendServer, connections.trajectorySendServerHandler.prescale);
		
		// create request-handlers
		ioQueryServerHandler = new IoQueryServerHandler(ioQueryServer);
		poseQueryServerHandler = new PoseQueryServerHandler(poseQueryServer);
		programQueryHandler = new ProgramQueryHandler(programQuery);
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateSlave = new SmartACE::StateSlave(component, stateChangeHandler);
		if (stateSlave->defineStates("Trajectory" ,"trajectory") != Smart::SMART_OK) std::cerr << "ERROR: defining state combinaion Trajectory.trajectory" << std::endl;
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
		
		
		// create Task PoseUpdateActivity
		poseUpdateActivity = new PoseUpdateActivity(component);
		// configure input-links
		baseStateServiceInUpcallManager->attach(poseUpdateActivity);
		// configure task-trigger (if task is configurable)
		if(connections.poseUpdateActivity.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.poseUpdateActivity.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(poseUpdateActivity);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				poseUpdateActivityTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task PoseUpdateActivity" << std::endl;
			}
		} else if(connections.poseUpdateActivity.trigger == "DataTriggered") {
			poseUpdateActivityTrigger = getInputTaskTriggerFromString(connections.poseUpdateActivity.inPortRef);
			if(poseUpdateActivityTrigger != NULL) {
				poseUpdateActivityTrigger->attach(poseUpdateActivity, connections.poseUpdateActivity.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.poseUpdateActivity.inPortRef << " as activation source for Task PoseUpdateActivity" << std::endl;
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
void ComponentWebotsURServer::run()
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
void ComponentWebotsURServer::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	baseStateServiceInUpcallManager->detach(poseUpdateActivity);
	// unlink the TaskTrigger
	if(poseUpdateActivityTrigger != NULL){
		poseUpdateActivityTrigger->detach(poseUpdateActivity);
		delete poseUpdateActivity;
	}

	// destroy all input-handler
	delete trajectorySendServerHandler;

	// destroy InputTaskTriggers and UpcallManagers
	delete baseStateServiceInInputTaskTrigger;
	delete baseStateServiceInUpcallManager;
	delete baseStateServiceInInputCollector;
	delete trajectorySendServerInputTaskTrigger;
	delete trajectorySendServerUpcallManager;
	delete trajectorySendServerInputCollector;

	// destroy client ports
	delete baseStateServiceIn;

	// destroy server ports
	delete ioEventServerWrapper;
	delete ioEventServer;
	delete ioQueryServer;
	delete ioQueryServerInputTaskTrigger;
	delete manipulatorEventServiceOutWrapper;
	delete manipulatorEventServiceOut;
	delete posePushServerWrapper;
	delete posePushServer;
	delete poseQueryServer;
	delete poseQueryServerInputTaskTrigger;
	delete programQuery;
	delete programQueryInputTaskTrigger;
	delete trajectorySendServer;
	// destroy event-test handlers (if needed)
	ioEventServerEventTestHandler;
	manipulatorEventServiceOutEventTestHandler;
	
	// destroy request-handlers
	delete ioQueryServerHandler;
	delete poseQueryServerHandler;
	delete programQueryHandler;
	
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

void ComponentWebotsURServer::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentWebotsURServer.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentWebotsURServer.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentWebotsURServer.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for client baseStateServiceIn
		parameter.getBoolean("baseStateServiceIn", "initialConnect", connections.baseStateServiceIn.initialConnect);
		parameter.getString("baseStateServiceIn", "serviceName", connections.baseStateServiceIn.serviceName);
		parameter.getString("baseStateServiceIn", "serverName", connections.baseStateServiceIn.serverName);
		parameter.getString("baseStateServiceIn", "wiringName", connections.baseStateServiceIn.wiringName);
		parameter.getInteger("baseStateServiceIn", "interval", connections.baseStateServiceIn.interval);
		if(parameter.checkIfParameterExists("baseStateServiceIn", "roboticMiddleware")) {
			parameter.getString("baseStateServiceIn", "roboticMiddleware", connections.baseStateServiceIn.roboticMiddleware);
		}
		
		// load parameters for server ioEventServer
		parameter.getString("ioEventServer", "serviceName", connections.ioEventServer.serviceName);
		if(parameter.checkIfParameterExists("ioEventServer", "roboticMiddleware")) {
			parameter.getString("ioEventServer", "roboticMiddleware", connections.ioEventServer.roboticMiddleware);
		}
		// load parameters for server ioQueryServer
		parameter.getString("ioQueryServer", "serviceName", connections.ioQueryServer.serviceName);
		if(parameter.checkIfParameterExists("ioQueryServer", "roboticMiddleware")) {
			parameter.getString("ioQueryServer", "roboticMiddleware", connections.ioQueryServer.roboticMiddleware);
		}
		// load parameters for server manipulatorEventServiceOut
		parameter.getString("manipulatorEventServiceOut", "serviceName", connections.manipulatorEventServiceOut.serviceName);
		if(parameter.checkIfParameterExists("manipulatorEventServiceOut", "roboticMiddleware")) {
			parameter.getString("manipulatorEventServiceOut", "roboticMiddleware", connections.manipulatorEventServiceOut.roboticMiddleware);
		}
		// load parameters for server posePushServer
		parameter.getString("posePushServer", "serviceName", connections.posePushServer.serviceName);
		if(parameter.checkIfParameterExists("posePushServer", "roboticMiddleware")) {
			parameter.getString("posePushServer", "roboticMiddleware", connections.posePushServer.roboticMiddleware);
		}
		// load parameters for server poseQueryServer
		parameter.getString("poseQueryServer", "serviceName", connections.poseQueryServer.serviceName);
		if(parameter.checkIfParameterExists("poseQueryServer", "roboticMiddleware")) {
			parameter.getString("poseQueryServer", "roboticMiddleware", connections.poseQueryServer.roboticMiddleware);
		}
		// load parameters for server programQuery
		parameter.getString("programQuery", "serviceName", connections.programQuery.serviceName);
		if(parameter.checkIfParameterExists("programQuery", "roboticMiddleware")) {
			parameter.getString("programQuery", "roboticMiddleware", connections.programQuery.roboticMiddleware);
		}
		// load parameters for server trajectorySendServer
		parameter.getString("trajectorySendServer", "serviceName", connections.trajectorySendServer.serviceName);
		if(parameter.checkIfParameterExists("trajectorySendServer", "roboticMiddleware")) {
			parameter.getString("trajectorySendServer", "roboticMiddleware", connections.trajectorySendServer.roboticMiddleware);
		}
		
		// load parameters for task PoseUpdateActivity
		parameter.getDouble("PoseUpdateActivity", "minActFreqHz", connections.poseUpdateActivity.minActFreq);
		parameter.getDouble("PoseUpdateActivity", "maxActFreqHz", connections.poseUpdateActivity.maxActFreq);
		parameter.getString("PoseUpdateActivity", "triggerType", connections.poseUpdateActivity.trigger);
		if(connections.poseUpdateActivity.trigger == "PeriodicTimer") {
			parameter.getDouble("PoseUpdateActivity", "periodicActFreqHz", connections.poseUpdateActivity.periodicActFreq);
		} else if(connections.poseUpdateActivity.trigger == "DataTriggered") {
			parameter.getString("PoseUpdateActivity", "inPortRef", connections.poseUpdateActivity.inPortRef);
			parameter.getInteger("PoseUpdateActivity", "prescale", connections.poseUpdateActivity.prescale);
		}
		if(parameter.checkIfParameterExists("PoseUpdateActivity", "scheduler")) {
			parameter.getString("PoseUpdateActivity", "scheduler", connections.poseUpdateActivity.scheduler);
		}
		if(parameter.checkIfParameterExists("PoseUpdateActivity", "priority")) {
			parameter.getInteger("PoseUpdateActivity", "priority", connections.poseUpdateActivity.priority);
		}
		if(parameter.checkIfParameterExists("PoseUpdateActivity", "cpuAffinity")) {
			parameter.getInteger("PoseUpdateActivity", "cpuAffinity", connections.poseUpdateActivity.cpuAffinity);
		}
		if(parameter.checkIfParameterExists("TrajectorySendServerHandler", "prescale")) {
			parameter.getInteger("TrajectorySendServerHandler", "prescale", connections.trajectorySendServerHandler.prescale);
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