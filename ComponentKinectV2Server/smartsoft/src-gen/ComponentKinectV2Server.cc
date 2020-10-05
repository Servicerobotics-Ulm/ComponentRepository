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
#include "ComponentKinectV2Server.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentKinectV2ServerAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentKinectV2Server* ComponentKinectV2Server::_componentKinectV2Server = 0;

// constructor
ComponentKinectV2Server::ComponentKinectV2Server()
{
	std::cout << "constructor of ComponentKinectV2Server\n";
	
	// set all pointer members to NULL
	colorImageQueryHandler = NULL;
	//componentKinectV2ServerParams = NULL;
	//coordinationPort = NULL;
	imageQueryHandler = NULL;
	imageTask = NULL;
	imageTaskTrigger = NULL;
	basePushTimedClient = NULL;
	basePushTimedClientInputTaskTrigger = NULL;
	basePushTimedClientUpcallManager = NULL;
	colorImagePushNewestServer = NULL;
	colorImageQueryServer = NULL;
	colorImageQueryServerInputTaskTrigger = NULL;
	depthImagePushServiceOut = NULL;
	imagePushNewestServer = NULL;
	imageQueryV2Server = NULL;
	imageQueryV2ServerInputTaskTrigger = NULL;
	ptuPosePushNewestClient = NULL;
	ptuPosePushNewestClientInputTaskTrigger = NULL;
	ptuPosePushNewestClientUpcallManager = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentKinectV2Server";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.colorImagePushNewestServer.serviceName = "colorImagePushNewestServer";
	connections.colorImagePushNewestServer.roboticMiddleware = "ACE_SmartSoft";
	connections.colorImageQueryServer.serviceName = "colorImageQueryServer";
	connections.colorImageQueryServer.roboticMiddleware = "ACE_SmartSoft";
	connections.depthImagePushServiceOut.serviceName = "depthImagePushServiceOut";
	connections.depthImagePushServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.imagePushNewestServer.serviceName = "imagePushNewestServer";
	connections.imagePushNewestServer.roboticMiddleware = "ACE_SmartSoft";
	connections.imageQueryV2Server.serviceName = "imageQueryV2Server";
	connections.imageQueryV2Server.roboticMiddleware = "ACE_SmartSoft";
	connections.basePushTimedClient.initialConnect = false;
	connections.basePushTimedClient.wiringName = "basePushTimedClient";
	connections.basePushTimedClient.serverName = "unknown";
	connections.basePushTimedClient.serviceName = "unknown";
	connections.basePushTimedClient.interval = 1;
	connections.basePushTimedClient.roboticMiddleware = "ACE_SmartSoft";
	connections.ptuPosePushNewestClient.initialConnect = false;
	connections.ptuPosePushNewestClient.wiringName = "ptuPosePushNewestClient";
	connections.ptuPosePushNewestClient.serverName = "unknown";
	connections.ptuPosePushNewestClient.serviceName = "unknown";
	connections.ptuPosePushNewestClient.interval = 1;
	connections.ptuPosePushNewestClient.roboticMiddleware = "ACE_SmartSoft";
	connections.imageTask.minActFreq = 0.0;
	connections.imageTask.maxActFreq = 0.0;
	// scheduling default parameters
	connections.imageTask.scheduler = "DEFAULT";
	connections.imageTask.priority = -1;
	connections.imageTask.cpuAffinity = -1;
	
	// initialize members of ComponentKinectV2ServerROSExtension
	
	// initialize members of OpcUaBackendComponentGeneratorExtension
	
	// initialize members of PlainOpcUaComponentKinectV2ServerExtension
	
}

void ComponentKinectV2Server::addPortFactory(const std::string &name, ComponentKinectV2ServerPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentKinectV2Server::addExtension(ComponentKinectV2ServerExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentKinectV2Server::getComponentImpl()
{
	return dynamic_cast<ComponentKinectV2ServerAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentKinectV2Server::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentKinectV2Server::connectBasePushTimedClient(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.basePushTimedClient.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = basePushTimedClient->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->basePushTimedClient->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	basePushTimedClient->subscribe(connections.basePushTimedClient.interval);
	return status;
}
Smart::StatusCode ComponentKinectV2Server::connectPtuPosePushNewestClient(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.ptuPosePushNewestClient.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = ptuPosePushNewestClient->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->ptuPosePushNewestClient->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	ptuPosePushNewestClient->subscribe(connections.ptuPosePushNewestClient.interval);
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentKinectV2Server::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectBasePushTimedClient(connections.basePushTimedClient.serverName, connections.basePushTimedClient.serviceName);
	if(status != Smart::SMART_OK) return status;
	status = connectPtuPosePushNewestClient(connections.ptuPosePushNewestClient.serverName, connections.ptuPosePushNewestClient.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentKinectV2Server::startAllTasks() {
	// start task ImageTask
	if(connections.imageTask.scheduler != "DEFAULT") {
		ACE_Sched_Params imageTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.imageTask.scheduler == "FIFO") {
			imageTask_SchedParams.policy(ACE_SCHED_FIFO);
			imageTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.imageTask.scheduler == "RR") {
			imageTask_SchedParams.policy(ACE_SCHED_RR);
			imageTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		imageTask->start(imageTask_SchedParams, connections.imageTask.cpuAffinity);
	} else {
		imageTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentKinectV2Server::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentKinectV2Server::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "basePushTimedClient") return basePushTimedClientInputTaskTrigger;
	if(client == "ptuPosePushNewestClient") return ptuPosePushNewestClientInputTaskTrigger;
	
	return NULL;
}


void ComponentKinectV2Server::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		// print out the actual parameters which are used to initialize the component
		std::cout << " \nComponentDefinition Initial-Parameters:\n" << COMP->getParameters() << std::endl;
		
		// initializations of ComponentKinectV2ServerROSExtension
		
		// initializations of OpcUaBackendComponentGeneratorExtension
		
		// initializations of PlainOpcUaComponentKinectV2ServerExtension
		
		
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
		
		ComponentKinectV2ServerPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentKinectV2ServerAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentKinectV2Server is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		colorImagePushNewestServer = portFactoryRegistry[connections.colorImagePushNewestServer.roboticMiddleware]->createColorImagePushNewestServer(connections.colorImagePushNewestServer.serviceName);
		colorImageQueryServer = portFactoryRegistry[connections.colorImageQueryServer.roboticMiddleware]->createColorImageQueryServer(connections.colorImageQueryServer.serviceName);
		colorImageQueryServerInputTaskTrigger = new Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, DomainVision::CommVideoImage>(colorImageQueryServer);
		depthImagePushServiceOut = portFactoryRegistry[connections.depthImagePushServiceOut.roboticMiddleware]->createDepthImagePushServiceOut(connections.depthImagePushServiceOut.serviceName);
		imagePushNewestServer = portFactoryRegistry[connections.imagePushNewestServer.roboticMiddleware]->createImagePushNewestServer(connections.imagePushNewestServer.serviceName);
		imageQueryV2Server = portFactoryRegistry[connections.imageQueryV2Server.roboticMiddleware]->createImageQueryV2Server(connections.imageQueryV2Server.serviceName);
		imageQueryV2ServerInputTaskTrigger = new Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage>(imageQueryV2Server);
		
		// create client ports
		basePushTimedClient = portFactoryRegistry[connections.basePushTimedClient.roboticMiddleware]->createBasePushTimedClient();
		ptuPosePushNewestClient = portFactoryRegistry[connections.ptuPosePushNewestClient.roboticMiddleware]->createPtuPosePushNewestClient();
		
		// create InputTaskTriggers and UpcallManagers
		basePushTimedClientInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommBaseState>(basePushTimedClient);
		basePushTimedClientUpcallManager = new BasePushTimedClientUpcallManager(basePushTimedClient);
		ptuPosePushNewestClientInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommDevicePoseState>(ptuPosePushNewestClient);
		ptuPosePushNewestClientUpcallManager = new PtuPosePushNewestClientUpcallManager(ptuPosePushNewestClient);
		
		// create input-handler
		
		// create request-handlers
		colorImageQueryHandler = new ColorImageQueryHandler(colorImageQueryServer);
		imageQueryHandler = new ImageQueryHandler(imageQueryV2Server);
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateSlave = new SmartACE::StateSlave(component, stateChangeHandler);
		if (stateSlave->defineStates("PushImage" ,"pushimage") != Smart::SMART_OK) std::cerr << "ERROR: defining state combinaion PushImage.pushimage" << std::endl;
		if (stateSlave->defineStates("QueryImage" ,"queryimage") != Smart::SMART_OK) std::cerr << "ERROR: defining state combinaion QueryImage.queryimage" << std::endl;
		status = stateSlave->setUpInitialState(connections.component.initialComponentMode);
		if (status != Smart::SMART_OK) std::cerr << status << "; failed setting initial ComponentMode: " << connections.component.initialComponentMode << std::endl;
		// activate state slave
		status = stateSlave->activate();
		if(status != Smart::SMART_OK) std::cerr << "ERROR: activate state" << std::endl;
		
		wiringSlave = new SmartACE::WiringSlave(component);
		// add client port to wiring slave
		if(connections.basePushTimedClient.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<CommBasicObjects::CommBaseState>*>(basePushTimedClient)->add(wiringSlave, connections.basePushTimedClient.wiringName);
		}
		if(connections.ptuPosePushNewestClient.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<CommBasicObjects::CommDevicePoseState>*>(ptuPosePushNewestClient)->add(wiringSlave, connections.ptuPosePushNewestClient.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task ImageTask
		imageTask = new ImageTask(component);
		// configure input-links
		// configure task-trigger (if task is configurable)
		if(connections.imageTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.imageTask.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(imageTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				imageTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task ImageTask" << std::endl;
			}
		} else if(connections.imageTask.trigger == "DataTriggered") {
			imageTaskTrigger = getInputTaskTriggerFromString(connections.imageTask.inPortRef);
			if(imageTaskTrigger != NULL) {
				imageTaskTrigger->attach(imageTask, connections.imageTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.imageTask.inPortRef << " as activation source for Task ImageTask" << std::endl;
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
void ComponentKinectV2Server::run()
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
void ComponentKinectV2Server::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	if(imageTaskTrigger != NULL){
		imageTaskTrigger->detach(imageTask);
		delete imageTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers
	delete basePushTimedClientInputTaskTrigger;
	delete basePushTimedClientUpcallManager;
	delete ptuPosePushNewestClientInputTaskTrigger;
	delete ptuPosePushNewestClientUpcallManager;

	// destroy client ports
	delete basePushTimedClient;
	delete ptuPosePushNewestClient;

	// destroy server ports
	delete colorImagePushNewestServer;
	delete colorImageQueryServer;
	delete colorImageQueryServerInputTaskTrigger;
	delete depthImagePushServiceOut;
	delete imagePushNewestServer;
	delete imageQueryV2Server;
	delete imageQueryV2ServerInputTaskTrigger;
	// destroy event-test handlers (if needed)
	
	// destroy request-handlers
	delete colorImageQueryHandler;
	delete imageQueryHandler;
	
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
	
	// destruction of ComponentKinectV2ServerROSExtension
	
	// destruction of OpcUaBackendComponentGeneratorExtension
	
	// destruction of PlainOpcUaComponentKinectV2ServerExtension
	
}

void ComponentKinectV2Server::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentKinectV2Server.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentKinectV2Server.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentKinectV2Server.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for client basePushTimedClient
		parameter.getBoolean("basePushTimedClient", "initialConnect", connections.basePushTimedClient.initialConnect);
		parameter.getString("basePushTimedClient", "serviceName", connections.basePushTimedClient.serviceName);
		parameter.getString("basePushTimedClient", "serverName", connections.basePushTimedClient.serverName);
		parameter.getString("basePushTimedClient", "wiringName", connections.basePushTimedClient.wiringName);
		parameter.getInteger("basePushTimedClient", "interval", connections.basePushTimedClient.interval);
		if(parameter.checkIfParameterExists("basePushTimedClient", "roboticMiddleware")) {
			parameter.getString("basePushTimedClient", "roboticMiddleware", connections.basePushTimedClient.roboticMiddleware);
		}
		// load parameters for client ptuPosePushNewestClient
		parameter.getBoolean("ptuPosePushNewestClient", "initialConnect", connections.ptuPosePushNewestClient.initialConnect);
		parameter.getString("ptuPosePushNewestClient", "serviceName", connections.ptuPosePushNewestClient.serviceName);
		parameter.getString("ptuPosePushNewestClient", "serverName", connections.ptuPosePushNewestClient.serverName);
		parameter.getString("ptuPosePushNewestClient", "wiringName", connections.ptuPosePushNewestClient.wiringName);
		parameter.getInteger("ptuPosePushNewestClient", "interval", connections.ptuPosePushNewestClient.interval);
		if(parameter.checkIfParameterExists("ptuPosePushNewestClient", "roboticMiddleware")) {
			parameter.getString("ptuPosePushNewestClient", "roboticMiddleware", connections.ptuPosePushNewestClient.roboticMiddleware);
		}
		
		// load parameters for server colorImagePushNewestServer
		parameter.getString("colorImagePushNewestServer", "serviceName", connections.colorImagePushNewestServer.serviceName);
		if(parameter.checkIfParameterExists("colorImagePushNewestServer", "roboticMiddleware")) {
			parameter.getString("colorImagePushNewestServer", "roboticMiddleware", connections.colorImagePushNewestServer.roboticMiddleware);
		}
		// load parameters for server colorImageQueryServer
		parameter.getString("colorImageQueryServer", "serviceName", connections.colorImageQueryServer.serviceName);
		if(parameter.checkIfParameterExists("colorImageQueryServer", "roboticMiddleware")) {
			parameter.getString("colorImageQueryServer", "roboticMiddleware", connections.colorImageQueryServer.roboticMiddleware);
		}
		// load parameters for server depthImagePushServiceOut
		parameter.getString("depthImagePushServiceOut", "serviceName", connections.depthImagePushServiceOut.serviceName);
		if(parameter.checkIfParameterExists("depthImagePushServiceOut", "roboticMiddleware")) {
			parameter.getString("depthImagePushServiceOut", "roboticMiddleware", connections.depthImagePushServiceOut.roboticMiddleware);
		}
		// load parameters for server imagePushNewestServer
		parameter.getString("imagePushNewestServer", "serviceName", connections.imagePushNewestServer.serviceName);
		if(parameter.checkIfParameterExists("imagePushNewestServer", "roboticMiddleware")) {
			parameter.getString("imagePushNewestServer", "roboticMiddleware", connections.imagePushNewestServer.roboticMiddleware);
		}
		// load parameters for server imageQueryV2Server
		parameter.getString("imageQueryV2Server", "serviceName", connections.imageQueryV2Server.serviceName);
		if(parameter.checkIfParameterExists("imageQueryV2Server", "roboticMiddleware")) {
			parameter.getString("imageQueryV2Server", "roboticMiddleware", connections.imageQueryV2Server.roboticMiddleware);
		}
		
		// load parameters for task ImageTask
		parameter.getDouble("ImageTask", "minActFreqHz", connections.imageTask.minActFreq);
		parameter.getDouble("ImageTask", "maxActFreqHz", connections.imageTask.maxActFreq);
		parameter.getString("ImageTask", "triggerType", connections.imageTask.trigger);
		if(connections.imageTask.trigger == "PeriodicTimer") {
			parameter.getDouble("ImageTask", "periodicActFreqHz", connections.imageTask.periodicActFreq);
		} else if(connections.imageTask.trigger == "DataTriggered") {
			parameter.getString("ImageTask", "inPortRef", connections.imageTask.inPortRef);
			parameter.getInteger("ImageTask", "prescale", connections.imageTask.prescale);
		}
		if(parameter.checkIfParameterExists("ImageTask", "scheduler")) {
			parameter.getString("ImageTask", "scheduler", connections.imageTask.scheduler);
		}
		if(parameter.checkIfParameterExists("ImageTask", "priority")) {
			parameter.getInteger("ImageTask", "priority", connections.imageTask.priority);
		}
		if(parameter.checkIfParameterExists("ImageTask", "cpuAffinity")) {
			parameter.getInteger("ImageTask", "cpuAffinity", connections.imageTask.cpuAffinity);
		}
		
		// load parameters for ComponentKinectV2ServerROSExtension
		
		// load parameters for OpcUaBackendComponentGeneratorExtension
		
		// load parameters for PlainOpcUaComponentKinectV2ServerExtension
		
		
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
