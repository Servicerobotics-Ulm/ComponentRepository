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
#include "ComponentSeq2SeqCom.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentSeq2SeqComAcePortFactory.hh"

#include "TaskEventOutEventTestHandler.hh"

// initialize static singleton pointer to zero
ComponentSeq2SeqCom* ComponentSeq2SeqCom::_componentSeq2SeqCom = 0;

// constructor
ComponentSeq2SeqCom::ComponentSeq2SeqCom()
{
	std::cout << "constructor of ComponentSeq2SeqCom\n";
	
	// set all pointer members to NULL
	commandHandler = NULL;
	//coordinationPort = NULL;
	//coordinationPort = NULL;
	debugTask = NULL;
	debugTaskTrigger = NULL;
	taskEventOut = NULL;
	taskEventOutWrapper = NULL;
	taskEventOutEventTestHandler = nullptr; 
	taskSendIn = NULL;
	taskSendInInputTaskTrigger = NULL;
	taskSendInUpcallManager = NULL;
	taskSendInInputCollector = NULL;
	taskSendOut = NULL;
	taskSendOutWrapper = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentSeq2SeqCom";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.taskEventOut.serviceName = "TaskEventOut";
	connections.taskEventOut.roboticMiddleware = "ACE_SmartSoft";
	connections.taskSendIn.serviceName = "TaskSendIn";
	connections.taskSendIn.roboticMiddleware = "ACE_SmartSoft";
	connections.taskSendOut.initialConnect = false;
	connections.taskSendOut.wiringName = "TaskSendOut";
	connections.taskSendOut.serverName = "unknown";
	connections.taskSendOut.serviceName = "unknown";
	connections.taskSendOut.interval = 1;
	connections.taskSendOut.roboticMiddleware = "ACE_SmartSoft";
	connections.debugTask.minActFreq = 0.0;
	connections.debugTask.maxActFreq = 0.0;
	// scheduling default parameters
	connections.debugTask.scheduler = "DEFAULT";
	connections.debugTask.priority = -1;
	connections.debugTask.cpuAffinity = -1;
	connections.commandHandler.prescale = 1;
	
	// initialize members of ComponentSeq2SeqComROS1InterfacesExtension
	
	// initialize members of ComponentSeq2SeqComROSExtension
	
	// initialize members of ComponentSeq2SeqComRestInterfacesExtension
	
	// initialize members of OpcUaBackendComponentGeneratorExtension
	
	// initialize members of PlainOpcUaComponentSeq2SeqComExtension
	
}

void ComponentSeq2SeqCom::addPortFactory(const std::string &name, ComponentSeq2SeqComPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentSeq2SeqCom::addExtension(ComponentSeq2SeqComExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentSeq2SeqCom::getComponentImpl()
{
	return dynamic_cast<ComponentSeq2SeqComAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentSeq2SeqCom::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentSeq2SeqCom::connectTaskSendOut(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.taskSendOut.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = taskSendOut->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->taskSendOut->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentSeq2SeqCom::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectTaskSendOut(connections.taskSendOut.serverName, connections.taskSendOut.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentSeq2SeqCom::startAllTasks() {
	// start task DebugTask
	if(connections.debugTask.scheduler != "DEFAULT") {
		ACE_Sched_Params debugTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.debugTask.scheduler == "FIFO") {
			debugTask_SchedParams.policy(ACE_SCHED_FIFO);
			debugTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.debugTask.scheduler == "RR") {
			debugTask_SchedParams.policy(ACE_SCHED_RR);
			debugTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		debugTask->start(debugTask_SchedParams, connections.debugTask.cpuAffinity);
	} else {
		debugTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentSeq2SeqCom::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentSeq2SeqCom::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "TaskSendIn") return taskSendInInputTaskTrigger;
	
	return NULL;
}


void ComponentSeq2SeqCom::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		// print out the actual parameters which are used to initialize the component
		std::cout << " \nComponentDefinition Initial-Parameters:\n" << COMP->getParameters() << std::endl;
		
		// initializations of ComponentSeq2SeqComROS1InterfacesExtension
		
		// initializations of ComponentSeq2SeqComROSExtension
		
		// initializations of ComponentSeq2SeqComRestInterfacesExtension
		
		// initializations of OpcUaBackendComponentGeneratorExtension
		
		// initializations of PlainOpcUaComponentSeq2SeqComExtension
		
		
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
		
		ComponentSeq2SeqComPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentSeq2SeqComAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentSeq2SeqCom is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		taskEventOutEventTestHandler = std::make_shared<TaskEventOutEventTestHandler>();
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		taskEventOutEventTestHandler = std::make_shared<TaskEventOutEventTestHandler>();
		taskEventOut = portFactoryRegistry[connections.taskEventOut.roboticMiddleware]->createTaskEventOut(connections.taskEventOut.serviceName, taskEventOutEventTestHandler);
		taskEventOutWrapper = new TaskEventOutWrapper(taskEventOut);
		taskSendIn = portFactoryRegistry[connections.taskSendIn.roboticMiddleware]->createTaskSendIn(connections.taskSendIn.serviceName);
		
		// create client ports
		taskSendOut = portFactoryRegistry[connections.taskSendOut.roboticMiddleware]->createTaskSendOut();
		taskSendOutWrapper = new TaskSendOutWrapper(taskSendOut);
		
		// create InputTaskTriggers and UpcallManagers
		taskSendInInputCollector = new TaskSendInInputCollector(taskSendIn);
		taskSendInInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommTaskMessage>(taskSendInInputCollector);
		taskSendInUpcallManager = new TaskSendInUpcallManager(taskSendInInputCollector);
		
		// create input-handler
		commandHandler = new CommandHandler(taskSendIn, connections.commandHandler.prescale);
		
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
		if(connections.taskSendOut.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::SendClient<CommBasicObjects::CommTaskMessage>*>(taskSendOut)->add(wiringSlave, connections.taskSendOut.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task DebugTask
		debugTask = new DebugTask(component);
		// configure input-links
		// configure task-trigger (if task is configurable)
		if(connections.debugTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.debugTask.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(debugTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				debugTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task DebugTask" << std::endl;
			}
		} else if(connections.debugTask.trigger == "DataTriggered") {
			debugTaskTrigger = getInputTaskTriggerFromString(connections.debugTask.inPortRef);
			if(debugTaskTrigger != NULL) {
				debugTaskTrigger->attach(debugTask, connections.debugTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.debugTask.inPortRef << " as activation source for Task DebugTask" << std::endl;
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
void ComponentSeq2SeqCom::run()
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
void ComponentSeq2SeqCom::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	if(debugTaskTrigger != NULL){
		debugTaskTrigger->detach(debugTask);
		delete debugTask;
	}

	// destroy all input-handler
	delete commandHandler;

	// destroy InputTaskTriggers and UpcallManagers
	delete taskSendInInputTaskTrigger;
	delete taskSendInUpcallManager;
	delete taskSendInInputCollector;

	// destroy client ports
	delete taskSendOutWrapper;
	delete taskSendOut;

	// destroy server ports
	delete taskEventOutWrapper;
	delete taskEventOut;
	delete taskSendIn;
	// destroy event-test handlers (if needed)
	taskEventOutEventTestHandler;
	
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
	
	// destruction of ComponentSeq2SeqComROS1InterfacesExtension
	
	// destruction of ComponentSeq2SeqComROSExtension
	
	// destruction of ComponentSeq2SeqComRestInterfacesExtension
	
	// destruction of OpcUaBackendComponentGeneratorExtension
	
	// destruction of PlainOpcUaComponentSeq2SeqComExtension
	
}

void ComponentSeq2SeqCom::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentSeq2SeqCom.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentSeq2SeqCom.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentSeq2SeqCom.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for client TaskSendOut
		parameter.getBoolean("TaskSendOut", "initialConnect", connections.taskSendOut.initialConnect);
		parameter.getString("TaskSendOut", "serviceName", connections.taskSendOut.serviceName);
		parameter.getString("TaskSendOut", "serverName", connections.taskSendOut.serverName);
		parameter.getString("TaskSendOut", "wiringName", connections.taskSendOut.wiringName);
		if(parameter.checkIfParameterExists("TaskSendOut", "roboticMiddleware")) {
			parameter.getString("TaskSendOut", "roboticMiddleware", connections.taskSendOut.roboticMiddleware);
		}
		
		// load parameters for server TaskEventOut
		parameter.getString("TaskEventOut", "serviceName", connections.taskEventOut.serviceName);
		if(parameter.checkIfParameterExists("TaskEventOut", "roboticMiddleware")) {
			parameter.getString("TaskEventOut", "roboticMiddleware", connections.taskEventOut.roboticMiddleware);
		}
		// load parameters for server TaskSendIn
		parameter.getString("TaskSendIn", "serviceName", connections.taskSendIn.serviceName);
		if(parameter.checkIfParameterExists("TaskSendIn", "roboticMiddleware")) {
			parameter.getString("TaskSendIn", "roboticMiddleware", connections.taskSendIn.roboticMiddleware);
		}
		
		// load parameters for task DebugTask
		parameter.getDouble("DebugTask", "minActFreqHz", connections.debugTask.minActFreq);
		parameter.getDouble("DebugTask", "maxActFreqHz", connections.debugTask.maxActFreq);
		parameter.getString("DebugTask", "triggerType", connections.debugTask.trigger);
		if(connections.debugTask.trigger == "PeriodicTimer") {
			parameter.getDouble("DebugTask", "periodicActFreqHz", connections.debugTask.periodicActFreq);
		} else if(connections.debugTask.trigger == "DataTriggered") {
			parameter.getString("DebugTask", "inPortRef", connections.debugTask.inPortRef);
			parameter.getInteger("DebugTask", "prescale", connections.debugTask.prescale);
		}
		if(parameter.checkIfParameterExists("DebugTask", "scheduler")) {
			parameter.getString("DebugTask", "scheduler", connections.debugTask.scheduler);
		}
		if(parameter.checkIfParameterExists("DebugTask", "priority")) {
			parameter.getInteger("DebugTask", "priority", connections.debugTask.priority);
		}
		if(parameter.checkIfParameterExists("DebugTask", "cpuAffinity")) {
			parameter.getInteger("DebugTask", "cpuAffinity", connections.debugTask.cpuAffinity);
		}
		if(parameter.checkIfParameterExists("CommandHandler", "prescale")) {
			parameter.getInteger("CommandHandler", "prescale", connections.commandHandler.prescale);
		}
		
		// load parameters for ComponentSeq2SeqComROS1InterfacesExtension
		
		// load parameters for ComponentSeq2SeqComROSExtension
		
		// load parameters for ComponentSeq2SeqComRestInterfacesExtension
		
		// load parameters for OpcUaBackendComponentGeneratorExtension
		
		// load parameters for PlainOpcUaComponentSeq2SeqComExtension
		
		
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
