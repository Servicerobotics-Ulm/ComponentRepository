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
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#include "CompHandler.hh"
#include "ComponentTCLSequencer.hh"

#include "CompTask.hh"

#include <iostream>
#include <dlfcn.h>

// include std switch methods
#include "switchWiring.hh"
#include "switchFetchEvents.hh"
#include "switchComponentLifeCycleEvents.hh"
#include "switchTimer.hh"

//#define DEBUG_CI

//CompTask can not be a managed task!
CompTask COMP_TASK;

void CompHandler::onStartup()
{
	//NOT USED
}

void CompHandler::onShutdown()
{
	//NOT USED
}

////////////////////////////////////////////////
// Lisp-C-Interface


/* switchModule
 * inString
 * - begins always with opening bracket
 * - first token is always module string
 * - second token is always function string
 * - then next block in brackets is parameter block
 *
 * example
 *   (SmartRobotinobaseServer1 basestate (red yellow))
 *
 * error messages
 *   (error (error no module or no function number))
 *   (error (no module))
 *   and error messages of module specific functions
 *
 * success
 *   module specific success messages
 *
 */


#if defined (__GNUC__) && defined(__unix__)
extern "C" char* command(char* ciType, char* ciInstance, char* compType, char* compInstance, char* service, char* param)
#elif defined (WIN32) || defined (WIN64)
extern "C" __declspec(dllexport) char* command(char* ciType, char* ciInstance, char* compType, char* compInstance, char* service, char* param)
#endif
{
	//the buffer will be freed on the lisp side
	char* rtrn = NULL;

	std::string result;
#ifdef DEBUG_CI
	std::cout<<"============================"<<std::endl;
	std::cout<<"CiType: "<<ciType<<std::endl;
	std::cout<<"CiInstance: "<<ciInstance<<std::endl;
	std::cout<<"ComponentType: "<<compType<<std::endl;
	std::cout<<"ComponentInstance: "<<compInstance<<std::endl;
	std::cout<<"Service: "<<service<<std::endl;
	std::cout<<"Param: "<<param<<std::endl;
	std::cout<<"============================"<<std::endl;
#endif

	std::map<std::string,ComponentTCLSequencerCore::CiFunctions>::const_iterator it = COMP->ciMap.find(ciType);
	if (it != COMP->ciMap.end())
	{
		result = it->second.switchCiFunction(ciInstance,compType,compInstance,service,param);
	} else {
		result = "(error (no module))";
	}

	rtrn = convertAndAllocateStdStringToCString(result);

	return rtrn;
}


#if defined (__GNUC__) && defined(__unix__)
extern "C" void delay(int x)
#elif defined (WIN32) || defined (WIN64)
extern "C" __declspec(dllexport) void delay(int x)
#endif
{
	usleep(x);
}


#if defined (__GNUC__) && defined(__unix__)
extern "C" char* destroyci(char* ciName, char* ciInstanceName)
#elif defined (WIN32) || defined (WIN64)
extern "C" __declspec(dllexport) char* destroyci(char* ciName, char* ciInstanceName)
#endif
{
	//the buffer will be freed on the lisp side
	char* rtrn = NULL;

	std::ostringstream outString;
	outString.str("(error (fini module))");

	std::string nameOfCi(ciName);
	std::string nameOfCiInstance(ciInstanceName);
	std::cout<<"destroyci - ciName: "<<nameOfCi<<" ciInstanceName: "<<ciInstanceName<<std::endl;
	std::map<std::string,ComponentTCLSequencerCore::CiFunctions>::const_iterator it = COMP->ciMap.find(nameOfCi);

	if (it != COMP->ciMap.end())
	{
		int destoryRes = it->second.finiCiFunction(nameOfCiInstance);
		if(destoryRes == 1){
			outString.str("(ok ())");
		} else {
			outString.str("(error (destroy ci))");
		}
	} else {
		outString.str("(error (no module))");
	}

	rtrn = convertAndAllocateStdStringToCString(outString.str());
	return rtrn;
}




#if defined (__GNUC__) && defined(__unix__)
extern "C" char* instantiateci(char* ciName, char* ciInstanceName,  char* componentInstanceName, char* serviceNameMap)
#elif defined (WIN32) || defined (WIN64)
extern "C" __declspec(dllexport) char* instantiateci(char* ciName, char* ciInstanceName,  char* componentInstanceName, char* serviceNameMap)
#endif
{
	std::cout<<"==========C==========="<<std::endl;

	//the buffer will be freed on the lisp side
	char* rtrn = NULL;

	std::ostringstream outString;
	outString.str("(error (init module))");

	std::string nameOfCi(ciName);
	std::string nameOfCiInstance(ciInstanceName);
	std::cout<<"instantiateci - ciName: "<<nameOfCi<<" ciInstanceName: "<<ciInstanceName<<std::endl;


	std::cout<<"serviceNameMap: "<<serviceNameMap<<std::endl;
	//Parse servicenamemap string to map
	std::deque <std::string> result;
	std::stringstream servicenamemapStream(serviceNameMap);
	std::string rest = splitIntoLines(result, servicenamemapStream);
	if(!rest.empty()){
		std::cout<<__FUNCTION__<<"Error on parse servicenamemap rest:"<<rest<<std::endl;
		std::cout<<__FUNCTION__<<" "<<nameOfCi<<" FAILURE"<<std::endl;
		std::cout<<__FUNCTION__<<"Full input was: "<<serviceNameMap<<std::endl;
		std::cout<<"==========================================="<<std::endl;
		rtrn = convertAndAllocateStdStringToCString(outString.str());
		return rtrn;
	}


	std::map< std::string,std::string >newServiceNameMap;

	std::deque<std::string>::iterator it;
	std::string delimiter (" ");
	for(it=result.begin(); it!=result.end();++it)
	{
		std::string p0,p1,p2,p3;
		std::stringstream ss(*it);
		std::getline(ss, p0, ' ');
		std::getline(ss, p1, ' ');
//		std::cout<<"|"<<p0<<"|"<<p1<<"|"<<std::endl;
		newServiceNameMap.insert(std::make_pair(p0,p1));
	}

	//Create ci component connection
	CiConnection con;
	con.ciTypeName = ciName;
	con.componentInstanceName = componentInstanceName;
//	con.serviceNameMap.insert(std::make_pair(p2,p3));
	con.serviceNameMap = newServiceNameMap;
	std::cout<<"Create new CI Connection - compType: "<<con.ciTypeName<<" compInstName: "<<con.componentInstanceName<<std::endl;
	std::pair<std::map<std::string, CiConnection, ciLessLibC >::iterator,bool> ret = COMP->ciConnectionsMap.insert(std::pair<std::string, CiConnection>(nameOfCiInstance, con));

	////////////////////
	//
	std::cout<<"#####################################"<<std::endl;
	std::cout<<"DEBUG connectionsMap: "<<std::endl;
	std::cout<<"#####################################"<<std::endl;
	for( const auto& ci_pair : COMP->ciConnectionsMap )
	{

		CiConnection con = ci_pair.second;
		std::cout<<"Cinst: "<<ci_pair.first<<std::endl;
		std::cout<<"Ci Connection - ciType:"<<con.ciTypeName<<" compInst:"<<con.componentInstanceName<<std::endl;


		std::map< std::string,std::string >serviceNameMap = con.serviceNameMap;
		for( const auto& sc_pair : serviceNameMap )
		{
			std::cout << "{" << sc_pair.first << " , " << sc_pair.second <<"}\n";
		}
	}
	std::cout<<"#####################################"<<std::endl;
	//
	////////////////////

	std::map<std::string,ComponentTCLSequencerCore::CiFunctions>::const_iterator moduleIt = COMP->ciMap.find(nameOfCi);
	if (moduleIt != COMP->ciMap.end())
	{
		//initialize module by call into loaded function --> create and connect ports
		int initres =  moduleIt->second.initCiFunction(COMP->getComponentImpl(),nameOfCiInstance,COMP->ciConnectionsMap);
		if(initres != 0)
		{
			std::cout<<__FUNCTION__<<" "<<nameOfCi<<" FAILURE"<<std::endl;
			std::cout<<"==========================================="<<std::endl;
			rtrn = convertAndAllocateStdStringToCString(outString.str());
			return rtrn;
		}

		outString.str("(ok (success))");
		rtrn = convertAndAllocateStdStringToCString(outString.str());
		return rtrn;
	} else {
		outString.str("(error (no module))");
		rtrn = convertAndAllocateStdStringToCString(outString.str());
		return rtrn;
	}
}


/* loadmodule
 * This function load module files (shared object libs)
 * and connect to the services of the loaded module
 *
 * moduleName
 * - string with the name of the module
 * servicenamemap
 * - string with
 *   1. the name of the used component instance
 *   2. the original name of the service (from component model)
 *   3. the runtime name of the service from the component instance (e.g. change by ini file)
 *   example: SmartRobtinoBaseServer1 baseQueryServer basestate
 *
 * return
 *   0 on success
 *   1 on failure
 *
 */
#if defined (__GNUC__) && defined(__unix__)
extern "C" char* loadcoordinationinterface(char* ciName, char* ciPath)
#elif defined (WIN32) || defined (WIN64)
extern "C" __declspec(dllexport) char* loadcoordinationinterface(char* ciName, char* ciPath)
#endif
{

	//the buffer will be freed on the lisp side
	char* rtrn = NULL;

	std::ostringstream outString;
	outString.str("(error (load ci))");

	std::string nameOfModule(ciName);
	std::stringstream moduleSoFileName;
	moduleSoFileName <<ciPath<<"/lib"<<nameOfModule<<".so";

	std::map<std::string,ComponentTCLSequencerCore::CiFunctions>::const_iterator moduleIt = COMP->ciMap.find(nameOfModule);
	if (moduleIt != COMP->ciMap.end())
	{
		std::cout<<"WARNING: trying to load already loaded ci --> skip!"<<std::endl;
		outString.str("(ok nil)");
		rtrn = convertAndAllocateStdStringToCString(outString.str());
		return rtrn;
	}

	std::cout<<"Opening "<<moduleSoFileName.str()<<std::endl;

	//open so file of module
	void* handle = dlopen(moduleSoFileName.str().c_str(), RTLD_LAZY);

	if (!handle) {
		std::cerr << "Cannot open library: " << dlerror() << '\n';
		std::cout<<__FUNCTION__<<" "<<nameOfModule<<" FAILURE"<<std::endl;
		rtrn = convertAndAllocateStdStringToCString(outString.str());
		return rtrn;
	}

	// load the symbol
	std::cout << "Loading symbols...\n";

	// reset errors
	dlerror();
	std::stringstream moduleinitfunction;
	moduleinitfunction << "init"<<nameOfModule;

	COMP->ciMap[nameOfModule].initCiFunction = (ComponentTCLSequencerCore::initCiFunction_t) dlsym(handle, moduleinitfunction.str().c_str());

	const char *dlsym_error_init = dlerror();
	if (dlsym_error_init) {
		std::cerr << "Cannot load symbol '"<<moduleinitfunction.str().c_str()<<"': " << dlsym_error_init <<'\n';
		dlclose(handle);
		std::cout<<__FUNCTION__<<" "<<nameOfModule<<" FAILURE"<<std::endl;
		rtrn = convertAndAllocateStdStringToCString(outString.str());
		return rtrn;
	}


	std::stringstream modulefunction;
	modulefunction << "switch"<<nameOfModule;

	//store functionpointer into ciMap for later user in switchModule
	COMP->ciMap[nameOfModule].switchCiFunction = (ComponentTCLSequencerCore::switchCiFunction_t) dlsym(handle, modulefunction.str().c_str());

	const char *dlsym_error_switch = dlerror();
	if (dlsym_error_switch) {
		std::cerr << "Cannot load symbol '"<<modulefunction.str().c_str()<<"': " << dlsym_error_switch <<'\n';
		dlclose(handle);
		std::cout<<__FUNCTION__<<" "<<nameOfModule<<" FAILURE"<<std::endl;
		rtrn = convertAndAllocateStdStringToCString(outString.str());
		return rtrn;
	}


	std::stringstream modulefinifunction;
	modulefinifunction << "fini"<<nameOfModule;

	COMP->ciMap[nameOfModule].finiCiFunction = (ComponentTCLSequencerCore::finiCiFunction_t) dlsym(handle, modulefinifunction.str().c_str());

	const char *dlsym_error_fini = dlerror();
		if (dlsym_error_fini) {
			std::cerr << "Cannot load symbol '"<<modulefinifunction.str().c_str()<<"':" << dlsym_error_fini <<'\n';
			dlclose(handle);
			std::cout<<__FUNCTION__<<" getLispModule FAILURE"<<std::endl;
			rtrn = convertAndAllocateStdStringToCString(outString.str());
			return rtrn;
		}

	//TODO where to close the handle?
	//dlclose(handle);

	outString.str("(ok ())");

	rtrn = convertAndAllocateStdStringToCString(outString.str());
	return rtrn;
}



#if defined (__GNUC__) && defined(__unix__)
extern "C" int initialize(char* param)
#elif defined (WIN32) || defined (WIN64)
extern "C" __declspec(dllexport) int initialize(char* param)
#endif
{
	Smart::StatusCode status;

	// force to use point as decimal seperator (required because of SBCL)
	std::setlocale(LC_NUMERIC, "C");

	std::cout << "initialize  param: " << param << "\n";


	std::istringstream iss(param);
	std::string s;
	std::vector< std::string > list;
	while ( getline( iss, s, ' ' ) ) {
		list.push_back(s);
	}

	char *argv[list.size()];
	for(unsigned int i=0;i<list.size();i++){
		argv[i] = const_cast<char*> (list[i].c_str());
	}
//	argv[0] = (char*)"dummy";
//	argv[1] = paramFile;
	COMP->init(list.size(), argv);

	// start all tasks
//	COMP_TASK.open();

	COMP_TASK.start();

	//set component state to ALIVE!
	COMP->setStartupFinished();

	// connect to all services
	// CONNECTION OF SERVICES IS DONE IN LOADMODULE TRIGGERED ON RUN-TIME!

	COMP->ciMap["WIRING"].switchCiFunction = &switchWiring;
	COMP->ciMap["WIRING"].initCiFunction = NULL;
	COMP->ciMap["WIRING"].finiCiFunction = NULL;

	COMP->ciMap["FETCHEVENTS"].switchCiFunction = &switchFetchEvents;
	COMP->ciMap["FETCHEVENTS"].initCiFunction = NULL;
	COMP->ciMap["FETCHEVENTS"].finiCiFunction = NULL;

	COMP->ciMap["COMPONENT"].switchCiFunction = &switchComponentLifeCycleEvents;
	COMP->ciMap["COMPONENT"].initCiFunction = NULL;
	COMP->ciMap["COMPONENT"].finiCiFunction = NULL;

	COMP->ciMap["TIMER"].switchCiFunction = &switchTimer;
	COMP->ciMap["TIMER"].initCiFunction = NULL;
	COMP->ciMap["TIMER"].finiCiFunction = NULL;


	return 0;
}


//used from the lisp interpreter to wait for the component to be finished
#if defined (__GNUC__) && defined(__unix__)
extern "C" void waitoncomptasktocomplete()
#elif defined (WIN32) || defined (WIN64)
extern "C" __declspec(dllexport) void waitoncomptasktocomplete()
#endif
{
	ACE_Thread_Manager::instance()->wait_task(&COMP_TASK);
	std::cout<<"waitoncomptasktocomplete - COMP_TASK  DONE"<<std::endl;
}


//used from the lisp interpreter to wait for a (c++) component initiated shutdown (SIGNALS)
#if defined (__GNUC__) && defined(__unix__)
extern "C" void waitoncompshutdown()
#elif defined (WIN32) || defined (WIN64)
extern "C" __declspec(dllexport) void waitoncompshutdown()
#endif
{

	std::cout<<"waitoncompshutdown - called"<<std::endl;
	std::cout<<"waitoncompshutdown - wait.."<<std::endl;
	Smart::StatusCode status;
	status = COMP->stateSlave->acquire("shutdown");
	std::cout<<"State status: "<<Smart::StatusCodeConversion(status)<<std::endl;
	std::cout<<"waitoncompshutdown - ..DONE"<<std::endl;
}


//used to initiate a component shutdown from lisp side (this should be the normal
//shutdown use case)
#if defined (__GNUC__) && defined(__unix__)
extern "C" void shutdowncomp()
#elif defined (WIN32) || defined (WIN64)
extern "C" __declspec(dllexport) void shutdowncomp()
#endif
{
	std::cout<<"shutdowncomp - called.."<<std::endl;
	COMP->stateSlave->setWaitState("Shutdown");
	std::cout<<"shutdowncomp - ..DONE"<<std::endl;
}


/////////////////////////////////////////
// HELPER FUNCTION


/*
 * This function converts an std string to a c-string.
 * The function is used to deal with the language interface C --> LISP
 * WARNING: The allocated memory needs to be freed afterwards!
 */

char* convertAndAllocateStdStringToCString (const std::string& inString){
	char* rtrn = NULL;

	//1 bytes extra for the null termination
	unsigned int char_size = 1;

	char_size += inString.size();

	rtrn = (char *)malloc(char_size * sizeof(char));
	strcpy(rtrn, inString.c_str());

	return rtrn;
}

std::string  splitIntoLines(std::deque <std::string> &result, std::stringstream &ss)
{
	std::string line;
	std::string rest = "";
	do{
		line = "";
		//ss >> line;
		std::getline(ss,line);
		//std::cout << "getline: line: "<<line<<" good: " << ss.good() << " eof: " << ss.eof() << " fail: " << ss.fail() << " bad: " << ss.bad() << std::endl;
		if(ss.good()){
			result.push_back( line );
		}else
		{
			rest = line;
		}

	}
	while (ss.good());
	return rest;
}
