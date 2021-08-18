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


//--------------------------------------------------------------------------
//BSD 3-Clause License
//
//  Copyright (C) Servicerobotics Ulm
//  University of Applied Sciences Ulm
//  Prittwitzstr. 10
//  89075 Ulm
//  Germany
//  All rights reserved.
//
//  Author: Nayabrasul Shaik
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//* Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.
//
//* Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//* Neither the name of the copyright holder nor the names of its
//  contributors may be used to endorse or promote products derived from
//  this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//--------------------------------------------------------------------------


#include "ConsoleTask.hh"
#include "ComponentQtRobotConsole.hh"

#include <smartNSAdapterACE.hh>
#include <ComponentListModel.hh>
#include <iostream>
#include <ios>
#include <string>
#include <iostream>
#include <string>
#include <string.h>
#include <stdio.h>

#include <QGuiApplication>
#include <QQmlApplicationEngine>
//#include <QQmlEngine>
#include <QQmlContext>
#include <QIcon>

#ifndef LISP_SEPARATOR
#define LISP_SEPARATOR " ()\"\n"
#define LISP_STRING    1000
#endif

ConsoleTask::ConsoleTask(SmartACE::SmartComponent *comp) 
:	ConsoleTaskCore(comp)
{
	std::cout << "constructor ConsoleTask\n";
}
ConsoleTask::~ConsoleTask() 
{
	std::cout << "destructor ConsoleTask\n";
}



int ConsoleTask::on_entry()
{
	//Qt stuff
	int argc =1;
	char* arg0 = "ComponentQtRobotConsole";
	char** argv = &arg0;

	//load QML
	QGuiApplication app(argc, argv);
	app.setWindowIcon(QIcon("qrc:qml_styling/app.png"));
	QQmlApplicationEngine engine;

	ComponentListModel cl(COMP->stateMaster, COMP->paramMaster);
	cl.refresh();

	QQmlContext* context = engine.rootContext();
	context->setContextProperty("ComponentListModel", &cl);
	engine.load(QUrl("qrc:/main.qml"));

	if (engine.rootObjects().isEmpty())
		return -1;

	app.exec();
	return 0;
}
int ConsoleTask::on_execute()
{
	return 0;
}
int ConsoleTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
