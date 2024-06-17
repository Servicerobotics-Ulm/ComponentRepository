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

// --------------------------------------------------------------------------
//
//  Copyright (C) 2014 Matthias Lutz
//
//        schlegel@hs-ulm.de
//
//        ZAFH Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// --------------------------------------------------------------------------
#include "UnLoadTask.hh"
#include "ComponentRobotinoConveyerBeltServer_OPCUA.hh"

#include <iostream>
#include <future>
#include <thread>
#include <chrono>

UnLoadTask::UnLoadTask(SmartACE::SmartComponent *comp) :
    UnLoadTaskCore(comp) {
  std::cout << "constructor UnLoadTask\n";
  box_present_bit = 0;
  unload_signal_bit = 0;
  if (COMP->getGlobalState().getRobot().getSignal_unloading_dout() >= 0) {
    unload_signal_bit = COMP->getGlobalState().getRobot().getSignal_unloading_dout();
  }
}
UnLoadTask::~UnLoadTask() {
  std::cout << "destructor UnLoadTask\n";
}

int UnLoadTask::on_entry() {
  // do initialization procedures here, which are called once, each time the task is started
  // it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
  box_present_bit = COMP->getGlobalState().getRobot().getBox_present_din();
  return 0;
}

void UnLoadTask::resetAbortFlag() {
  SmartACE::SmartRecursiveGuard g(this->abortLock);
  abortFlag = false;
}

bool UnLoadTask::getAbortFlag() {
  SmartACE::SmartRecursiveGuard g(this->abortLock);
  return abortFlag;
}

void UnLoadTask::setAbortFlag() {
  SmartACE::SmartRecursiveGuard g(this->abortLock);
  abortFlag = true;
}

int UnLoadTask::on_execute() {
    // this method is called from an outside loop,
    // hence, NEVER use an infinite loop (like "while(1)") here inside!!!
    // also do not use blocking calls which do not result from smartsoft kernel

    Smart::StatusCode status;
    OPCUA::StatusCode opcuaStatus;
    CommRobotinoObjects::RobotinoConveyerBeltEventState eventState;

    status = COMP->stateSlave->acquire("unload");
    COMP->stateSlave->release("unload");
    std::cout << "[UnLoadTask]: start..." << std::endl;
    if (status == Smart::SMART_OK) {
        COMP->triggerUnLoad.acquire();

        ParameterStateStruct localState = COMP->getGlobalState();
        float unloadDirection = localState.getRobot().getUnload_motor_direction();
        bool signalUnLoading = (localState.getRobot().getSignal_unloading_dout() >= 0);
        int station_id = localState.getCommRobotinoObjects().getRobotinoConveyerParameter().getSetStationID().getId();

        bool isPixtend;
        std::list<std::string> isPixtendList = localState.getOPCUAstatic().getIs_pixtend();
        if (station_id >= 0 && station_id < isPixtendList.size()) {
            std::list<std::string>::iterator it = isPixtendList.begin();
            std::advance(it, station_id);
            isPixtend = *it == "1";
        } else
            isPixtend = true;

        std::string station_addr;
        bool passiveStation = false;
        std::list<std::string> stationsList = localState.getOPCUAstatic().getServer_address();
        std::vector<std::string> stationsVector { std::begin(stationsList), std::end(stationsList) };

        if (station_id >= 0 && station_id < stationsVector.size()) {
            station_addr = stationsVector.at(station_id);
            std::cout << "[UnLoadTask] station id: " << station_id << " station_addr: " << station_addr << std::endl;
            //deal with passive drop of stations
            if (localState.getRobot().getIgnore_station_communication() == true) {
                passiveStation = true;
            } else if (station_addr.compare("PASSIVE_STATION") == 0) {
                passiveStation = true;
            } else {
                passiveStation = false;
            }
            //this is a hack to use one station with different communication types
            //in this case it is used for unloading only!!!
            if (station_addr[0] == '!') {
                std::cout << "Overwrite passiveStation!" << std::endl;
                station_addr.erase(0, 1);
                passiveStation = true;
            }
        } else {
            std::cout << "ERROR invalid station id: " << station_id << " - stationsVector size: " << stationsVector.size() << std::endl;
            station_addr = "opc.tcp://0.0.0.0:4840";
        }
        if (this->getAbortFlag() == true) {
            return 0;
        }
        if (queryDigitalInput(box_present_bit) != true) {
            eventState.set(CommRobotinoObjects::RobotinoConveyerBeltEventType::CONVEYER_BELT_UNLOAD_ERROR_NO_BOX);
            COMP->robotinoConveyerBeltEventOut->put(eventState);
            std::cout << "[UnLoadTask] ERROR: unloading triggered - no box on the belt!" << std::endl;
            COMP->commPowerOutputSendOut->send(CommRobotinoObjects::CommRobotinoPowerOutputValue(0.0));
        } else {
            if (passiveStation == false) {
                //1. Connect to Station
                std::cout << "station_addr=" << station_addr << " name=" << localState.getOPCUAstatic().getObject_name() << std::endl;
                if (isPixtend) {
                    opcuaStatus = stationPixtend.connect(station_addr, localState.getOPCUAstatic().getObject_name());
                } else {
                    opcuaStatus = stationCpx.connect(station_addr, localState.getOPCUAstatic().getRootObjectPath(), 1);
                }
                std::cout << "[UnLoadTask] connect: " << opcuaStatus << std::endl;

                //this sleep might help the OPC server to sort things
                ACE_OS::sleep(ACE_Time_Value(1, 0));
            }
            //Trans_ready is the signal from Transport Station
            if (passiveStation == true || opcuaStatus == OPCUA::StatusCode::ALL_OK) {
                std::cout << "[UnLoadTask] unloading" << std::endl; //assuming the programm cycle time is 1ms
                if (passiveStation == true) {
                    //no station case
                    for (unsigned int i = 0; i < localState.getRobot().getIgnore_station_communication_unload_time_sec() * 10; ++i) {
                        if (signalUnLoading == true) {
                            queryDigitalOutput(unload_signal_bit, true);
                        }
                        std::cout << "[UnLoadTask] Still in the loop - passive" << std::endl;
                        //sleep till the box is loaded
                        Smart::StatusCode status;
                        status = COMP->commPowerOutputSendOut->send(CommRobotinoObjects::CommRobotinoPowerOutputValue(50.0 * unloadDirection));
                        if (status != Smart::SMART_OK) {
                            std::cout << "Error sending power output: " << Smart::StatusCodeConversion(status) << std::endl;
                        }
                        ACE_OS::sleep(ACE_Time_Value(0, 100000)); // 0.1 seconds
                        if (getAbortFlag() == true) {
                            std::cout << "[UnLoadTask] Abort Loop!" << std::endl;
                            break;
                        }
                    }
                } else {
                    //normal case (communication with station)
                    //4. Command Station to receive the Box
                    std::string callResult;
                    //TODO THE status code will be ERROR_COMMUNICATION FOR WHAT EVER REASON?
                    if (isPixtend) {
                        std::cout << "[UnLoadTask] Calling loadbox async..." << std::endl;
                        stationPixtend.setLED_YELLOW(true);
                        stationPixtend.setLED_GREEN(true);
                        auto future = std::async(std::launch::async, &OPCUA::ProductionStation::callLoadbox, &stationPixtend, 10, std::ref(callResult));
                        //loop till the station send done
                        while (future.wait_for(std::chrono::microseconds(0)) != std::future_status::ready) {
                            if (signalUnLoading == true) {
                                queryDigitalOutput(unload_signal_bit, true);
                            }
                            std::cout << "[UnLoadTask] Still in the loop" << std::endl;
                            //sleep till the box is loaded
                            COMP->commPowerOutputSendOut->send(CommRobotinoObjects::CommRobotinoPowerOutputValue(50.0 * unloadDirection));
                            ACE_OS::sleep(ACE_Time_Value(0, 250000));
                            if (getAbortFlag() == true) {
                                std::cout << "[UnLoadTask] Abort Loop!" << std::endl;
                                break;
                            }
                        }
                        stationPixtend.setLED_YELLOW(false);
                        auto result = future.get();
                        std::cout << "[UnLoadTask] loadbox call result: " << result << " value: " << callResult << std::endl;
                    } else {
                        int timeOut=10;  // seconds
                        float timeCounter=0.0;
                        int sleepTime=250000;  // micro seconds
                        stationCpx.setLED_YELLOW(true);
                        stationCpx.setLED_GREEN(true);
                        stationCpx.setMotor_timeout(timeOut);
                        stationCpx.setLoadbox(true);
                        std::cout << "stationCpx.getLoadbox()= " << stationCpx.getLoadbox() << std::endl;
                        while (stationCpx.getMethod_result() != "SUCCESS") {
                            if (signalUnLoading == true) {
                                queryDigitalOutput(unload_signal_bit, true);
                            }
                            std::cout << "[UnLoadTask] Still in the loop" << std::endl;
                            //sleep till the box is loaded
                            COMP->commPowerOutputSendOut->send(CommRobotinoObjects::CommRobotinoPowerOutputValue(50.0 * unloadDirection));
                            ACE_OS::sleep(ACE_Time_Value(0, sleepTime));
                            timeCounter+=sleepTime/1000000.0;
                            if (getAbortFlag() == true || timeCounter>timeOut) {
                                std::cout << "[UnLoadTask] Abort Loop!" << std::endl;
                                break;
                            }
                        }
                        stationCpx.setLED_YELLOW(false);
                    }
                }
                COMP->commPowerOutputSendOut->send(CommRobotinoObjects::CommRobotinoPowerOutputValue(0.0));
                if (signalUnLoading == true) {
                    queryDigitalOutput(unload_signal_bit, false);
                }
                std::cout << "[UnLoadTask] Loop done" << std::endl;
                if (queryDigitalInput(box_present_bit) != true) {
                    eventState.set(CommRobotinoObjects::RobotinoConveyerBeltEventType::CONVEYER_BELT_UNLOAD_DONE);
                    COMP->robotinoConveyerBeltEventOut->put(eventState);
                    std::cout << "[UnLoadTask] Load EVENT CONVEYER_BELT_UNLOAD_DONE FIRED!" << std::endl;
                } else {
                    eventState.set(CommRobotinoObjects::RobotinoConveyerBeltEventType::CONVEYER_BELT_UNLOAD_ERROR_BOX_STILL_PRESENT);
                    COMP->robotinoConveyerBeltEventOut->put(eventState);
                    std::cout << "[UnLoadTask] ERROR BOX STILL ON ROBOTINO" << std::endl;
                }
            } else {
                eventState.set(CommRobotinoObjects::RobotinoConveyerBeltEventType::CONVEYER_BELT_UNLOAD_ERROR_NO_RESPONSE_FROM_STATION);
                COMP->robotinoConveyerBeltEventOut->put(eventState);
                std::cout << "[unloadTask] ERROR: No trans ready from Station!" << std::endl;
                COMP->commPowerOutputSendOut->send(CommRobotinoObjects::CommRobotinoPowerOutputValue(0.0));
            }
            //5. Disconnect from Station
            if (passiveStation == false) {
                if (isPixtend)
                    stationPixtend.disconnect();
                else
                    stationCpx.disconnect();
            }
        }
    } // if (status == CHS::SMART_OK) <<<---- status = state->acquire("moverobot");
    std::cout << "[UnLoadTask]: ...stop" << std::endl;
    // it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
    return 0;
}

int UnLoadTask::on_exit() {
  // use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
  return 0;
}

bool UnLoadTask::queryDigitalInput(const unsigned int &bit) {

  CommBasicObjects::CommIOValues inputQuery;
  CommBasicObjects::CommIOValues inputQueryResult;

  Smart::StatusCode status;
  status = COMP->commIOValuesQueryServiceReq->query(inputQuery, inputQueryResult);
  if (status != Smart::SMART_OK) {
    std::cout << __FUNCTION__ << "[UnLoadTask] ERROR: query digital IO" << std::endl;
    COMP->stateSlave->setWaitState("FatalError");
    return false;
  }
  if (bit > inputQueryResult.getDigitalInputValuesSize()) {
    std::cout << __FUNCTION__ << "[UnLoadTask] ERROR: query bit: " << bit
        << " digital IO size: " << inputQueryResult.getDigitalInputValuesSize() << std::endl;
    COMP->stateSlave->setWaitState("FatalError");
    return false;
  }
  return inputQueryResult.getDigitalInputValuesElemAtPos(bit);

}

void UnLoadTask::queryDigitalOutput(const unsigned int &bit, const bool &value) {
  CommBasicObjects::CommIOValues outputQuery;
  CommBasicObjects::CommIOValues dummy;
  outputQuery.resizeDigitalOutputValues(1);
  Smart::StatusCode status;
  CommBasicObjects::CommDigitalOutputRequest digitalOutputRequest(bit, value);

  outputQuery.setDigitalOutputValuesElemAtPos(0, digitalOutputRequest);
  status = COMP->commIOValuesQueryServiceReq->query(outputQuery, dummy);
  if (status != Smart::SMART_OK) {
    std::cout << __FUNCTION__ << "[UnLoadTask] ERROR: query digital IO" << std::endl;
    COMP->stateSlave->setWaitState("FatalError");
  }
}
