/*
 * CommJoystickOpcUa.hh
 *
 *  Created on: Feb 6, 2018
 *      Author: alexej
 */

#ifndef COMMJOYSTICKOPCUA_HH_
#define COMMJOYSTICKOPCUA_HH_

// FreeOpcUa includes
#include <opc/ua/node.h>

#include "CommBasicObjects/CommJoystick.hh"

// serialization operator for CommunicationObject CommJoystick
bool operator<<(std::vector<OpcUa::Variant> &cdr, const CommBasicObjects::CommJoystick &obj);

// de-serialization operator for CommunicationObject CommJoystick
bool operator>>(const std::vector<OpcUa::Variant> &cdr, CommBasicObjects::CommJoystick &obj);

#endif /* COMMJOYSTICKOPCUA_HH_ */
