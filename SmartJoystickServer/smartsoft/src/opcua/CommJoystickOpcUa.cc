/*
 * CommJoystickOpcUa.cc
 *
 *  Created on: Feb 6, 2018
 *      Author: alexej
 */

#include "CommJoystickOpcUa.hh"


// serialization operator for element CommJoystick
bool operator<<(std::vector<OpcUa::Variant> &cdr, const CommBasicObjects::CommJoystick &obj)
{
	cdr.resize(5);
	cdr[0] = obj.getXpos();
	cdr[1] = obj.getYpos();
	cdr[2] = obj.getX2pos();
	cdr[3] = obj.getY2pos();
	cdr[4] = obj.getButtons();
	return true;
}

// de-serialization operator for element CommJoystick
bool operator>>(const std::vector<OpcUa::Variant> &cdr, CommBasicObjects::CommJoystick &obj)
{
	if(cdr.size() < 5) return false;
	obj.setXpos(cdr[0].As<short>());
	obj.setYpos(cdr[1].As<short>());
	obj.setX2pos(cdr[2].As<short>());
	obj.setY2pos(cdr[3].As<short>());
	obj.setButtons(cdr[4].As<unsigned short>());
	return true;
}
