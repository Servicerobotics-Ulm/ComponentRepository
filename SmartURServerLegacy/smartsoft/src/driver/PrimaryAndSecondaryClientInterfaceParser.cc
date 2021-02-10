/*
 * PrimaryAndSecondaryClientInterfaceParser.cc
 *
 *  Created on: Feb 1, 2011
 *      Author: wopfner
 */

#include "PrimaryAndSecondaryClientInterfaceParser.hh"

std::string PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData::asString() const
{
	stringstream out;

	const PrimaryAndSecondaryClientInterfaceParser::RobotModeData &rmd = robotModeData;
	const vector<PrimaryAndSecondaryClientInterfaceParser::JointData> &jd = jointData;
	const PrimaryAndSecondaryClientInterfaceParser::CartesianInfo& ci = cartesianInfo;
	const PrimaryAndSecondaryClientInterfaceParser::ToolData& td = toolData;
	const PrimaryAndSecondaryClientInterfaceParser::MasterboardData& md = masterboardData;

	out << "--- RobotModeData ---" << endl;
	out << "timestamp:			" << rmd.timestamp << endl;
	out << "isRobotConnected:		" << rmd.isRobotConnected << endl;
	out << "isRealRobotEnabled:		" << rmd.isRealRobotEnabled << endl;
	out << "isPowerOnRobot:			" << rmd.isPowerOnRobot << endl;
	out << "isEmergencyStopped:		" << rmd.isEmergencyStopped << endl;
	out << "isSecurityStopped:		" << rmd.isSecurityStopped << endl;
	out << "isProgramRunning:		" << rmd.isProgramRunning << endl;
	out << "isProgramPaused:		" << rmd.isProgramPaused << endl;
	out << "robotMode:			" << (int) rmd.robotMode << endl;
	out << "speedFraction:			" << rmd.speedFraction << endl << endl;

	out << "--- JointData ---" << endl;
	for (size_t i = 0; i < jd.size(); i++)
	{
		out << "Joint :				" << i << endl;
		out << "q_actual:			" << jd[i].q_actual << endl;
		out << "q_target:			" << jd[i].q_target << endl;
		out << "qd_actual:			" << jd[i].qd_actual << endl;
		out << "I_actual:			" << jd[i].I_actual << endl;
		out << "V_actual:			" << jd[i].V_actual << endl;
		out << "T_motor:			" << jd[i].T_motor << endl;
		out << "T_micro:			" << jd[i].T_micro << endl;
		out << "jointMode:			" << (int) jd[i].jointMode << endl;
		out << "-------------------" << endl;
	}
	out << endl;

	out << "--- CartesianInfo ---" << endl;
	out << "x:				" << ci.x << endl;
	out << "y:				" << ci.y << endl;
	out << "z:				" << ci.z << endl;
	out << "rx:				" << ci.rx << endl;
	out << "ry:				" << ci.ry << endl;
	out << "rz:				" << ci.rz << endl << endl;

	out << "--- ToolData ---" << endl;
	out << "analogInputRange2:		" << (int) td.analogInputRange2 << endl;
	out << "analogInputRange3:		" << (int) td.analogInputRange3 << endl;
	out << "analogInput2:			" << td.analogInput2 << endl;
	out << "analogInput3:			" << td.analogInput3 << endl;
	out << "toolVoltage48V:			" << td.toolVoltage48V << endl;
	out << "toolOutputVoltage:		" << (int) td.toolOutputVoltage << endl;
	out << "toolCurrent:			" << td.toolCurrent << endl;
	out << "toolTemperature:		" << td.toolTemperature << endl << endl;

	out << "--- MasterboardData ---" << endl;
	out << "digitalInputBits:		" << (int) md.digitalInputBits << endl;
	out << "digitalOutputBits:		" << (int) md.digitalOutputBits << endl;
	out << "analogInputRange0:		" << (int) md.analogInputRange0 << endl;
	out << "analogInputRange1:		" << (int) md.analogInputRange1 << endl;
	out << "analogInput0:			" << md.analogInput0 << endl;
	out << "analogInput1:			" << md.analogInput1 << endl;
	out << "analogOutputDomain0:		" << (int) md.analogOutputDomain0 << endl;
	out << "analogOutputDomain1:		" << (int) md.analogOutputDomain1 << endl;
	out << "analogOutput0:			" << md.analogOutput0 << endl;
	out << "analogOutput1:			" << md.analogOutput1 << endl;
	out << "masterBoardTemperature:		" << md.masterBoardTemperature << endl;
	out << "robotVoltage48V:		" << md.robotVoltage48V << endl;
	out << "robotCurrent:			" << md.robotCurrent << endl;
	out << "masterIOCurrent:		" << md.masterIOCurrent << endl;
	out << "masterSaftyState:		" << (int) md.masterSaftyState << endl;
	out << "masterOnOffState:		" << (int) md.masterOnOffState << endl;

	return out.str();
}

char* PrimaryAndSecondaryClientInterfaceParser::parse(std::shared_ptr<ClientInterfaceData> data, char* ptr, size_t len)
{

	if (len < 5)
		return ptr;

	char* startPtr = ptr;

	int32_t messageSize = getInt32(ptr);
	ptr += 4;
	uint8_t messageType = getUint8(ptr);
	ptr += 1;

	//	cout << "message size: " << messageSize << endl;
	//	cout << "message len: " << len << endl;

	if ((size_t) messageSize > len)
		return ptr;

	while (ptr < startPtr + messageSize)
	{
		int32_t size = getInt32(ptr);
		uint8_t type = getUint8(ptr + 4);

		//std::cout << endl << "type: " << (int) type << ", bytes: " << size << endl;

		switch (type)
		{
		case 0:
			parseRobotModeData(data->robotModeData, ptr + 5);
			break;
		case 1:
			parseJointData(data->jointData, ptr + 5);
			break;
		case 2:
			parseToolData(data->toolData, ptr + 5);
			break;
		case 3:
			parseMasterboardData(data->masterboardData, ptr + 5);
			break;
		case 4:
			parseCartesianInfo(data->cartesianInfo, ptr + 5);
			break;
		case 20:
			parseMessages(ptr + 5);
			break;
		}

		ptr += size;
	}

	return ptr;
}

std::string PrimaryAndSecondaryClientInterfaceParser::robotModeToString(RobotMode mode)
{
	switch (mode)
	{
	case ROBOT_RUNNING_MODE:
		return "ROBOT_RUNNING_MODE";
	case ROBOT_BACKDRIVE_MODE:
		return "ROBOT_BACKDRIVE_MODE";
	case ROBOT_READY_MODE:
		return "ROBOT_READY_MODE";
	case ROBOT_INITIALIZING_MODE:
		return "ROBOT_INITIALIZING_MODE";
	case ROBOT_SECURITY_STOPPED_MODE:
		return "ROBOT_SECURITY_STOPPED_MODE";
	case ROBOT_EMERGENCY_STOPPED_MODE:
		return "ROBOT_EMERGENCY_STOPPED_MODE";
	case ROBOT_FATAL_ERROR_MODE:
		return "ROBOT_FATAL_ERROR_MODE";
	case ROBOT_NO_POWER_MODE:
		return "ROBOT_NO_POWER_MODE";
	case ROBOT_NOT_CONNECTED_MODE:
		return "ROBOT_NOT_CONNECTED_MODE";
	}
}

std::string PrimaryAndSecondaryClientInterfaceParser::jointModeToString(JointMode mode)
{
	switch (mode)
	{
	case JOINT_POWER_OFF_MODE:
		return "JOINT_POWER_OFF_MODE";
	case JOINT_EMERGENCY_STOPPED_MODE:
		return "JOINT_EMERGENCY_STOPPED_MODE";
	case JOINT_CALVAL_INITIALIZATION_MODE:
		return "JOINT_CALVAL_INITIALIZATION_MODE";
	case JOINT_ERROR_MODE:
		return "JOINT_ERROR_MODE";
	case JOINT_BACKDRIVEABLE_MODE:
		return "JOINT_BACKDRIVEABLE_MODE";
	case JOINT_SIMULATED_MODE:
		return "JOINT_SIMULATED_MODE";
	case JOINT_NOT_RESPONDING_MODE:
		return "JOINT_NOT_RESPONDING_MODE";
	case JOINT_MOTOR_INITIALISATION_MODE:
		return "JOINT_MOTOR_INITIALISATION_MODE";
	case JOINT_ADC_CALIBRATION_MODE:
		return "JOINT_ADC_CALIBRATION_MODE";
	case JOINT_DEAD_COMMUTATION_MODE:
		return "JOINT_DEAD_COMMUTATION_MODE";
	case JOINT_BOOTLOADER_MODE:
		return "JOINT_BOOTLOADER_MODE";
	case JOINT_CALIBRATION_MODE:
		return "JOINT_CALIBRATION_MODE";
	case JOINT_SECURITY_STOPPED_MODE:
		return "JOINT_SECURITY_STOPPED_MODE";
	case JOINT_FAULT_MODE:
		return "JOINT_FAULT_MODE";
	case JOINT_RUNNING_MODE:
		return "JOINT_RUNNING_MODE";
	case JOINT_INITIALISATION_MODE:
		return "JOINT_INITIALISATION_MODE";
	case JOINT_IDLE_MODE:
		return "JOINT_IDLE_MODE";
	}
}

////////////////////////////////////////////////
//
// private
//
////////////////////////////////////////////////

void PrimaryAndSecondaryClientInterfaceParser::parseRobotModeData(RobotModeData& robotModeData, char* ptr)
{
	//std::cout << "parse RobotModeData\n";
	robotModeData.timestamp = getUint64(ptr);
	ptr += 8;
	robotModeData.isRobotConnected = getBool(ptr);
	ptr += 1;
	robotModeData.isRealRobotEnabled = getBool(ptr);
	ptr += 1;
	robotModeData.isPowerOnRobot = getBool(ptr);
	ptr += 1;
	robotModeData.isEmergencyStopped = getBool(ptr);
	ptr += 1;
	robotModeData.isSecurityStopped = getBool(ptr);
	ptr += 1;
	robotModeData.isProgramRunning = getBool(ptr);
	ptr += 1;
	robotModeData.isProgramPaused = getBool(ptr);
	ptr += 1;
	robotModeData.robotMode = (RobotMode) getUint8(ptr);
	ptr += 1;
	robotModeData.speedFraction = getDouble(ptr);
}

void PrimaryAndSecondaryClientInterfaceParser::parseJointData(vector<JointData>& jointData, char* ptr)
{
	//std::cout << "parse JointData\n";

	jointData.clear();

	JointData d;
	for (size_t i = 0; i < 6; i++)
	{
		d.q_actual = getDouble(ptr);
		ptr += 8;
		d.q_target = getDouble(ptr);
		ptr += 8;
		d.qd_actual = getDouble(ptr);
		ptr += 8;
		d.I_actual = getFloat(ptr);
		ptr += 4;
		d.V_actual = getFloat(ptr);
		ptr += 4;
		d.T_motor = getFloat(ptr);
		ptr += 4;
		d.T_micro = getFloat(ptr);
		ptr += 4;
		d.jointMode = (JointMode) getUint8(ptr);
		ptr += 1;

		jointData.push_back(d);
	}
}

void PrimaryAndSecondaryClientInterfaceParser::parseCartesianInfo(CartesianInfo& cartesianInfo, char* ptr)
{
	//std::cout << "parse CartesianInfo\n";

	cartesianInfo.x = getDouble(ptr);
	ptr += 8;
	cartesianInfo.y = getDouble(ptr);
	ptr += 8;
	cartesianInfo.z = getDouble(ptr);
	ptr += 8;
	cartesianInfo.rx = getDouble(ptr);
	ptr += 8;
	cartesianInfo.ry = getDouble(ptr);
	ptr += 8;
	cartesianInfo.rz = getDouble(ptr);

	//std::cout<<"x|y|z | rx|ry|rz"<<cartesianInfo.x<<"|"<<cartesianInfo.y<<"|"<<cartesianInfo.z<<" | "<<cartesianInfo.rx<<"|"<<cartesianInfo.ry<<"|"<<cartesianInfo.rz<<std::endl;
}

void PrimaryAndSecondaryClientInterfaceParser::parseToolData(ToolData& toolData, char* ptr)
{
	//std::cout << "parse ToolData\n";

	toolData.analogInputRange2 = (IoAnalogRange) getInt8(ptr);
	ptr += 1;
	toolData.analogInputRange3 = (IoAnalogRange) getInt8(ptr);
	ptr += 1;
	toolData.analogInput2 = getDouble(ptr);
	ptr += 8;
	toolData.analogInput3 = getDouble(ptr);
	ptr += 8;
	toolData.toolVoltage48V = getFloat(ptr);
	ptr += 4;
	toolData.toolOutputVoltage = getUint8(ptr);
	ptr += 1;
	toolData.toolCurrent = getFloat(ptr);
	ptr += 4;
	toolData.toolTemperature = getFloat(ptr);
}

void PrimaryAndSecondaryClientInterfaceParser::parseMasterboardData(MasterboardData& masterboardData, char* ptr)
{
	//std::cout << "parse MasterboardData\n";

	masterboardData.digitalInputBits = getInt16(ptr);
	ptr += 2;
	masterboardData.digitalOutputBits = getInt16(ptr);
	ptr += 2;
	masterboardData.analogInputRange0 = (IoAnalogRange) getInt8(ptr);
	ptr += 1;
	masterboardData.analogInputRange1 = (IoAnalogRange) getInt8(ptr);
	ptr += 1;
	masterboardData.analogInput0 = getDouble(ptr);
	ptr += 8;
	masterboardData.analogInput1 = getDouble(ptr);
	ptr += 8;
	masterboardData.analogOutputDomain0 = getInt8(ptr);
	ptr += 1;
	masterboardData.analogOutputDomain1 = getInt8(ptr);
	ptr += 1;
	masterboardData.analogOutput0 = getDouble(ptr);
	ptr += 8;
	masterboardData.analogOutput1 = getDouble(ptr);
	ptr += 8;
	masterboardData.masterBoardTemperature = getFloat(ptr);
	ptr += 4;
	masterboardData.robotVoltage48V = getFloat(ptr);
	ptr += 4;
	masterboardData.robotCurrent = getFloat(ptr);
	ptr += 4;
	masterboardData.masterIOCurrent = getFloat(ptr);
	ptr += 4;
	masterboardData.masterSaftyState = getUint8(ptr);
	ptr += 1;
	masterboardData.masterOnOffState = getUint8(ptr);
}

void PrimaryAndSecondaryClientInterfaceParser::parseMessages(char* ptr)
{
	uint64_t timestamp = getUint64(ptr);
	ptr += 8;
	int8_t source = getInt8(ptr);
	ptr += 1;
	int8_t robotMessageType = getInt8(ptr);
	ptr += 1;

	cout << "timestamp: " << timestamp << ", source: " << source << ", robotMessageType: " << robotMessageType << endl;

}

bool PrimaryAndSecondaryClientInterfaceParser::getBool(char* array)
{
	return (bool) getUint8(array);
}

int8_t PrimaryAndSecondaryClientInterfaceParser::getInt8(char* array)
{
	return (int8_t) getUint8(array);
}

uint8_t PrimaryAndSecondaryClientInterfaceParser::getUint8(char* array)
{
	uint8_t ret = array[0];
	return ret;
}

int16_t PrimaryAndSecondaryClientInterfaceParser::getInt16(char* array)
{
	return (int16_t) getUint16(array);
}

uint16_t PrimaryAndSecondaryClientInterfaceParser::getUint16(char* array)
{
	uint16_t i = 0;
	char *tmp = (char *) &i;

	tmp[0] = array[0];
	tmp[1] = array[1];

	return be16toh(i);
}

float PrimaryAndSecondaryClientInterfaceParser::getFloat(char* array)
{
	float ret;
	uint64_t t = getUint32(array);

	char* tmp1 = (char *) &ret;
	char* tmp2 = (char *) &t;

	tmp1[0] = tmp2[0];
	tmp1[1] = tmp2[1];
	tmp1[2] = tmp2[2];
	tmp1[3] = tmp2[3];

	return ret;
}

double PrimaryAndSecondaryClientInterfaceParser::getDouble(char* array)
{
	double ret;
	uint64_t t = getUint64(array);

	char* tmp1 = (char *) &ret;
	char* tmp2 = (char *) &t;

	tmp1[0] = tmp2[0];
	tmp1[1] = tmp2[1];
	tmp1[2] = tmp2[2];
	tmp1[3] = tmp2[3];
	tmp1[4] = tmp2[4];
	tmp1[5] = tmp2[5];
	tmp1[6] = tmp2[6];
	tmp1[7] = tmp2[7];

	return ret;
}

int32_t PrimaryAndSecondaryClientInterfaceParser::getInt32(char* array)
{
	return (int32_t) getUint32(array);
}

uint32_t PrimaryAndSecondaryClientInterfaceParser::getUint32(char* array)
{
	uint32_t i = 0;
	char *tmp = (char *) &i;

	tmp[0] = array[0];
	tmp[1] = array[1];
	tmp[2] = array[2];
	tmp[3] = array[3];

	return be32toh(i);
}

uint64_t PrimaryAndSecondaryClientInterfaceParser::getUint64(char* array)
{
	uint64_t i = 0;
	char *tmp = (char *) &i;

	tmp[0] = array[0];
	tmp[1] = array[1];
	tmp[2] = array[2];
	tmp[3] = array[3];
	tmp[4] = array[4];
	tmp[5] = array[5];
	tmp[6] = array[6];
	tmp[7] = array[7];

	return be64toh(i);
}

