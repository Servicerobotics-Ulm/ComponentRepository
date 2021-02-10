/*
 * RealTimeInterfaceParser.cc
 *
 *  Created on: Feb 1, 2011
 *      Author: wopfner
 */

#include "RealTimeInterfaceParser.hh"

std::string RealTimeInterfaceParser::RealtimeImterfaceData::asString() const
{

	stringstream out;
	out << "timestamp:			" << time << endl;

	// Target joint positions
	out << "-------------------" << endl;
	for (size_t i = 0; i < q_target.size(); i++)
	{
		out << "q_target[" << i << "]:			" << q_target[i] << endl;
	}

	// Target joint velocities
	out << "-------------------" << endl;
	for (size_t i = 0; i < qd_target.size(); i++)
	{
		out << "qd_target[" << i << "]:			" << qd_target[i] << endl;
	}

	// Target joint accelerations
	out << "-------------------" << endl;
	for (size_t i = 0; i < qdd_target.size(); i++)
	{
		out << "qdd_target[" << i << "]:			" << qdd_target[i] << endl;
	}

	// Target joint currents
	out << "-------------------" << endl;
	for (size_t i = 0; i < I_target.size(); i++)
	{
		out << "I_target[" << i << "]:			" << I_target[i] << endl;
	}

	// Target joint moments (torques)
	out << "-------------------" << endl;
	for (size_t i = 0; i < M_target.size(); i++)
	{
		out << "M_target[" << i << "]:			" << M_target[i] << endl;
	}

	// Actual joint positions
	out << "-------------------" << endl;
	for (size_t i = 0; i < q_actual.size(); i++)
	{
		out << "q_actual[" << i << "]:			" << q_actual[i] << endl;
	}

	// Actual joint velocities
	out << "-------------------" << endl;
	for (size_t i = 0; i < qd_actual.size(); i++)
	{
		out << "qd_actual[" << i << "]:			" << qd_actual[i] << endl;
	}

	// Actual joint currents
	out << "-------------------" << endl;
	for (size_t i = 0; i < I_actual.size(); i++)
	{
		out << "I_actual[" << i << "]:			" << I_actual[i] << endl;
	}

	// x,y and z accelerometer values from each joint (x0,y0,z0,...,x5,y5,z5)
	out << "-------------------" << endl;
	for (size_t i = 0; i < Acc_values.size(); i++)
	{
		out << "Acc_values[" << i << "]:			" << Acc_values[i] << endl;
	}

	// Generalized forces in the TCP
	out << "-------------------" << endl;
	for (size_t i = 0; i < tcp_force.size(); i++)
	{
		out << "tcp_force[" << i << "]:			" << tcp_force[i] << endl;
	}

	// Cartesian coordinates of the tool: (x,y,z,a0,a1,a2), where a0, a1 and a2 is an axis angle representation of the tool orientation
	out << "-------------------" << endl;
	for (size_t i = 0; i < tool_vector.size(); i++)
	{
		out << "tool_vector[" << i << "]:			" << tool_vector[i] << endl;
	}

	// Speed of the tool given in cartesian coordinates
	out << "-------------------" << endl;
	for (size_t i = 0; i < tcp_speed.size(); i++)
	{
		out << "tcp_speed[" << i << "]:			" << tcp_speed[i] << endl;
	}

	// Current state of the digital inputs. NOTE: these are bits encoded as int64_t!
	out << "-------------------" << endl;
	out << "digital_input_bits:		" << digital_input_bits << endl;

	// Temperature of each joint in degrees celcius
	out << "-------------------" << endl;
	for (size_t i = 0; i < motor_temperatures.size(); i++)
	{
		out << "motor_temperatures[" << i << "]:		" << motor_temperatures[i] << endl;
	}

	return out.str();
}

char * RealTimeInterfaceParser::parse(std::shared_ptr<RealtimeImterfaceData> data, char* ptr, size_t len)
{
	if (len < 4)
		return ptr;

	// Total message length in bytes
	int32_t messageSize = getInt32(ptr);
	ptr += 4;

	if ((size_t) messageSize > len)
		return ptr;

	//	cout << "message size: " << messageSize << endl;
	//	cout << "message len: " << len << endl;

	// Time elapsed since the controller was started
	data->time = getDouble(ptr);
	ptr += 8;

	// Target joint positions
	for (size_t i = 0; i < 6; i++)
	{
		data->q_target.push_back(getDouble(ptr));
		ptr += 8;
	}

	// Target joint velocities
	for (size_t i = 0; i < 6; i++)
	{
		data->qd_target.push_back(getDouble(ptr));
		ptr += 8;
	}

	// Target joint accelerations
	for (size_t i = 0; i < 6; i++)
	{
		data->qdd_target.push_back(getDouble(ptr));
		ptr += 8;
	}

	// Target joint currents
	for (size_t i = 0; i < 6; i++)
	{
		data->I_target.push_back(getDouble(ptr));
		ptr += 8;
	}

	// Target joint moments (torques)
	for (size_t i = 0; i < 6; i++)
	{
		data->M_target.push_back(getDouble(ptr));
		ptr += 8;
	}

	// Actual joint positions
	for (size_t i = 0; i < 6; i++)
	{
		data->q_actual.push_back(getDouble(ptr));
		ptr += 8;
	}

	// Actual joint velocities
	for (size_t i = 0; i < 6; i++)
	{
		data->qd_actual.push_back(getDouble(ptr));
		ptr += 8;
	}

	// Actual joint currents
	for (size_t i = 0; i < 6; i++)
	{
		data->I_actual.push_back(getDouble(ptr));
		ptr += 8;
	}

	// x,y and z accelerometer values from each joint (x0,y0,z0,...,x5,y5,z5)
	for (size_t i = 0; i < 18; i++)
	{
		data->Acc_values.push_back(getDouble(ptr));
		ptr += 8;
	}

	// Generalised forces in the TCP
	for (size_t i = 0; i < 6; i++)
	{
		data->tcp_force.push_back(getDouble(ptr));
		ptr += 8;
	}

	// Cartesian coordinates of the tool: (x,y,z,a0,a1,a2), where a0, a1 and a2 is an axis angle representation of the tool orientation
	for (size_t i = 0; i < 6; i++)
	{
		data->tool_vector.push_back(getDouble(ptr));
		ptr += 8;
	}

	// Speed of the tool given in cartesian coordinates
	for (size_t i = 0; i < 6; i++)
	{
		data->tcp_speed.push_back(getDouble(ptr));
		ptr += 8;
	}

	data->digital_input_bits = getUint64(ptr);
	ptr += 8;

	// Temperature of each joint in degrees celcius
	for (size_t i = 0; i < 6; i++)
	{
		data->motor_temperatures.push_back(getDouble(ptr));
		ptr += 8;
	}

	return ptr;
}

////////////////////////////////////////////////
//
// private
//
////////////////////////////////////////////////

bool RealTimeInterfaceParser::getBool(char* array)
{
	return (bool) getUint8(array);
}

int8_t RealTimeInterfaceParser::getInt8(char* array)
{
	return (int8_t) getUint8(array);
}

uint8_t RealTimeInterfaceParser::getUint8(char* array)
{
	uint8_t ret = array[0];
	return ret;
}

int16_t RealTimeInterfaceParser::getInt16(char* array)
{
	return (int16_t) getUint16(array);
}

uint16_t RealTimeInterfaceParser::getUint16(char* array)
{
	uint16_t i = 0;
	char *tmp = (char *) &i;

	tmp[0] = array[0];
	tmp[1] = array[1];

	return be16toh(i);
}

float RealTimeInterfaceParser::getFloat(char* array)
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

double RealTimeInterfaceParser::getDouble(char* array)
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

int32_t RealTimeInterfaceParser::getInt32(char* array)
{
	return (int32_t) getUint32(array);
}

uint32_t RealTimeInterfaceParser::getUint32(char* array)
{
	uint32_t i = 0;
	char *tmp = (char *) &i;

	tmp[0] = array[0];
	tmp[1] = array[1];
	tmp[2] = array[2];
	tmp[3] = array[3];

	return be32toh(i);
}

uint64_t RealTimeInterfaceParser::getUint64(char* array)
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
