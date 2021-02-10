#ifndef REAL_TIME_INTERFACE_PARSER_HH_
#define REAL_TIME_INTERFACE_PARSER_HH_

#include <vector>
#include <string>
#include <sstream>

#include <thread>

#include <cstdio>
#include <arpa/inet.h>

using namespace std;

class RealTimeInterfaceParser
{

public:
	struct RealtimeImterfaceData
	{

	public:
		// Time elapsed since the controller was started
		double time;

		// Target joint positions
		vector<double> q_target;

		// Target joint velocities
		vector<double> qd_target;

		// Target joint accelerations
		vector<double> qdd_target;

		// Target joint currents
		vector<double> I_target;

		// Target joint moments (torques)
		vector<double> M_target;

		// Actual joint positions
		vector<double> q_actual;

		// Actual joint velocities
		vector<double> qd_actual;

		// Actual joint currents
		vector<double> I_actual;

		// x,y and z accelerometer values from each joint (x0,y0,z0,...,x5,y5,z5)
		vector<double> Acc_values;

		// Generalized forces in the TCP
		vector<double> tcp_force;

		// Cartesian coordinates of the tool: (x,y,z,a0,a1,a2), where a0, a1 and a2 is an axis angle representation of the tool orientation
		vector<double> tool_vector;

		// Speed of the tool given in cartesian coordinates
		vector<double> tcp_speed;

		// Current state of the digital inputs. NOTE: these are bits encoded as int64_t!
		uint64_t digital_input_bits;

		// Temperature of each joint in degrees celcius
		vector<double> motor_temperatures;

	public:
		std::string asString() const;

	};

public:
	static char* parse(std::shared_ptr<RealtimeImterfaceData> data, char* ptr, size_t len);

private:
	static bool getBool(char* array);
	static int8_t getInt8(char* array);
	static uint8_t getUint8(char* array);
	static int16_t getInt16(char* array);
	static uint16_t getUint16(char* array);
	static float getFloat(char* array);
	static double getDouble(char* array);
	static int32_t getInt32(char* array);
	static uint32_t getUint32(char* array);
	static uint64_t getUint64(char* array);

};

#endif
