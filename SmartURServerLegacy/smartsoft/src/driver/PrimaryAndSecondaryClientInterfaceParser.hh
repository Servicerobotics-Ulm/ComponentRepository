#ifndef PRIMARY_AND_SECONDARY_CLIENT_INTERFACE_PARSER_HH_
#define PRIMARY_AND_SECONDARY_CLIENT_INTERFACE_PARSER_HH_

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <cstdio>
#include <arpa/inet.h>

#include <thread>

using namespace std;

//TODO Messages from UR Controller have changed in newer versions! (lutz 2013-01-23)

class PrimaryAndSecondaryClientInterfaceParser {

public:
	enum RobotMode {
		ROBOT_RUNNING_MODE = 0,
		ROBOT_BACKDRIVE_MODE = 1,
		ROBOT_READY_MODE = 2,
		ROBOT_INITIALIZING_MODE = 3,
		ROBOT_SECURITY_STOPPED_MODE = 4,
		ROBOT_EMERGENCY_STOPPED_MODE = 5,
		ROBOT_FATAL_ERROR_MODE = 6,
		ROBOT_NO_POWER_MODE = 7,
		ROBOT_NOT_CONNECTED_MODE = 8
	};

	enum JointMode {
		JOINT_POWER_OFF_MODE = 239,
		JOINT_EMERGENCY_STOPPED_MODE = 240,
		JOINT_CALVAL_INITIALIZATION_MODE = 241,
		JOINT_ERROR_MODE = 242,
		JOINT_BACKDRIVEABLE_MODE = 243,
		JOINT_SIMULATED_MODE = 244,
		JOINT_NOT_RESPONDING_MODE = 245,
		JOINT_MOTOR_INITIALISATION_MODE = 246,
		JOINT_ADC_CALIBRATION_MODE = 247,
		JOINT_DEAD_COMMUTATION_MODE = 248,
		JOINT_BOOTLOADER_MODE = 249,
		JOINT_CALIBRATION_MODE = 250,
		JOINT_SECURITY_STOPPED_MODE = 251,
		JOINT_FAULT_MODE = 252,
		JOINT_RUNNING_MODE = 253,
		JOINT_INITIALISATION_MODE = 254,
		JOINT_IDLE_MODE = 255

	};

	enum IoAnalogRange {
		IO_ANALOG_INPUT_RANGE_0_TO_5_VOLT = 0x00,
		IO_ANALOG_INPUT_RANGE_5_TO_5_VOLT = 0x01,
		IO_ANALOG_INPUT_RANGE_0_TO_10_VOLT = 0x02,
		IO_ANALOG_INPUT_RANGE_10_TO_10_VOLT = 0x03
	};

	struct RobotModeData {
		uint64_t timestamp;
		bool isRobotConnected;
		bool isRealRobotEnabled;
		bool isPowerOnRobot;
		bool isEmergencyStopped;
		bool isSecurityStopped;
		bool isProgramRunning;
		bool isProgramPaused;
		RobotMode robotMode;
		double speedFraction;
	};

	struct JointData {
		double q_actual;
		double q_target;
		double qd_actual;
		float I_actual;
		float V_actual;
		float T_motor;
		float T_micro;
		JointMode jointMode;
	};

	struct CartesianInfo {
		double x;
		double y;
		double z;
		double rx;
		double ry;
		double rz;
	};

	struct ToolData {
		IoAnalogRange analogInputRange2;
		IoAnalogRange analogInputRange3;
		double analogInput2;
		double analogInput3;
		float toolVoltage48V;
		uint8_t toolOutputVoltage;
		float toolCurrent;
		float toolTemperature;
	};

	struct MasterboardData {
		int16_t digitalInputBits;
		int16_t digitalOutputBits;
		IoAnalogRange analogInputRange0;
		IoAnalogRange analogInputRange1;
		double analogInput0;
		double analogInput1;
		int8_t analogOutputDomain0;
		int8_t analogOutputDomain1;
		double analogOutput0;
		double analogOutput1;
		float masterBoardTemperature;
		float robotVoltage48V;
		float robotCurrent;
		float masterIOCurrent;
		uint8_t masterSaftyState;
		uint8_t masterOnOffState;
	};

	struct ClientInterfaceData {
	public:
		RobotModeData robotModeData;
		CartesianInfo cartesianInfo;
		ToolData toolData;
		MasterboardData masterboardData;
		vector<JointData> jointData;

	public:
		std::string asString() const;
	};

public:
	static char* parse(std::shared_ptr<ClientInterfaceData> data, char* ptr, size_t len);

	static std::string robotModeToString(RobotMode mode);
	static std::string jointModeToString(JointMode mode);

private:
	static void parseRobotModeData(RobotModeData& robotModeData, char* ptr);
	static void parseJointData(vector<JointData>& jointData, char* ptr);
	static void parseCartesianInfo(CartesianInfo& cartesianInfo, char* ptr);
	static void parseToolData(ToolData& toolData, char* ptr);
	static void parseMasterboardData(MasterboardData& masterboardData, char* ptr);
	static void parseMessages(char* ptr);

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
