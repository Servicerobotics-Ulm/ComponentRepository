#ifndef _DRIVER_H_
#define _DRIVER_H_

#include "MathUtils.hh"
#include "ComponentRobotToRobotDocking.hh"

class StationDetector;

class Driver
{
public:
	Driver();

	void start( int belt );

//	bool update( double* vx, double* vy, double* omega, double x, double y, double phi, const StationDetector& stationDetector );
	bool update( double& vx, double& vy, double& omega, double x, double y, double phi ,const std::vector<double> & trackingGoal );
	bool isFinished() const;

private:
	int _belt;

	std::vector<Eigen::Vector2d> _omegaControl;
	std::vector<Eigen::Vector2d> _vxControl;
	std::vector<Eigen::Vector2d> _vyControl;

	typedef enum
	{
		Rotate,
		Align,
		Measure,
		Drive,
		Finished
	} State_t;

	State_t _state;

};

#endif //_DRIVER_H_
