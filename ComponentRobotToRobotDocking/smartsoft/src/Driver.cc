#include "Driver.h"
#include <iostream>
#include "MathUtils.hh"
Driver::Driver()
: _state( Rotate ), _belt(0)
{
	_omegaControl.push_back(Eigen::Vector2d( 1, 0 ) );
	_omegaControl.push_back(Eigen::Vector2d( 1, 2 ) );
	_omegaControl.push_back(Eigen::Vector2d( 20, COMP->getGlobalState().getDocking().getLaserDocking_rotVel() ) );

	_vxControl.push_back(Eigen::Vector2d( 0.2+COMP->getGlobalState().getDocking().getLaserDocking_stopDistance(), 0 ) );
	_vxControl.push_back(Eigen::Vector2d( 0.2+COMP->getGlobalState().getDocking().getLaserDocking_stopDistance(), 0.02 ) );
	_vxControl.push_back(Eigen::Vector2d( 0.25+COMP->getGlobalState().getDocking().getLaserDocking_stopDistance(), 0.05 ) );
	_vxControl.push_back(Eigen::Vector2d( 0.4+COMP->getGlobalState().getDocking().getLaserDocking_stopDistance(), COMP->getGlobalState().getDocking().getLaserDocking_transVelX() ) );

	_vyControl.push_back(Eigen::Vector2d( 0.01, 0 ) );
	_vyControl.push_back(Eigen::Vector2d( 0.01, 0.02 ) );
	_vyControl.push_back(Eigen::Vector2d( 0.1, 0.1 ) );
	_vyControl.push_back(Eigen::Vector2d( 0.4, COMP->getGlobalState().getDocking().getLaserDocking_transVelY() ) );

}

void Driver::start( int belt )
{
	_belt = belt;
	_state = Rotate;

	std::cout << "Start -> Rotate" << std::endl;
}

bool Driver::isFinished() const
{
	return (Finished == _state);
}

//bool Driver::update( double* vx, double* vy, double* omega, double x, double y, double phi, const StationDetector& stationDetector )
bool Driver::update( double& vx, double& vy, double& omega, double x, double y, double phi, const std::vector<double>& trackingGoal)
{


	if( trackingGoal.empty() )
	{
		vx = 0;
		vy = 0;
		omega = 0;
		return false;
	}

	double diffX = trackingGoal[0];
	double diffY = trackingGoal[1];
	double diffAlpha = atan2(diffY,diffX);

	std::cout << "[Driver:] Diff x:" << diffX << " y: " << diffY << " a: " << rad2deg( diffAlpha ) << "deg" << std::endl;


	vx = linearapproximation( _vxControl, diffX );
	vy = linearapproximation( _vyControl, fabs( diffY ) );
	if( diffY < 0 )
	{
		vy = -vy;
	}

	omega = linearapproximation( _omegaControl, fabs( rad2deg( diffAlpha ) ) );

	omega = deg2rad( omega );
	if( diffAlpha < 0 )
	{
		omega = -omega;
	}

	std::cout << "[Driver:] vx=" << vx << "m/s  vy=" << vy << "m/s  omega=" << rad2deg( omega ) << "deg/s" << std::endl;

	switch( _state )
	{
		case Rotate:
			vx = 0;
			vy = 0;
			if( 0 == omega )
			{
				_state = Align;
				std::cout << "Rotate -> Align" << std::endl;
			}
			break;

		case Align:
			vx = 0;
			if( 0 == omega && 0 == vy )
			{
				_state = Measure;
				std::cout << "Align -> Measure" << std::endl;
			}
			break;

		case Measure:
			vx = 0;
			vy = 0;
			omega = 0;
//			if( scansStationsVisible - _numScansWithVisibleStationAtMeasureStart > 10 )
//			{
				_state = Drive;
				std::cout << "Measure -> Drive" << std::endl;
//			}
//			else
//			{
//				std::cout << "No new station seen" << std::endl;
//				return false;
//			}
			break;


		case Drive:
			if( 0 == omega && 0 == vx && 0 == vy )
			{
				_state = Finished;
				std::cout << "Drive -> Finished" << std::endl;
			}
			break;

		default:
			vx = 0;
			vy = 0;
			omega = 0;
			break;
	}

	return true;
}
