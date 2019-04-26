/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008-2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "driver.h"

#include <assert.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <boost/thread.hpp>

#include "ace/ACE.h"

#include "hokuyo.h"

#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;

class HokuyoDriverConfig
{
public:
	HokuyoDriverConfig()
		: intensity( false )
		, min_angle( -M_PI/2 )
		, max_angle( M_PI/2 )
		, cluster( 0 )
		, skip( 0 )
		, count( 0 )
		, timeout( 0 )
		, port( "/dev/scanner_front" )
	{
	}

	bool intensity;
	double min_angle;
	double max_angle;
	int cluster;
	int skip;
	int count;
	int timeout;
	std::string port;
};

class HokuyoDriver : public driver_base::Driver
{
private:
	typedef boost::function<void(const hokuyo::LaserScan &)> UseScanFunction;
	UseScanFunction useScan_;

	boost::shared_ptr<boost::thread> scan_thread_;

	std::string device_status_;
	std::string device_id_;
	std::string last_seen_device_id_;

	bool first_scan_;

	std::string vendor_name_;
	std::string product_name_;
	std::string protocol_version_;
	std::string firmware_version_;

	std::string connect_fail_;

	hokuyo::LaserScan  scan_;
	hokuyo::Laser laser_;
	hokuyo::LaserConfig laser_config_;

	bool calibrated_;
	int lost_scan_thread_count_;
	int corrupted_scan_count_;
	
	HokuyoDriverConfig config_;

	void driverDebugMessage( const std::string& message );

	void driverWarnMessage( const std::string& message );

public:

	virtual void infoMessage( const std::string& message );

	virtual void warnMessage( const std::string& message );

	virtual void debugMessage( const std::string& message );

	virtual void errorMessage( const std::string& message );

	virtual void hokuyoScanCallback( const hokuyo::LaserScan& scan );

	HokuyoDriver();

	bool checkAngleRange( HokuyoDriverConfig& newConfig );

	void doOpen();

	void doClose();

	void doStart();

	void doStop();

	virtual std::string getID();

	void config_update( HokuyoDriverConfig &new_config, int level = 0);

	const HokuyoDriverConfig& config() const { return config_; }

	void scanThread();
};

