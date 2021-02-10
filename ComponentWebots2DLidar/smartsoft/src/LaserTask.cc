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
//------------------------------------------------------------------------
//
//  Copyright (C) 2010 Manuel Wopfner
//                2020 Thomas Feldmeier
//
//        thomas.feldmeier@thu.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartLaserLMS100Server component".
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
//--------------------------------------------------------------------------
#include "LaserTask.hh"
#include "ComponentWebots2DLidar.hh"
#include <EulerTransformationMatrices.hh>

#include <climits>
#include <cstdint>
#include <iostream>

#include <webots/Device.hpp>
#include <webots/Node.hpp>

#include <iostream>

LaserTask::LaserTask(SmartACE::SmartComponent *comp) 
:	LaserTaskCore(comp)
{
    std::cout << "constructor LaserTask\n";
    _default_base_position.set_x(COMP->getGlobalState().getBase_manipulator().getX());
    _default_base_position.set_y(COMP->getGlobalState().getBase_manipulator().getY());
    _default_base_position.set_z(COMP->getGlobalState().getBase_manipulator().getZ());
    _default_base_position.set_base_azimuth(COMP->getGlobalState().getBase_manipulator().getBase_a());
    //_default_base_position.set_steer_alpha(COMP->getGlobalState().getBase_manipulator().getSteer_a());
    //_default_base_position.set_turret_alpha(COMP->getGlobalState().getBase_manipulator().getTurret_a());
}

LaserTask::~LaserTask() 
{
    std::cout << "destructor LaserTask\n";
}

int LaserTask::on_entry()
{
    std::string robotName = COMP->getParameters().getWebots().getRobotName();
    char environment[256] = "WEBOTS_ROBOT_NAME=";
    putenv(strcat(environment, robotName.c_str()));
    webotsRobot = new webots::Robot();
    if(!webotsRobot) {
        std::cerr << "Webots robot '" << robotName << "' not found" << std::endl;
        return -1;
    }

    webotsTimeStep = webotsRobot->getBasicTimeStep();
    // TODO: do something so timestep and lidar frequency matches better
    //       old lidar data is bad for performance

    // get timestep from the world and match the one in SmartMDSD component
    // int coeff = S_TO_MS / (webotsTimeStep * COMP->connections.laserTask.periodicActFreq);
    // webotsTimeStep *= coeff;

    // connect to the sensor from Webots
    webotsLidar = NULL;
    for (int i = 0; i < webotsRobot->getNumberOfDevices(); i++)
    {
      webots::Device *webotsDevice = webotsRobot->getDeviceByIndex(i);

      if (webotsDevice->getNodeType() == webots::Node::LIDAR)
      {
        std::string lidarName = webotsDevice->getName();
        webotsLidar = webotsRobot->getLidar(lidarName);
        webotsLidar->enable(webotsTimeStep);
        webotsLidar->enablePointCloud();
        std::cout << "Device #" << i << " called " << lidarName << " is a lidar." << std::endl;
        // set Webots sensor's properties to SmartMDSD model
        // useful doc:
        // http://servicerobotik-ulm.de/drupal/doxygen/components_commrep/classCommBasicObjects_1_1CommMobileLaserScan.html
        horizontalResolution = webotsLidar->getHorizontalResolution();
        numberValidPoints = webotsLidar->getNumberOfPoints();
        double hfov_deg = webotsLidar->getFov()* 180/M_PI;
        scan.set_scan_size(numberValidPoints);
        scan.set_max_scan_size(numberValidPoints);
        scan.set_scan_integer_field_of_view(-horizontalResolution * UNIT_FACTOR / 2.0, horizontalResolution * UNIT_FACTOR);


        std::cout << " horizontalResolution : " << horizontalResolution <<std::endl;
        std::cout << " numberValidPoints : " << numberValidPoints <<std::endl;
        std::cout << " hfov_deg : " << hfov_deg <<std::endl;
      //scan.set_scan_integer_field_of_view(-1* hfov_deg * UNIT_FACTOR / 2.0, (hfov_deg/numberValidPoints)*UNIT_FACTOR);
        // pay attention to limits as min/max_distance variables are short type (max value is 65535)
        if (webotsLidar->getMaxRange() * M_TO_MM > SHORT_LIMIT)
        {
          std::cout << "The lidar range is bigger than 65.535 meters and will be set to 65 meters." << std::endl;
          scan.set_max_distance(65 * M_TO_MM);
        }
        else
        {
          scan.set_max_distance(webotsLidar->getMaxRange() * M_TO_MM);
        }

        std::cout << "laser max :" << webotsLidar->getMaxRange() <<std::endl;
        std::cout << "laser min :" << webotsLidar->getMinRange() <<std::endl;
        scan.set_min_distance(webotsLidar->getMinRange() * M_TO_MM);
        scan.set_scan_length_unit(MEASURE_UNIT);
        break;
      }
    }

    if (!webotsLidar)
    {
      std::cout << "No lidar found, no data is sent." << std::endl;
      return -1;
    }

    zero_velocity.set_vX(0);
    zero_velocity.set_WX_base(0);;

    min_range = COMP->getGlobalState().getScanner().getMin_range();
    max_range = COMP->getGlobalState().getScanner().getMax_range();
    opening_angle = COMP->getGlobalState().getScanner().getOpening_angle();
    resolution = COMP->getGlobalState().getScanner().getResolution();
    length_unit = COMP->getGlobalState().getScanner().getLength_unit();
    frequency = COMP->getGlobalState().getScanner().getFrequency();


    std::cout <<"-----------------------------------------------------------------"<<std::endl;
    std::cout <<std::setw(40)<<"Laser Parameters"<<std::endl;
    std::cout <<"-----------------------------------------------------------------"<<std::endl;
    std::cout <<std::setw(25)<<  "Number of rays"    <<" = " << opening_angle/resolution<<std::endl;
    std::cout <<std::setw(25)<<  "Horizontal Field of View"  <<" = " << opening_angle << " degrees"<<std::endl;
    std::cout <<std::setw(25)<<  "Angle resolution"  <<" = " << resolution<< " degrees"<<std::endl;
    std::cout <<std::setw(25)<<  "Start_angle"       <<" = " << -0.5*opening_angle <<std::endl;
    std::cout <<std::setw(25)<<  "Max distance"      <<" = " << min_range/1000.0<< " meters"<<std::endl;
    std::cout <<std::setw(25)<<  "Min distance"      <<" = " << max_range/1000.0<< " meters"<<std::endl;
    std::cout <<"-----------------------------------------------------------------"<<std::endl;

    return 0;
}

/* value                              webots                       smartsoft
 * horizontal field of view           getFov() [radians]           opening_angle [degrees]
 * number of horizontal points        getHorizontalResolution()    -
 * angle between 2 horizontal points  -                            resolution [degrees]
 *
 * If in smartsoft these values are smaller than the correspondent values in webots,
 * only this part of the scanned points are returned.
 * e.g. if in webots the Lidar has a horizontal field of view of 3.14 radians
 *      but in smartsoft the opening_angle is set to 120 degrees, some points at the left/right side are not returned.
 *      resolution can be reduced too.
 *
 * note: webots calculates the number of rays = opening_angle / resolution,
 *       but real lidar (e.g. Sick LMS 200) and smartsoft use 1 + opening_angle / resolution
 *       e.g. opening_angle = 180, resolution = 0.5 => max. number data points = 361
 *
 * what happens if there is no obstacle between minRange and maxRange of the lidar ray?
 *   webots will return infinity as distance (even in case obstacle < minRange)
 *   smartsoft will handle this data point as 'invalid', removing this point from CommMobileLaserScan
 *   (get_scan_angle(i) returns the angle of an data point, 0=front of lidar, pi/2=left of lidar, 3*pi/2=right of lidar, 0<=angle<2*pi)
 *
 * In webots scan points are from left to right, but in smartsoft from right to left ordered.
 *
 * In webots distances are floating point numbers in m, but in smartsoft lidar distances are unsigned short (16 bit) in mm
 * (=> max. distance 65535 mm, you can set length_unit to e.g. 10 to change from mm to cm so lidar has bigger maxRange but less accuracy)
 */
int LaserTask::on_execute()
{
    while(webotsRobot->step(webotsTimeStep)!=-1) {
        const int allScans = webotsLidar ->getHorizontalResolution();
        const float *_allData = webotsLidar->getRangeImage();
        float allData[allScans];
        for(int i=allScans; i--;)
            allData[i] = _allData[allScans - 1 - i]*M_TO_MM / length_unit;
//        double maxRange = webotsLidar->getMaxRange()*M_TO_MM / length_unit;
        double ranger_opening_angle = webotsLidar->getFov() / M_PI * 180;

        if (COMP->getGlobalState().getScanner().getVerbose())
            std::cout << "set scan parameters\n";

        const int desiredScans = 1 + opening_angle / resolution;
        const int rangerScans = 1 + ranger_opening_angle / resolution;

        int fewerScans = rangerScans;
        if(fewerScans > allScans)
            fewerScans = allScans;
        float fewerData[fewerScans];
        for(int i=0; i<fewerScans; i++)
            fewerData[i] = allData[i*allScans/fewerScans];

        const int firstScanIndex = (rangerScans - desiredScans) * 0.5;
        int lastScanIndex = rangerScans - firstScanIndex;

        if (lastScanIndex > fewerScans)
            lastScanIndex = fewerScans;

        if (COMP->getGlobalState().getScanner().getVerbose()) {
            std::cout << "Read scans: " << fewerScans << "\n";
        }

        int num_valid_points = 0;
        for (int i = firstScanIndex; i < lastScanIndex; ++i) {
            int dist = fewerData[i];
            if (dist >= min_range && dist <= max_range) {
                ++num_valid_points;
            }
        }
        if (COMP->getGlobalState().getScanner().getVerbose())
          std::cout << "valid_points:" << num_valid_points << " allScans:" << allScans << " desiredScans:" << desiredScans << " rangerScans:" << rangerScans << " resolution:" << resolution << std::endl;

        CommBasicObjects::CommTimeStamp lastTimeStep = CommBasicObjects::CommTimeStamp::now();
        // webots is updating the physics world only every timeStep,
        // so scanning was done timeStep ms ago
        lastTimeStep.advance(-webotsTimeStep/1000.0);
//        scan.set_scan_time_stamp(CommBasicObjects::CommTimeStamp::now().advance() );
        scan.set_scan_time_stamp(lastTimeStep);

        scan.set_scan_update_count(scanCounter++);
        scan.set_scan_length_unit(length_unit);
        scan.set_scan_double_field_of_view(-0.5 * opening_angle, resolution);
        scan.set_min_distance(min_range);
        scan.set_max_distance(max_range);
        scan.set_max_scan_size(desiredScans);
        scan.set_scan_size(num_valid_points);

        int valid_point_index = 0;
        for (int i = firstScanIndex; i < lastScanIndex; ++i) {
            const unsigned int dist = fewerData[i];
            if (dist >= min_range && dist <= max_range) {
                //this line will cut of the starting beam if the component is configures to provide smaller scans (less opening angle)
                scan.set_scan_index(valid_point_index, i - firstScanIndex);
                scan.set_scan_integer_distance(valid_point_index, dist);
                scan.set_scan_intensity(valid_point_index, 0);
                ++valid_point_index;
            }
        }
        scan.set_scan_valid(true);

        bool scan_is_valid = false;

        if (COMP->getGlobalState().getScanner().getVerbose())
            std::cout << "[LaserTask] set base state\n";

        // read base state from base server
        if (COMP->getGlobalState().getBase_manipulator().getOn_base()) {
            Smart::StatusCode status = COMP->baseStateServiceIn->getUpdate(base_state);
            if (status == Smart::SMART_OK) {
                scan_is_valid = true;

                if (COMP->getGlobalState().getScanner().getVerbose()) {
                    //std::cout << base_state << "\n";
                    std::cout << "Odom from Base State : " << base_state.get_base_raw_position() <<std::endl;
                    std::cout << "POse from Base State : " << base_state.get_base_position() <<std::endl;
                }
            } else {
                std::cerr << "[LaserTask] WARNING: failed to get current base state ("
                        << Smart::StatusCodeConversion(status) << "), pushing invalid scan" << std::endl;
                scan.set_scan_valid(false);
            }
        }
        // default base state
        else {
            base_state.set_time_stamp(CommBasicObjects::CommTimeStamp::now());
            base_state.set_base_position(_default_base_position);
            base_state.set_base_velocity(zero_velocity);
            scan_is_valid = true;
        }
        scan.set_base_state(base_state);

        //////////////////////////
        // set robot scanner position
        //////////////////////////
        double x = COMP->getGlobalState().getScanner().getX();
        double y = COMP->getGlobalState().getScanner().getY();
        double z = COMP->getGlobalState().getScanner().getZ();
        double azimuth = COMP->getGlobalState().getScanner().getAzimuth();
        double elevation = COMP->getGlobalState().getScanner().getElevation();
        double roll = COMP->getGlobalState().getScanner().getRoll();
        arma::mat mat_sensor;

        EulerTransformationMatrices::create_zyx_matrix(x, y, z, azimuth, elevation, roll, mat_sensor);

        CommBasicObjects::CommPose3d sensor_pose(mat_sensor);
        scan.set_sensor_pose(sensor_pose);

        //////////////////////////
        // set world scanner position
        //////////////////////////
        double base_x = 0;
        double base_y = 0;
        double base_z = 0;
        double base_a = 0;
        arma::mat mat_base(4, 4);

        if(scanCounter==0)
        	lastBasePosition = base_state.get_base_position();

        if (scan_is_valid) {
            base_x = lastBasePosition.get_x();
            base_y = lastBasePosition.get_y();
            base_z = lastBasePosition.get_z();
            if (COMP->getGlobalState().getScanner().getOn_turret()) {
                //base_a = base_state.get_base_position().get_turret_alpha();
            } else {
                base_a = lastBasePosition.get_base_azimuth();
            }
        }
        // use robot's position of last physics timeStep instead of the actual timeStep,
        // because lidar data was generated at last timeStep too
        lastBasePosition = base_state.get_base_position();

        EulerTransformationMatrices::create_zyx_matrix(base_x, base_y, base_z, base_a, 0, 0, mat_base);
        arma::mat mat_world = mat_base * mat_sensor;
        CommBasicObjects::CommPose3d world_pose(mat_world);

        scan.set_scanner_x(world_pose.get_x());
        scan.set_scanner_y(world_pose.get_y());
        scan.set_scanner_z(world_pose.get_z());

        scan.set_scanner_azimuth(world_pose.get_azimuth());
        scan.set_scanner_elevation(world_pose.get_elevation());
        scan.set_scanner_roll(world_pose.get_roll());

        //////////////////////////
        // send scan to clients
        //////////////////////////
        if (COMP->getGlobalState().getScanner().getVerbose())
            std::cout << "[LaserTask] send scan to clients\n";

        if (COMP->getGlobalState().getServices().getActivate_push_newest()) {
            Smart::StatusCode push_status = COMP->laserServiceOut->put(scan);
            if (push_status != Smart::SMART_OK) {
                std::cerr << "[LaserTask] WARNING: error on push (" << Smart::StatusCodeConversion(push_status)
                << ")" << std::endl;
            }
        }

        // copy local scan to global scan
        SmartACE::SmartGuard scan_guard(COMP->ScanLock);
        COMP->global_scan = scan;
        scan_guard.release();

        if (COMP->getGlobalState().getScanner().getVerbose()) {
            const unsigned int index = scan.get_scan_size() / 2;
//            std::cout << "[LaserTask] Scan " << scanCounter << " sent." << " Scan Position " << index << "/"
//                    << scan.get_scan_size() << " = " << scan.get_scan_distance(index - 1) << " mm" << std::endl;
        }
    }
    return 1; // ends this thread after webots world ends
}

int LaserTask::on_exit()
{
    std::cout << "[LaserTask] Disconnect from laser" << std::endl;
    return 0;
}