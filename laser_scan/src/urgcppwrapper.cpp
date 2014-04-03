/*
* Copyright (c) 2014, Georgia Tech Research Corporation
* All rights reserved.
*
* Author: Paul Bernier <bernier.pja@gmail.com>
* Date: Jan 2014
*
* Humanoid Robotics Lab Georgia Institute of Technology
* Director: Mike Stilman http://www.golems.org
*
*
* This file is provided under the following "BSD-style" License:
* Redistribution and use in source and binary forms, with or
* without modification, are permitted provided that the following
* conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
*
* * Neither the name of the Humanoid Robotics Lab nor the names of
* its contributors may be used to endorse or promote products
* derived from this software without specific prior written
* permission
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
* USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "urgcppwrapper.h"

URGCPPWrapper::URGCPPWrapper(const std::string &ip, const int ip_port)
    : distance(0), intensity(0),
      ip(ip), ip_port(ip_port), serial_baudrate(0), serial_port(""),
      started(false), use_intensity(true), use_multi_echo(false),
      first_step(0), last_step(0)

{
    if (!urg.open(ip.c_str(), ip_port, qrk::Lidar::Ethernet))
    {
        // Error
        std::stringstream ss;
        ss << "Urg_driver::open(): ";
        ss << ip << ":" << ip_port << std::endl;
        ss << urg.what();
        throw std::runtime_error(ss.str());
    }

    init();
}

URGCPPWrapper::URGCPPWrapper(const int serial_baudrate, const std::string& serial_port)
    : distance(0), intensity(0),
      ip(""), ip_port(0), serial_baudrate(serial_baudrate), serial_port(serial_port),
      started(false), use_intensity(true), use_multi_echo(false),
      first_step(0), last_step(0)

{
    if (!urg.open(serial_port.c_str(), serial_baudrate, qrk::Lidar::Serial))
    {
        // Error
        std::stringstream ss;
        ss << "Urg_driver::open(): ";
        ss << serial_port << ":" << serial_baudrate << std::endl;
        ss << urg.what();
        throw std::runtime_error(ss.str());
    }

    init();
}

void URGCPPWrapper::init()
{
    first_step = urg.min_step();
    last_step = urg.max_step();
    initDataContainer();
}

URGCPPWrapper::~URGCPPWrapper()
{
    stop();
    urg.close();
}

void URGCPPWrapper::start(bool use_intensity, bool use_multi_echo)
{
    if(!started){
        setMeasurementType(use_intensity, use_multi_echo);

        if(!urg.start_measurement(measurement_type))
        {
            std::stringstream ss;
            ss << "Urg_driver::start_measurement(): ";
            ss << urg.what();
            throw std::runtime_error(ss.str());
        }

        started = true;
    }
}

void URGCPPWrapper::stop()
{
    if(started){
        urg.stop_measurement();
        started = false;
    }
}

void URGCPPWrapper::grabScan()
{
    if (!urg.get_distance(distance, &time_stamp)) {
        std::stringstream ss;
        ss << "Urg_driver::get_distance(): " << urg.what() << std::endl;
        throw std::runtime_error(ss.str());
    }
}

void URGCPPWrapper::grabScanWithIntensity()
{
    if (!urg.get_distance_intensity(distance, intensity, &time_stamp)) {
        std::stringstream ss;
        ss << "Urg_driver::get_distance_intensity(): " << urg.what() << std::endl;
        throw std::runtime_error(ss.str());
    }
}

void URGCPPWrapper::grabScanEcho()
{
    if (!urg.get_multiecho(distance, &time_stamp)) {
        std::stringstream ss;
        ss << "Urg_driver::get_distance(): " << urg.what() << std::endl;
        throw std::runtime_error(ss.str());
    }
}

void URGCPPWrapper::grabScanEchoWithIntensity()
{
    if (!urg.get_multiecho_intensity(distance, intensity, &time_stamp)) {
        std::stringstream ss;
        ss << "Urg_driver::get_multiecho_intensity(): " << urg.what() << std::endl;
        throw std::runtime_error(ss.str());
    }
}

void URGCPPWrapper::sync()
{
    if(!urg.set_sensor_time_stamp(qrk::ticks())) {
        std::stringstream ss;
        ss << "Urg_driver::set_sensor_time_stamp(): " << urg.what() << std::endl;
        throw std::runtime_error(ss.str());
    }
}

void URGCPPWrapper::setDetectionAngle(int start, int end)
{
    if(start < urg.min_step() || end > urg.max_step() || start >= end)
    {
        std::cerr << "URGCPPWrapper::setDetectionAngle Invalid start/end parameters" << std::endl;
    }
    else
    {
        if(urg.set_scanning_parameter(start, end))
        {
            first_step = start;
            last_step = end;
            initDataContainer();
        }
        else
        {
            std::cerr << "URGCPPWrapper::setDetectionAngle Error while trying to set detection angle" << std::endl;
        }
    }
}

void URGCPPWrapper::setDetectionAngleRadian(double start, double end)
{
    setDetectionAngle(urg.rad2step(start),urg.rad2step(end));
}

void URGCPPWrapper::setDetectionAngleDegree(double start, double end)
{
    setDetectionAngle(urg.deg2step(start),  urg.deg2step(end));
}

double URGCPPWrapper::getAngleMinRadian() const
{
    return urg.step2rad(first_step);
}
double URGCPPWrapper::getAngleMaxRadian() const
{
    return urg.step2rad(last_step);
}
double URGCPPWrapper::getAngleMinDegree() const
{
    return urg.step2deg(first_step);
}
double URGCPPWrapper::getAngleMaxDegree() const
{
    return urg.step2deg(last_step);
}

double URGCPPWrapper::getAngleMinLimitRadian() const
{
    return urg.step2rad(urg.min_step());
}

double URGCPPWrapper::getAngleMaxLimitRadian() const
{
    return urg.step2rad(urg.max_step());
}

double URGCPPWrapper::getAngleMinLimitDegree() const
{
    return urg.step2deg(urg.min_step());
}

double URGCPPWrapper::getAngleMaxLimitDegree() const
{
    return urg.step2deg(urg.max_step());
}

void URGCPPWrapper::setMeasurementType(bool use_intensity, bool use_multi_echo)
{
    if(!started)
    {
        this->use_intensity = use_intensity;
        this->use_multi_echo = use_multi_echo;

        if(use_intensity && use_multi_echo){
            measurement_type = qrk::Lidar::Multiecho_intensity;
        }else if(use_intensity){
            measurement_type = qrk::Lidar::Distance_intensity;
        }else if(use_multi_echo){
            measurement_type = qrk::Lidar::Multiecho;
        }else{
            measurement_type = qrk::Lidar::Distance;
        }

    }else{
        std::cerr << "Measurement already running. You must stop measurement before to change measurement type." << std::endl;
    }
}

void URGCPPWrapper::initDataContainer()
{
    const int data_size = urg.max_data_size();
    const int echo_size = urg.max_echo_size();

    distance.clear();
    intensity.clear();
    distance.reserve(data_size * echo_size);
    intensity.reserve(data_size * echo_size);
}

/***************
 *
 * Getters
 *
 ***************/

std::string URGCPPWrapper::getAllInfo() const
{
    std::stringstream ss;
    ss << "Sensor product type: " << urg.product_type() << std::endl;
    ss << "Sensor firmware version: " << urg.firmware_version() << " (last version is 1.1.8 ?)" << std::endl;
    ss << "Sensor serial ID: " << urg.serial_id() << std::endl;
    ss << "Sensor status: " << urg.status() << std::endl;
    ss << "Sensor state: " << urg.state() << std::endl;

    ss << "step limits: ["
       << urg.min_step() << ", "
       << urg.max_step() << "]" << std::endl;

    ss << "fixed steps: ["
       << first_step << ", "
       << last_step << "]" << std::endl;

    ss << "distance: ["
       << urg.min_distance()
       << ", " << urg.max_distance() << "]" << std::endl;

    ss << "scan interval: " << urg.scan_usec() << " [usec]" << std::endl;
    ss << "sensor data size: " << getNumberOfPoints() << std::endl;

    return ss.str();
}

bool URGCPPWrapper::isStarted() const
{
    return started;
}

const std::vector<long>& URGCPPWrapper::getDistance() const
{
    return distance;
}

const std::vector<unsigned short>& URGCPPWrapper::getIntensity() const
{
    return intensity;
}

long URGCPPWrapper::getTimeStamp() const
{
    return time_stamp;
}

bool URGCPPWrapper::useIntensity() const
{
    return use_intensity;
}

bool URGCPPWrapper::useMultiEcho() const
{
    return use_multi_echo;
}

unsigned long int URGCPPWrapper::getNumberOfPoints() const
{
    return last_step - first_step + 1;
}

long URGCPPWrapper::getMinDistance() const
{
    return urg.min_distance();
}

long URGCPPWrapper::getMaxDistance() const
{
    return urg.max_distance();
}

double URGCPPWrapper::index2rad(int index) const
{
    return urg.index2rad(index);
}
