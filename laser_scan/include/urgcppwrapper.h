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

#ifndef URGCPPWRAPPER_H
#define URGCPPWRAPPER_H

#include <urg_cpp/Urg_driver.h>
#include <urg_cpp/ticks.h>
#include <sstream>
#include <stdexcept>
#include <iostream>

class URGCPPWrapper
{
public:  
    URGCPPWrapper(const std::string& ip = "192.168.0.10", const int ip_port = 10940);
    URGCPPWrapper(const int serial_baudrate, const std::string& serial_port);
    ~URGCPPWrapper();
    void start(bool use_intensity = true, bool use_multi_echo = false);
    void stop();

    void setDetectionAngle(int start, int end);
    void setDetectionAngleRadian(double start, double end);
    void setDetectionAngleDegree(double start, double end);

    void grabScan();
    void grabScanWithIntensity();
    void grabScanEcho();
    void grabScanEchoWithIntensity();

    void sync();

    // Getters
    std::string getAllInfo() const;
    bool isStarted() const;
    const std::vector<long>& getDistance() const;
    const std::vector<unsigned short>& getIntensity() const;
    long getTimeStamp() const;
    bool useIntensity() const;
    bool useMultiEcho() const;
    unsigned long int getNumberOfPoints() const;
    long getMinDistance() const;
    long getMaxDistance() const;

    double getAngleMinRadian() const;
    double getAngleMaxRadian() const;
    double getAngleMinDegree() const;
    double getAngleMaxDegree() const;

    // Constants
    double getAngleMinLimitRadian() const;
    double getAngleMaxLimitRadian() const;
    double getAngleMinLimitDegree() const;
    double getAngleMaxLimitDegree() const;

    // Utility
    double index2rad(int index) const;

private:
    qrk::Urg_driver urg;
    std::vector<long> distance;
    std::vector<unsigned short> intensity;
    long time_stamp;

    // Connection info
    std::string ip;
    int ip_port;
    int serial_baudrate;
    std::string serial_port;

    // States
    bool started;
    bool use_intensity;
    bool use_multi_echo;
    qrk::Lidar::measurement_type_t measurement_type;

    //
    int first_step;
    int last_step;

    void init();
    void initDataContainer();
    void setMeasurementType(bool use_intensity, bool use_multi_echo);

};

#endif // URGCPPWRAPPER_H
