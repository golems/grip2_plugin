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

#ifndef DXL_H
#define DXL_H

#include <dynamixel.h>
#include <sstream>
#include <stdexcept>
#include <iostream>

class Dxl
{
public:
    // RAM address
    static const unsigned char GOAL_POSITION_L = 30;
    static const unsigned char GOAL_POSITION_H = 31;

    static const unsigned char MOVING_SPEED_L = 32;
    static const unsigned char MOVING_SPEED_H = 33;

    static const unsigned char PRESENT_POSITION_L = 36;
    static const unsigned char PRESENT_POSITION_H = 37;
    static const unsigned char MOVING = 46;

    // Constants
    constexpr static double DEFAULT_ID = 1;
    constexpr static double POSITION_TO_DEGREE = 0.088;
    constexpr static double POSITION_TO_RADIAN = 0.001535889742;

    Dxl(unsigned int device_index = 0, unsigned int baudrate = 1);
    ~Dxl();

    // Move
    void moveToPosition(int position, unsigned int device_id = DEFAULT_ID);
    void moveToDegree(double degree, unsigned int device_id = DEFAULT_ID);
    void moveToRadian(double radian, unsigned int device_id = DEFAULT_ID);

    void setSpeed(int speed, unsigned int device_id = DEFAULT_ID);

    // Get
    int isMoving(unsigned int device_id = DEFAULT_ID) const;
    double getCurrentAngleDegree(unsigned int device_id = DEFAULT_ID) const;
    double getCurrentAngleRadian(unsigned int device_id = DEFAULT_ID) const;
    int getCurrentPosition(unsigned int device_id = DEFAULT_ID) const;
    int getMovingSpeed(unsigned int device_id = DEFAULT_ID) const;

private:
    unsigned int device_index;//device index x in /dev/ttyUSBx
    unsigned int baudrate;

    const char* get_comm_status(int status) const;
    bool commRXIsOk() const;
    bool commTXIsOk() const;

};

#endif // DXL_H
