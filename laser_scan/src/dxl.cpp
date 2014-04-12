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

#include "dxl.h"

Dxl::Dxl(unsigned int device_index, unsigned int baudrate)
    : device_index(device_index), baudrate(baudrate)
{
    if(!dxl_initialize(device_index, baudrate))
    {
        // Error
        std::stringstream ss;
        ss << "dxl_initialize() failed."<<  std::endl;
    }
    else
    {
        std::cout << "Connection to DXL device is ok." << std::endl;
    }
}

Dxl::~Dxl()
{
    dxl_terminate();
}

/*******************
 *
 * Move
 *
 *******************/

void Dxl::moveToPosition(int position, unsigned int device_id)
{
    dxl_write_word(device_id, GOAL_POSITION_L, position);
}

void Dxl::moveToDegree(double degree, unsigned int device_id)
{
    moveToPosition(degree / POSITION_TO_DEGREE, device_id);
}

void Dxl::moveToRadian(double radian, unsigned int device_id)
{
    moveToPosition(radian / POSITION_TO_RADIAN, device_id);
}

void Dxl::setSpeed(int speed, unsigned int device_id)
{
    dxl_write_word(device_id, MOVING_SPEED_L, speed);
}

/*******************
 *
 * Get
 *
 *******************/

int Dxl::isMoving(unsigned int device_id) const
{
    int moving = dxl_read_byte(device_id, MOVING);

    if(commRXIsOk())
    {
        return moving;
    }

    return false;
}

double Dxl::getCurrentAngleDegree(unsigned int device_id) const
{
    return getCurrentPosition(device_id) * POSITION_TO_DEGREE;
}

double Dxl::getCurrentAngleRadian(unsigned int device_id) const
{
    return getCurrentPosition(device_id) * POSITION_TO_RADIAN;
}

int Dxl::getCurrentPosition(unsigned int device_id) const
{
    const int position = dxl_read_word(device_id, PRESENT_POSITION_L);

    if(commRXIsOk())
    {
        return position;
    }

    return -1;
}

int Dxl::getMovingSpeed(unsigned int device_id) const
{
    const int speed = dxl_read_word(device_id, MOVING_SPEED_L);

    if(commRXIsOk())
    {
        return speed;
    }

    return -1;
}

/*******************
 *
 * Check
 *
 *******************/

bool Dxl::commRXIsOk() const
{
    const int status = dxl_get_result();

    if(status == COMM_RXSUCCESS)
    {
        return true;
    }

    return false;
}

const char* Dxl::get_comm_status(int status) const
{

    switch(status)
    {
    case COMM_TXFAIL:
        return "COMM_TXFAIL: Failed transmit instruction packet!";
        break;

    case COMM_TXERROR:
        return "COMM_TXERROR: Incorrect instruction packet!";
        break;

    case COMM_RXFAIL:
        return "COMM_RXFAIL: Failed get status packet from device!";
        break;

    case COMM_RXWAITING:
        return "COMM_RXWAITING: Now recieving status packet!";
        break;

    case COMM_RXTIMEOUT:
        return "COMM_RXTIMEOUT: There is no status packet!";
        break;

    case COMM_RXCORRUPT:
        return "COMM_RXCORRUPT: Incorrect status packet!";
        break;

    default:
        return "This is unknown error code!";
        break;
    }

}
