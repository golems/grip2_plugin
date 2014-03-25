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
    unsigned int device_index;//device index is x in /dev/ttyUSBx
    unsigned int baudrate;

    const char* get_comm_status(int status) const;
    bool commRXIsOk() const;
    bool commTXIsOk() const;

};

#endif // DXL_H
