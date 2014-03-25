#include "dxl.h"

Dxl::Dxl(unsigned int device_index, unsigned int baudrate)
    : device_index(device_index), baudrate(baudrate)
{
    if(!dxl_initialize(device_index, baudrate))
    {
        // Error
        std::stringstream ss;
        ss << "dxl_initialize() failed."<<  std::endl;
        throw std::runtime_error(ss.str());
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
    else
    {
        throw std::runtime_error(get_comm_status(status));
    }
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
