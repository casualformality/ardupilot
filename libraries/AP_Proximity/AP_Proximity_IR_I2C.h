#pragma once

#include <AP_HAL/I2CDevice.h>
#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define AP_RANGE_FINDER_IR_I2C_DEFAULT_ADDR     0x29
#define AP_RANGE_FINDER_IR_I2C_MAX_DISTANCE     0.80f
#define AP_RANGE_FINDER_IR_I2C_MIN_DISTANCE     0.20f

#define AP_IR_I2C_CHK_BYTE  0x29

#define AP_IR_I2C_CHK       0x00
#define AP_IR_I2C_NORTH     0x01
#define AP_IR_I2C_SOUTH     0x02
#define AP_IR_I2C_EAST      0x03
#define AP_IR_I2C_WEST      0x04
#define AP_IR_I2C_CHK_INV   0x05

class AP_Proximity_IR_I2C : public AP_Proximity_Backend
{

public:
    // static detection function
    static AP_Proximity_Backend *detect(AP_Proximity &_frontend, 
                                        AP_Proximity::Proximity_State &_state);

    // update state
    void update();

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const;
    float distance_min() const;

    // handle mavlink DISTANCE_SENSOR messages
//    void handle_msg(mavlink_message_t *msg)

    bool get_last_status() {return _last_status;}

    bool initialise();
    void init_sectors();

protected:


private:
    typedef enum {
        SensorAddress_yaw_0     = 0,
        SensorAddress_yaw_90    = 1,
        SensorAddress_yaw_180   = 2,
        SensorAddress_yaw_270   = 3,
    } SensorAddress;

    // constructor
    AP_Proximity_IR_I2C(AP_Proximity &_frontend, 
                        AP_Proximity::Proximity_State &_state);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    bool _last_status;
    bool _sector_initialised;
};
