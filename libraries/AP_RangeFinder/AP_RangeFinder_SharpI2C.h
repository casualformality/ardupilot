#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

#define AP_RANGE_FINDER_SHARPI2C_DEFAULT_ADDR   0x66

#define AP_RANGEFINDER_SHARPI2C                4
#define AP_RANGE_FINDER_SHARPI2C_MIN_DISTANCE  10
#define AP_RANGE_FINDER_SHARPI2C_MAX_DISTANCE  80

class AP_RangeFinder_SharpI2C : public AP_RangeFinder_Backend
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void);

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_INFRARED;
    }

private:
    // constructor
    AP_RangeFinder_SharpI2C(RangeFinder::RangeFinder_State &_state);

    bool _init(void);
    void _timer(void);

    uint16_t distance;
    bool new_distance;
    
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
