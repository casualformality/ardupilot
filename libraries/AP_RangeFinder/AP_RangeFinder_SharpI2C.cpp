/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_RangeFinder_MaxsonarI2CXL.cpp - Arduino Library for MaxBotix I2C XL sonar
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       datasheet: http://www.maxbotix.com/documents/I2CXL-MaxSonar-EZ_Datasheet.pdf
 *
 *       Sensor should be connected to the I2C port
 */
#include "AP_RangeFinder_SharpI2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_SharpI2C::AP_RangeFinder_SharpI2C(RangeFinder::RangeFinder_State &_state)
    : AP_RangeFinder_Backend(_state)
    , _dev(hal.i2c_mgr->get_device(1, AP_RANGE_FINDER_SHARPI2C_DEFAULT_ADDR))
{
}

AP_RangeFinder_Backend *AP_RangeFinder_SharpI2C::detect(RangeFinder::RangeFinder_State &_state)
{
    AP_RangeFinder_SharpI2C *sensor
        = new AP_RangeFinder_SharpI2C(_state);
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_SharpI2C::_init(void)
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    uint16_t reading_cm;
    if (!get_reading(reading_cm)) {
        _dev->get_semaphore()->give();
        return false;
    }

    _dev->get_semaphore()->give();
    
    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_SharpI2C::_timer, void));
    
    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_SharpI2C::get_reading(uint16_t &reading_cm)
{
    uint8_t val;

    // take range reading and read back results
    bool ret = _dev->transfer(nullptr, 0, (uint8_t *) &val, sizeof(val));

    if (ret) {
        // combine results into distance
        reading_cm = val;
    }

    return ret;
}

/*
  timer called at 20Hz
*/
void AP_RangeFinder_SharpI2C::_timer(void)
{
    uint16_t d;
    if (get_reading(d)) {
        if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            distance = d;
            new_distance = true;
            _sem->give();
        }
    }
}


/*
   update the state of the sensor
*/
void AP_RangeFinder_SharpI2C::update(void)
{
    if (_sem->take_nonblocking()) {
        if (new_distance) {
            state.distance_cm = distance;
            new_distance = false;
            update_status();
        } else {
            set_status(RangeFinder::RangeFinder_NoData);
        }
         _sem->give();
    }
}
