#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_IR_I2C.h"
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_Proximity_IR_I2C::AP_Proximity_IR_I2C(AP_Proximity &_frontend, 
                                         AP_Proximity::Proximity_State &_state)
    : AP_Proximity_Backend(_frontend, _state)
    , _dev(hal.i2c_mgr->get_device(1, AP_RANGE_FINDER_IR_I2C_DEFAULT_ADDR))
    , _last_status(false)
    , _sector_initialised(false)
{
}

AP_Proximity_Backend *AP_Proximity_IR_I2C::detect(AP_Proximity &_frontend, 
                                                   AP_Proximity::Proximity_State &_state)
{
    AP_Proximity_IR_I2C *sensor = new AP_Proximity_IR_I2C(_frontend, _state);

    if (!sensor) {
        return nullptr;
    }

    if (sensor->_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        sensor->update();
	if (!sensor->get_last_status()) {
            sensor->_dev->get_semaphore()->give();
            delete sensor;
            return nullptr;
        }
        sensor->_dev->get_semaphore()->give();
    }

    sensor->initialise();

    return sensor;
}

// initialise sensor (returns true if sensor is succesfully initialised)
bool AP_Proximity_IR_I2C::initialise()
{
    if (!_sector_initialised) {
        init_sectors();
        return false;
    }

    return true;
}

// initialise sector angles using pre-defined ignore areas
void AP_Proximity_IR_I2C::init_sectors()
{
    uint8_t sector;

    if (_sector_initialised) {
        return;
    }

    _num_sectors = 4;
    for (sector=0; sector < _num_sectors; sector++ ) {
        _sector_middle_deg[sector] = 90*sector;
        _sector_width_deg[sector] = 40;
    }

    init_boundary();
    
    _sector_initialised = true;
}

void AP_Proximity_IR_I2C::update()
{
    uint8_t     buffer[6];

    if (!initialise()) {
        return;
    }

    _last_status = _dev->read_registers(SensorAddress_yaw_0, buffer, 6);
    if (_last_status) {
        _distance[SensorAddress_yaw_0] = buffer[AP_IR_I2C_NORTH];
        _distance_valid[SensorAddress_yaw_0] = true;
        update_boundary_for_sector(SensorAddress_yaw_0);

        _distance[SensorAddress_yaw_90] = buffer[AP_IR_I2C_EAST];
        _distance_valid[SensorAddress_yaw_90] = true;
        update_boundary_for_sector(SensorAddress_yaw_90);

        _distance[SensorAddress_yaw_180] = buffer[AP_IR_I2C_SOUTH];
        _distance_valid[SensorAddress_yaw_180] = true;
        update_boundary_for_sector(SensorAddress_yaw_180);

        _distance[SensorAddress_yaw_270] = buffer[AP_IR_I2C_WEST];
        _distance_valid[SensorAddress_yaw_270] = true;
        update_boundary_for_sector(SensorAddress_yaw_270);
    } else {
        _distance_valid[AP_IR_I2C_NORTH] = false;
        _distance_valid[AP_IR_I2C_SOUTH] = false;
        _distance_valid[AP_IR_I2C_WEST] = false;
        _distance_valid[AP_IR_I2C_EAST] = false;
    }

    set_status(AP_Proximity::Proximity_Good);
}
    
/* get distance upwards in meters. returns true on success
bool AP_Proximity_IR_I2C::get_upward_distance(float &distance) const 
{ 
    return false; 
}
*/

/* handle mavlink DISTANCE_SENSOR messages
void AP_Proximity_IR_I2C::handle_msg(mavlink_message_t *msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_DISTANCE_SENSOR) {
        return;
    }

    if (msg.sysid == mavlink_system.sysid) {
        // we do not obstruct ourselved...
        return;
    }

    mavlink_distance_sensor_t packet;
    mavlink_msg_distance_sensor_decode(msg, &packet);

    if (packet.len > MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN) {
        // invalid packet
        return;
    }

    if (ret_buffer == nullptr) {
    }
*/

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_IR_I2C::distance_max() const
{
    return AP_RANGE_FINDER_IR_I2C_MAX_DISTANCE;
}
float AP_Proximity_IR_I2C::distance_min() const
{
    return AP_RANGE_FINDER_IR_I2C_MIN_DISTANCE;
}
