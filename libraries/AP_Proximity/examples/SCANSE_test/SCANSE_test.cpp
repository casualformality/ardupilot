/*
 * ProximitySensor test code
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Proximity/AP_Proximity.h>

#include "orientation.h"

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Serial manager is needed for UART comunications
static AP_SerialManager serial_manager;
static AP_Proximity sensor {serial_manager};

void setup()
{
    // print welcome message
#if HAL_OS_POSIX_IO
    ::printf("SCANSE Proximity Sensor library test\n");
#endif

    // SCANSE Proximity Sensor setup
    AP_Param::set_object_value(&sensor, sensor.var_info, "_TYPE", 
            AP_Proximity::Proximity_Type_SCANSE);
    AP_Param::set_object_value(&serial_manager, serial_manager.var_info, 
            "3_PROTOCOL", 11.0);
    AP_Param::set_object_value(&serial_manager, serial_manager.var_info, 
            "3_BAUD", 115.0);
    AP_Param::set_object_value(&serial_manager, serial_manager.var_info, 
            "PRX_TYPE", 1.0);
    AP_Param::set_object_value(&serial_manager, serial_manager.var_info, 
            "PRX_ORIENT", 0.0);


    // Initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    serial_manager.init();
    sensor.init();
#if HAL_OS_POSIX_IO
    ::printf("ProximitySensor: %d devices detected\n", sensor.num_sensors());
    if (sensor.num_sensors()) {
        ::printf("Min Dist: %f, Max Dist: %f\n", 
                sensor.distance_max(), sensor.distance_min());
    }
#endif
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(500);
    sensor.update();
    
    if (sensor.get_status() == AP_Proximity::Proximity_NotConnected) {
#if HAL_OS_POSIX_IO
        ::printf("Proximity sensor not connected.\n");
#endif
        return;
    }
    if (sensor.get_status() == AP_Proximity::Proximity_NoData) {
#if HAL_OS_POSIX_IO
        ::printf("Proximity sensor has no data.\n");
#endif
        return;
    }

    AP_Proximity::Proximity_Distance_Array prx_arr;
    if (sensor.get_horizontal_distances(prx_arr) == false) {
#if HAL_OS_POSIX_IO
        ::printf("Unable to retrieve proximity data.\n");
#endif
        return;
    }
#if HAL_OS_POSIX_IO
    ::printf("Received Distance Measurement:\n");
#endif
    for (uint8_t i=0; i<PROXIMITY_MAX_DIRECTION; i++) {
#if HAL_OS_POSIX_IO
        ::printf("orientation: Roll: %d Pitch: %d Yaw: %d, distance: %f\n",
                            get_orientation_roll(prx_arr.orientation[i]), 
                            get_orientation_pitch(prx_arr.orientation[i]), 
                            get_orientation_yaw(prx_arr.orientation[i]), 
                            prx_arr.distance[i]);
#endif
    }
#if HAL_OS_POSIX_IO
    ::printf("\n");
#endif
}
AP_HAL_MAIN();
