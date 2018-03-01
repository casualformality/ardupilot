/*
 * ProximitySensor test code
 */

#include <AP_HAL/AP_HAL.h>

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Proximity/AP_Proximity.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Serial manager is needed for UART comunications
static AP_SerialManager serial_manager;
static AP_Proximity sensor {serial_manager};

const char *get_orientation_string(uint8_t dir) {
    switch(dir) {
        case MAV_SENSOR_ROTATION_NONE:
            return "Roll: 0 Pitch: 0 Yaw:   0";
        case MAV_SENSOR_ROTATION_YAW_45:
            return "Roll: 0 Pitch: 0 Yaw:  45";
        case MAV_SENSOR_ROTATION_YAW_90:
            return "Roll: 0 Pitch: 0 Yaw:  90";
        case MAV_SENSOR_ROTATION_YAW_135:
            return "Roll: 0 Pitch: 0 Yaw: 135";
        case MAV_SENSOR_ROTATION_YAW_180:
            return "Roll: 0 Pitch: 0 Yaw: 180";
        case MAV_SENSOR_ROTATION_YAW_225:
            return "Roll: 0 Pitch: 0 Yaw: 225";
        case MAV_SENSOR_ROTATION_YAW_270:
            return "Roll: 0 Pitch: 0 Yaw: 270";
        case MAV_SENSOR_ROTATION_YAW_315:
            return "Roll: 0 Pitch: 0 Yaw: 315";
        default:
            return "Roll: ? Pitch: ? Yaw: ?";
    }
}

void setup()
{
    // print welcome message
    hal.console->printf("SCANSE Proximity Sensor library test\n");

    // SCANSE Proximity Sensor setup
    AP_Param::set_object_value(&sensor, sensor.var_info, "_TYPE", 
            AP_Proximity::Proximity_Type_SCANSE);
    AP_Param::set_object_value(&serial_manager, serial_manager.var_info, 
            "2_PROTOCOL", 11.0);
    AP_Param::set_object_value(&serial_manager, serial_manager.var_info, 
            "2_BAUD", 115.0);


    // Initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    serial_manager.init();
    sensor.init();
    
    hal.console->printf("ProximitySensor: %d devices detected\n", sensor.num_sensors());
    if (sensor.num_sensors()) {
        hal.console->printf("Min Dist: %f, Max Dist: %f\n", 
                sensor.distance_max(), sensor.distance_min());
    }
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(500);
    sensor.update();
    
    if (sensor.get_status() == AP_Proximity::Proximity_NotConnected) {
        hal.console->printf("Proximity sensor not connected.\n");
        return;
    }
    if (sensor.get_status() == AP_Proximity::Proximity_NoData) {
        hal.console->printf("Proximity sensor has no data.\n");
        return;
    }

    AP_Proximity::Proximity_Distance_Array prx_arr;
    if (sensor.get_horizontal_distances(prx_arr) == false) {
        hal.console->printf("Unable to retrieve proximity data.\n");
        return;
    }
    hal.console->printf("Received Distance Measurement:\n");
    for (uint8_t i=0; i<PROXIMITY_MAX_DIRECTION; i++) {
        hal.console->printf("%s, distance: %f\n",
                            get_orientation_string(prx_arr.orientation[i]),
                            prx_arr.distance[i]);
    }
    hal.console->printf("\n");
}
AP_HAL_MAIN();
