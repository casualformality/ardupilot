/*
 * ProximitySensor test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Proximity/AP_Proximity_Backend.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_SerialManager serial_manager;
static ProximitySensor sensor {serial_manager, serial_manager};

char *get_orientation_str(uint8_t dir) {
    switch(dir) {
        case MAV_SENSOR_ROTATION_NONE:
            return "Roll: 0, Pitch: 0, Yaw: 0";
        case MAV_SENSOR_ROTATION_YAW_45:
            return "Roll: 0, Pitch: 0, Yaw: 45";
        case MAV_SENSOR_ROTATION_YAW_90:
            return "Roll: 0, Pitch: 0, Yaw: 90";
        case MAV_SENSOR_ROTATION_YAW_135:
            return "Roll: 0, Pitch: 0, Yaw: 135";
        case MAV_SENSOR_ROTATION_YAW_180:
            return "Roll: 0, Pitch: 0, Yaw: 180";
        case MAV_SENSOR_ROTATION_YAW_225:
            return "Roll: 0, Pitch: 0, Yaw: 225";
        case MAV_SENSOR_ROTATION_YAW_270:
            return "Roll: 0, Pitch: 0, Yaw: 270";
        case MAV_SENSOR_ROTATION_YAW_315:
            return "Roll: 0, Pitch: 0, Yaw: 315";
        case MAV_SENSOR_ROTATION_ROLL_180:
            return "Roll: 180, Pitch: 0, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_180_YAW_45:
            return "Roll: 180, Pitch: 0, Yaw: 45";
        case MAV_SENSOR_ROTATION_ROLL_180_YAW_90:
            return "Roll: 180, Pitch: 0, Yaw: 90";
        case MAV_SENSOR_ROTATION_ROLL_180_YAW_135:
            return "Roll: 180, Pitch: 0, Yaw: 135";
        case MAV_SENSOR_ROTATION_PITCH_180:,
            return "Roll: 0, Pitch: 180, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_180_YAW_225:
            return "Roll: 180, Pitch: 0, Yaw: 225";
        case MAV_SENSOR_ROTATION_ROLL_180_YAW_270:
            return "Roll: 180, Pitch: 0, Yaw: 270";
        case MAV_SENSOR_ROTATION_ROLL_180_YAW_315:
            return "Roll: 180, Pitch: 0, Yaw: 315";
        case MAV_SENSOR_ROTATION_ROLL_90:
            return "Roll: 90, Pitch: 0, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_90_YAW_45:
            return "Roll: 90, Pitch: 0, Yaw: 45";
        case MAV_SENSOR_ROTATION_ROLL_90_YAW_90:
            return "Roll: 90, Pitch: 0, Yaw: 90";
        case MAV_SENSOR_ROTATION_ROLL_90_YAW_135:
            return "Roll: 90, Pitch: 0, Yaw: 135";
        case MAV_SENSOR_ROTATION_ROLL_270:
            return "Roll: 270, Pitch: 0, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_270_YAW_45:
            return "Roll: 270, Pitch: 0, Yaw: 45";
        case MAV_SENSOR_ROTATION_ROLL_270_YAW_90:
            return "Roll: 270, Pitch: 0, Yaw: 90";
        case MAV_SENSOR_ROTATION_ROLL_270_YAW_135:
            return "Roll: 270, Pitch: 0, Yaw: 135";
        case MAV_SENSOR_ROTATION_PITCH_90:
            return "Roll: 0, Pitch: 90, Yaw: 0";
        case MAV_SENSOR_ROTATION_PITCH_270:
            return "Roll: 0, Pitch: 270, Yaw: 0";
        case MAV_SENSOR_ROTATION_PITCH_180_YAW_90:
            return "Roll: 0, Pitch: 180, Yaw: 90";
        case MAV_SENSOR_ROTATION_PITCH_180_YAW_270:
            return "Roll: 0, Pitch: 180, Yaw: 270";
        case MAV_SENSOR_ROTATION_ROLL_90_PITCH_90:
            return "Roll: 90, Pitch: 90, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_180_PITCH_90:
            return "Roll: 180, Pitch: 90, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_270_PITCH_90:
            return "Roll: 270, Pitch: 90, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_90_PITCH_180:
            return "Roll: 90, Pitch: 180, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_270_PITCH_180:
            return "Roll: 270, Pitch: 180, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_90_PITCH_270:
            return "Roll: 90, Pitch: 270, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_180_PITCH_270:
            return "Roll: 180, Pitch: 270, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_270_PITCH_270:
            return "Roll: 270, Pitch: 270, Yaw: 0";
        case MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90:
            return "Roll: 90, Pitch: 180, Yaw: 90";
        case MAV_SENSOR_ROTATION_ROLL_90_YAW_270:
            return "Roll: 90, Pitch: 0, Yaw: 270";
        case MAV_SENSOR_ROTATION_MAX:
            return "Roll: 315, Pitch: 315, Yaw: 315";
        default:
            return "Unknown";
    }
}

void setup()
{
    // print welcome message
    hal.console->printf("Proximity Sensor library test\n");

    // IR Proximity Sensor setup
    AP_PARAM::set_object_value(&sensor, sensor.var_info, "_TYPE", AP_Proximity::Proximity_Type_IR);

    // Initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    sensor.init();
    hal.console->printf("ProximitySensor: %d devices detected\n", sensor.num_sensors());
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(100);
    sonar.update();

    AP_Proximity_Backend *prox = sensor.get_rangefinder();
    if (prox == nullptr) {
        hal.console->printf("No proximity sensor found.\n");
        return;
    }
    if (prox->get_status() == AP_Proximity::Proximity_NotConnected) {
        hal.console->printf("Proximity sensor not connected.\n");
        return;
    }
    if (prox->get_status() == AP_Proximity::Proximity_NoData) {
        hal.console->printf("Proximity sensor has no data.\n");
        return;
    }

    AP_Proximity::Proximity_Distance_Array prx_arr;
    if (prox.get_horizontal_distances(&prx_arr) == false) {
        hal.console->printf("Unable to retrieve proximity data.\n");
        return
    }
    hal.console->printf("Received Distance Measurement:\n");
    for (uint8_t i=0; i<PROXIMITY_MAX_DIRECTION; i++) {
        hal.console->printf("orientation: %s, distance: %f\n",
                            get_orientation_str(prx_arr.orientation[i]), 
                            prx_arr.distance[i]);
    }
    hal.console->printf("\n");
}
AP_HAL_MAIN();
