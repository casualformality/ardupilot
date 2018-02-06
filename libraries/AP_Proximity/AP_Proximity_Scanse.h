#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_SCANSE_TIMEOUT_MS            200                               // requests timeout after 0.2 seconds
#define PROXIMITY_SCANSE_RST_ITVL_MS           500                               // requests timeout after 0.2 seconds

class AP_Proximity_Scanse : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_Scanse(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update(void);

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const;
    float distance_min() const;

private:

    enum RequestType {
        RequestType_None = 0,
        RequestType_MotorSpeed,
        RequestType_SampleRate,
        RequestType_MotorReady,
        RequestType_Version,
        RequestType_DevInfo,
        RequestType_Reset,
        RequestType_StartAcq,
        RequestType_StopAcq,
        RequestType_DataPacket
    };

    // initialise sensor (returns true if sensor is successfully initialised)
    bool initialise();
    void init_sectors();
    void set_motor_speed(bool on_off);
    void get_motor_status();
    void set_sample_rate();
    void start_acquisition();
    void stop_acquisition();
    void reset_device();
    bool check_for_reply();
    bool process_reply();
    void clear_buffer();

    // Device Info Structure
    typedef union {
        typedef struct {
            uint8_t  header[2];
            uint8_t  bit_rate[6];
            uint8_t  laser_state;
            uint8_t  mode;
            uint8_t  diagnostic;
            uint8_t  motor_speed[2];
            uint8_t  sample_rate[4];
            uint8_t  lf;
        } _info_struct;
        uint8_t bytes[18];
    } Device_Info;

    // Version Info Structure
    typedef union {
        typedef struct {
            uint8_t header[2];
            uint8_t model[5];
            uint8_t protocol[2];
            uint8_t fw_ver[2];
            uint8_t hw_ver;
            uint8_t serial[8];
            uint8_t lf;
        } _version_struct;
        uint8_t bytes[21];
    } Device_Version;

    // Measurement Data Structure
    typedef struct {
        // Currently, only e0 (bit 1) indicates an error
        // Sync (bit 0) indicates first measurement on new rotation
        uint8_t status;
        uint8_t azimuth[2];
        uint8_t distance[2];
        uint8_t strength;
        uint8_t checksum;
    } Data_Block;

    // Response Structure (no params)
    typedef struct {
        uint8_t header[2];
        uint8_t status[2];
        uint8_t sum;
        uint8_t lf;
    } Status_Block;

    // Response Structure (with params)
    typedef struct {
        uint8_t header[2];
        uint8_t rsp[2];
        uint8_t lf;
        uint8_t status[2];
        uint8_t sum;
        uint8_t lf2;
    } Rsp_Block;

    // config variables
    uint8_t _motor_speed = 0;           // motor speed as reported by lidar
    uint8_t _sample_rate = 0;           // sample rate as reported by lidar

    bool _sector_initialised = false;
    // middle angle of each sector
    uint16_t _sector_middle_deg[PROXIMITY_SECTORS_MAX] = 
            {   0,  45,  90, 135, 180, 225, 270, 315,   0,   0,   0,   0};
    // width (in degrees) of each sector
    uint8_t _sector_width_deg[PROXIMITY_SECTORS_MAX] = 
            {  45,  45,  45,  45,  45,  45,  45,  45,   0,   0,   0,   0};

    // communication variables
    AP_HAL::UARTDriver *uart = nullptr;
    uint8_t _buffer[21];
    uint8_t _buf_idx;

    // Request related variables
    RequestType _last_request_type;
    uint32_t _last_request_ms;
    uint32_t _last_distance_received_ms;
    
    // Convert 16-bit fixed point to floating point
    inline float _fixed_to_float(uint8_t *bytes) {
        return _bytes_to_float(bytes) / 16.0f;
    }

    // 
    inline float _bytes_to_float(uint8_t *bytes) {
        uint16_t val;
        val = ((uint16_t) bytes[1]) << 8;
        val = val + bytes[0];
        return (float) val;
    }

    // Parse 2 bytes as an integer
    inline uint16_t _bytes_to_uint16(uint8_t *bytes) {
        uint16_t status;
        status = (bytes[0] - 0x30) * 10;
        status = status + (bytes[1] - 0x30);
        return status;
    }
};
