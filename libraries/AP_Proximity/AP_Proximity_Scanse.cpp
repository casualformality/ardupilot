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

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_Scanse.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

#define DBG_LOG

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_Scanse::AP_Proximity_Scanse(AP_Proximity &_frontend,
                                         AP_Proximity::Proximity_State &_state,
                                         AP_SerialManager &serial_manager) :
    AP_Proximity_Backend(_frontend, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0));
        ::printf("Flow control: %d\n", uart->get_flow_control());
        uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        ::printf("Flow control: %d\n", uart->get_flow_control());
        ::printf("Baud rate: %d\n", serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0));
    }
    _last_request_type = RequestType_None;
    _last_distance_received_ms = 0;
}

// detect if a Scanse proximity sensor is connected by looking for a configured serial port
bool AP_Proximity_Scanse::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(
            AP_SerialManager::SerialProtocol_Lidar360, 0) != nullptr;
}

// update the state of the sensor
void AP_Proximity_Scanse::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // initialise sensor if necessary
    initialise();

    // process incoming messages
    check_for_reply();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || 
            (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_SCANSE_TIMEOUT_MS)) {
        set_status(AP_Proximity::Proximity_NoData);
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_Scanse::distance_max() const
{
    return 40.0f;
}
float AP_Proximity_Scanse::distance_min() const
{
    return 0.50f;
}

// initialise sensor (returns true if sensor is succesfully initialised)
bool AP_Proximity_Scanse::initialise()
{
    unsigned long curTime = AP_HAL::millis();

    if(_last_request_type == RequestType_None) {
        if ((_last_request_ms == 0) || curTime - _last_request_ms > 1000) {
            reset_device();
        }
        return false;
    } else if(_last_request_type == RequestType_Reset) {
        // Check every 500ms for completion of reset
        if (curTime - _last_request_ms > 500) {
            get_motor_status();
            _last_request_type = RequestType_Reset;
        }
    }
    // initialise sectors
    if (!_sector_initialised) {
        init_sectors();
        return false;
    }
    return true;
}

// initialise sector angles using user defined ignore areas
void AP_Proximity_Scanse::init_sectors()
{
    // use defaults if no ignore areas defined
    uint8_t ignore_area_count = get_ignore_area_count();
    if (ignore_area_count == 0) {
        _sector_initialised = true;
        return;
    }

    uint8_t sector = 0;

    for (uint8_t i=0; i<ignore_area_count; i++) {

        // get ignore area info
        uint16_t ign_area_angle;
        uint8_t ign_area_width;
        if (get_ignore_area(i, ign_area_angle, ign_area_width)) {

            // calculate how many degrees of space we have between 
            // this end of this ignore area and the start of the end
            int16_t start_angle, end_angle;
            get_next_ignore_start_or_end(1, ign_area_angle, start_angle);
            get_next_ignore_start_or_end(0, start_angle, end_angle);
            int16_t degrees_to_fill = wrap_360(end_angle - start_angle);

            // divide up the area into sectors
            while ((degrees_to_fill > 0) && (sector < PROXIMITY_SECTORS_MAX)) {
                uint16_t sector_size;
                if (degrees_to_fill >= 90) {
                    // set sector to maximum of 45 degrees
                    sector_size = 45;
                } else if (degrees_to_fill > 45) {
                    // use half the remaining area to optimise size of this sector and the next
                    sector_size = degrees_to_fill / 2.0f;
                } else  {
                    // 45 degrees or less are left so put it all into the next sector
                    sector_size = degrees_to_fill;
                }
                // record the sector middle and width
                _sector_middle_deg[sector] = wrap_360(start_angle + sector_size / 2.0f);
                _sector_width_deg[sector] = sector_size;

                // move onto next sector
                start_angle += sector_size;
                sector++;
                degrees_to_fill -= sector_size;
            }
        }
    }

    // set num sectors
    _num_sectors = sector;

    // re-initialise boundary because sector locations have changed
    init_boundary();

    // record success
    _sector_initialised = true;
}

// set speed of rotating motor
void AP_Proximity_Scanse::set_motor_speed(bool on_off)
{
#ifdef DBG_LOG
    ::printf("Setting motor speed\n");
#endif
    // exit immediately if no uart
    if (uart == nullptr) {
        return;
    }

    // set motor update speed
    if (on_off) {
        uart->write("MS05\n");  // send request to spin motor at 5hz
        _motor_speed = 5;
    } else {
        uart->write("MS00\n");  // send request to stop motor
        _motor_speed = 0;
    }

    _last_request_type = RequestType_MotorSpeed;
    _last_request_ms = AP_HAL::millis();
}

void AP_Proximity_Scanse::get_motor_status()
{
    // exit immediately if no uart
    if (uart == nullptr) {
        return;
    }

    uart->write("MZ\n");        // send request for motor status

    _last_request_type = RequestType_MotorReady;
    _last_request_ms = AP_HAL::millis();
}

void AP_Proximity_Scanse::set_sample_rate()
{
#ifdef DBG_LOG
    ::printf("Setting sample rate\n");
#endif
    // exit immediately if no uart
    if (uart == nullptr) {
        return;
    }

    uart->write("LR01\n");      // send request to sample at 500-600Hz
    _sample_rate = 1;

    _last_request_type = RequestType_SampleRate;
    _last_request_ms = AP_HAL::millis();
}

void AP_Proximity_Scanse::start_acquisition()
{
#ifdef DBG_LOG
    ::printf("Starting acquisition\n");
#endif
    // exit immediately if no uart
    if (uart == nullptr) {
        return;
    }

    uart->write("DS\n");      // send request to sample at 500-600Hz

    _last_request_type = RequestType_StartAcq;
    _last_request_ms = AP_HAL::millis();
}

void AP_Proximity_Scanse::stop_acquisition()
{
#ifdef DBG_LOG
    ::printf("Stopping acquisition\n");
#endif
    // exit immediately if no uart
    if (uart == nullptr) {
        return;
    }

    uart->write("DX\n");      // send request to sample at 500-600Hz

    _last_request_type = RequestType_StopAcq;
    _last_request_ms = AP_HAL::millis();
}

void AP_Proximity_Scanse::reset_device()
{
    // exit immediately if no uart
    if (uart == nullptr) {
        return;
    }

    uart->write("RR\n");      // send request to reset the device
#ifdef DBG_LOG
    ::printf("Resetting device\n");
#endif

    _motor_speed = 0;
    _last_request_type = RequestType_Reset;
    _last_request_ms = AP_HAL::millis();
    clear_buffer();
    while (uart->available()) {
        uart->read();
    }
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_Scanse::check_for_reply()
{
    if (uart == nullptr) {
        return false;
    }

    // Read any available lines from the lidar
    //    If CR (i.e. \r), LF (\n) it means we have received a full packet.
    //    MotorSpeed and SampleRate request return 9 bytes, though.
    //    During acquisition, there are no LFs, so parse on every 7th byte.
    uint16_t packet_count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c = uart->read();
        if (_last_request_type == RequestType_DataPacket) {
            _buffer[_buf_idx++] = c;
            if (_buf_idx >= 7) {
                if (process_reply()) {
                    packet_count++;
                }
                clear_buffer();
            }
        } else if (_last_request_type == RequestType_MotorSpeed ||
                   _last_request_type == RequestType_SampleRate) {
            _buffer[_buf_idx++] = c;
            if (_buf_idx >= 9) {
                if (process_reply()) {
                    packet_count++;
                }
                clear_buffer();
            }
        } else {
            // check for end of packet
            if (c == '\r' || c == '\n') {
                if (_buf_idx > 0) {
                    _buffer[_buf_idx] = c;
                    if (process_reply()) {
                        packet_count++;
                    }
                }
                // clear buffers after processing
                clear_buffer();
            } else {
                _buffer[_buf_idx++] = c;
                if (_buf_idx >= sizeof(_buffer)) {
                    // too long, discard the line
                    clear_buffer();
                }
            }
        }
    }

    return (packet_count > 0);
}

// process reply
bool AP_Proximity_Scanse::process_reply()
{
    if (uart == nullptr) {
        return false;
    }

    bool success = false;

    switch (_last_request_type) {
        case RequestType_None:
            break;

        case RequestType_MotorSpeed:
        {
            Rsp_Block *rsp = (Rsp_Block *) _buffer;
            uint16_t status = _bytes_to_uint16(rsp->status);
            uint16_t speed = _bytes_to_uint16(rsp->rsp);
            if (status == 0) {
                // Wait until motor speed has stabilized
                success = true;
                get_motor_status();
            } else if (status == 11) {
                // Invalid parameter, try again
                set_motor_speed(true);
            } else if (status == 12) {
                // Previous motor speed command has not completed
                get_motor_status();
            } else {
                // Unknown error
                reset_device();
            }
            break;
        }

        case RequestType_MotorReady:
        {
            Status_Block *rsp = (Status_Block *) _buffer;
            uint16_t status = _bytes_to_uint16(rsp->status);
            if (status == 0) {
                // Move to next initialization stage
                success = true;
                set_sample_rate();
            } else if (status == 1) {
                // Motor not ready yet
                get_motor_status();
            } else {
                // Unknown error
                reset_device();
            }
            break;
        }
        
        case RequestType_SampleRate:
        {
            Rsp_Block *rsp = (Rsp_Block *) _buffer;
            uint16_t status = _bytes_to_uint16(rsp->status);
            uint16_t speed = _bytes_to_uint16(rsp->rsp);
            if (status == 0) {
                // Move to next initialization stage
                success = true;
                start_acquisition();
            } else if (status == 11) {
                // Invalid parameter, try again
                set_sample_rate();
            } else {
                // Unknown error
                reset_device();
            }
            break;
        }
        
        case RequestType_Version:
        {
            Device_Version *rsp = (Device_Version *) _buffer;
            success = true;
            // TODO: Print device version here
            break;
        }
        
        case RequestType_DevInfo:
        {
            Device_Info *rsp = (Device_Info *) _buffer;
            success = true;
            // TODO: Print device info here
            break;
        }
        
        case RequestType_Reset:
        {
            success = true;
            clear_buffer();
            set_motor_speed(true);
            break;
        }

        case RequestType_StartAcq:
        {
            Status_Block *rsp = (Status_Block *) _buffer;
            uint16_t status = _bytes_to_uint16(rsp->status);
            if (status == 0) {
                // Ready to receive data packets
                success = true;
                _last_request_type = RequestType_DataPacket;
            } else if (status == 12) {
                // Motor speed not stabilized
                get_motor_status();
            } else if (status == 13) {
                // Motor is stationary
                set_motor_speed(true);
            } else {
                // Unknown error
                reset_device();
            }
            break;
        }

        case RequestType_StopAcq:
        {
            Status_Block *rsp = (Status_Block *) _buffer;
            uint16_t status = _bytes_to_uint16(rsp->status);
            if (status == 0 || status == 99) {
                success = true;
                _last_request_type = RequestType_None;
            } else {
                reset_device();
            }
            break;
        }

        case RequestType_DataPacket:
        {
            Data_Block *packet = (Data_Block *) _buffer;
            float angle_deg = _fixed_to_float(packet->azimuth);
            float distance_m = _bytes_to_float(packet->distance);

            uint8_t sector;
            if (convert_angle_to_sector(angle_deg, sector)) {
                _angle[sector] = angle_deg;
                _distance[sector] = distance_m / 100.0f;
                _distance_valid[sector] = is_positive(distance_m);
                _last_distance_received_ms = AP_HAL::millis();
                success = true;
                // update boundary used for avoidance
                update_boundary_for_sector(sector);
                _last_request_type = RequestType_DataPacket;
            }
            break;
        }

        default:
            break;
    }

    return success;
}

// clear buffers ahead of processing next message
void AP_Proximity_Scanse::clear_buffer()
{
    memset(_buffer, 0, sizeof(_buffer));
    _buf_idx = 0;
}
