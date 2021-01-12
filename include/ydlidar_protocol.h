//
// The MIT License (MIT)
//
// Copyright (c) 2020 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef YDLDAR_PROTOCOL_H_
#define YDLDAR_PROTOCOL_H_

#include <stdint.h>
#include "ydlidar_def.h"
#include "ydlidar_cmd.h"
#include "serial.h"

namespace ydlidar {
namespace protocol {

using namespace serial;

const char *DescribeError(const lidar_error_t &error);

void reset_ct_packet_t(ct_packet_t &ct);

lidar_error_t convert_ct_packet_to_error(const ct_packet_t &ct) ;

result_t check_ct_packet_t(const  ct_packet_t &ct);

void write_command(Serial *serial, uint8_t cmd);

result_t wait_for_data(Serial *serial, size_t data_count,
                       uint32_t timeout = READ_DEFAULT_TIMEOUT);

result_t read_command(Serial *serial, uint8_t *buffer, size_t size,
                      lidar_error_t &error,
                      uint32_t timeout = READ_DEFAULT_TIMEOUT);

result_t read_response_header_t(Serial *serial,
                                lidar_ans_header_t &header, lidar_error_t &error,
                                uint32_t timeout = READ_DEFAULT_TIMEOUT);

result_t check_ans_header_t(const lidar_ans_header_t &header,
                            lidar_error_t &error);

result_t read_response_health_t(Serial *serial, device_health &health,
                                lidar_error_t &error, uint32_t timeout = READ_DEFAULT_TIMEOUT);

result_t read_response_device_info_t(Serial *serial,
                                     device_info &info,
                                     lidar_error_t &error, uint32_t timeout = READ_DEFAULT_TIMEOUT);

result_t read_response_sample_rate_t(Serial *serial,
                                     sampling_rate_t &rate,
                                     lidar_error_t &error, uint32_t timeout = READ_DEFAULT_TIMEOUT);

result_t read_response_scan_frequency_t(Serial *serial,
                                        scan_frequency_t &frequency,
                                        lidar_error_t &error, uint32_t timeout = READ_DEFAULT_TIMEOUT);

result_t read_response_offset_angle_t(Serial *serial,
                                      offset_angle_t &angle,
                                      lidar_error_t &error, uint32_t timeout = READ_DEFAULT_TIMEOUT);

result_t parse_payload(const scan_packet_t &scan, LaserFan &data);

result_t parse_intensity_payload(const scan_intensity_packet_t &scan,
                                 LaserFan &data);

result_t check_package_header_t(const node_package_header_t &header,
                                lidar_error_t &error);

result_t parse_ct_packet_t(const node_package_header_t &header,
                           unsigned short error_count, ct_packet_t &ct);

uint8_t crc8_t(uint8_t *ptr, uint16_t len, uint8_t default_crc = 0x00,
               uint8_t poly = 0x8c, uint8_t inverted = 1);

uint16_t checksum_response_scan_packet_t(const scan_packet_t &scan);

uint16_t checksum_response_scan_intensity_packet_t(const scan_intensity_packet_t
    &scan);

result_t read_response_scan_header_t(Serial *serial,
                                     node_package_header_t &header, ct_packet_t &ct,
                                     lidar_error_t &error, uint32_t timeout = READ_DEFAULT_TIMEOUT);

bool is_valid_data(uint8_t *data, size_t size);

result_t check_scan_protocol(Serial *serial, int8_t &protocol,
                             uint32_t timeout = READ_DEFAULT_TIMEOUT);

result_t read_response_scan_t(Serial *serial, scan_packet_t &scan,
                              ct_packet_t &ct,
                              lidar_error_t &error, uint32_t timeout = READ_DEFAULT_TIMEOUT);

result_t read_response_scan_intensity_t(Serial *serial,
                                        scan_intensity_packet_t &scan, ct_packet_t &ct,
                                        lidar_error_t &error, uint32_t timeout = READ_DEFAULT_TIMEOUT);

}//namespace protocol
}  // namespace ydlidar
#endif  // YDLDAR_PROTOCOL_H_
