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
#pragma once

#include "v8stdint.h"
#include <vector>
#include "ydlidar_cmd.h"

typedef enum  {
  NoError,//无错误
  DeviceNotFoundError,//无串口设备
  PermissionError,
  OpenError,//打开串口错误
  ParityError,
  FramingError,
  BreakConditionError,
  WriteError,//
  ReadError,//读串口超时(雷达断电，雷达堵转，雷达接线不良)(警告级别）
  ResourceError,//串口占用
  UnsupportedOperationError,
  TimeoutError,//解包超时(警告级别）
  NotOpenError,//串口没打开
  HeaderError,//数据包头错误(警告级别）
  FirstSampleAngleError,//采样角错误(警告级别）
  LastSampleAngleError,//采样角错误(警告级别）
  PackageNumberError,//采样包数错误(警告级别）
  CheckSumError,//通讯校验和错误(警告级别）
  SensorError,//雷达硬件错误(致命级别）(S2不支持）
  EncodeError,//编码错误(警告级别)(S2不支持）
  PWRError,//供电异常(警告级别)(S2不支持）
  PDError,//PD异常(警告级别)(S2不支持）
  LDError,//LD异常(警告级别)(S2不支持）
  DataError,//数据异常(雷达被遮挡)(警告级别)
  TrembleError,
  LidarNotFoundError,//雷达离线
  UnknownError,//未初始化
} lidar_error_t;

#pragma pack(1)

struct node_info {
  uint8_t    sync_flag;  //sync flag
  uint16_t   sync_quality;//!信号质量
  uint16_t   angle_q6_checkbit; //!测距点角度
  uint16_t   distance_q2; //! 当前测距点距离
  uint64_t   stamp; //! 时间戳
  uint8_t    scan_frequence;//! 特定版本此值才有效,无效值是0
} __attribute__((packed)) ;


/// LiDAR Intensity Nodes Package
struct node_package_header_t {
  uint8_t   packageHeaderMSB;///< package header MSB
  uint8_t   packageHeaderLSB;///< package header LSB
  uint8_t   packageSync: 1; ///< package sync flag
  uint8_t   packageCTInfo: 7; ///< package ct info
  uint8_t   nowPackageNum;///< package number
  uint16_t  packageFirstSampleAngleSync: 1;///first sample angle sync flag
  uint16_t  packageFirstSampleAngle: 15; ///< first sample angle
  uint16_t  packageLastSampleAngleSync: 1; ///< last sample angle sync flag
  uint16_t  packageLastSampleAngle: 15; ///< last sample angle
  uint16_t  checkSum;///< checksum
} __attribute__((packed));

static_assert(sizeof(node_package_header_t) == 10,
              "response scan header size mismatch.");


namespace response_health_error {
enum bits : uint8_t {
  SensorError = 1 << 0,  // sensor error
  EncodeError = 1 << 1,  // encode error
  PWRError = 1 << 2,  // ref error
  PDError = 1 << 3,  // pd error
  LDError = 1 << 4,  // ld error
  DataError = 1 << 5,  // data error
  CSError = 1 << 6,  // cs error
};
}  // namespace response_health_error

namespace response_scan_packet_sync {
enum bits : uint8_t {
  sync = 1 << 0,                 // beginning of new full scan or sample angle flag
  // Reserved for future error bits
  reserved1 = 1 << 1,
  reserved2 = 1 << 2,
  reserved3 = 1 << 3,
  reserved4 = 1 << 4,
  reserved5 = 1 << 5,
  reserved6 = 1 << 6,
  reserved7 = 1 << 7,
};
}  // namespace response_scan_packet_sync


/// package node info
struct node_package_payload_t {
  uint16_t PackageSampleSi: 2; ///< si
  uint16_t PackageSampleDistance: 14; ///< range
} __attribute__((packed));
static_assert(sizeof(node_package_payload_t) == 2,
              "response scan payload size mismatch.");

struct scan_packet_t {
  node_package_header_t header;
  node_package_payload_t payload[40];
} __attribute__((packed));

static_assert(sizeof(scan_packet_t) == 90,
              "response scan packet size mismatch.");

struct node_package_intensity_payload_t {
  uint8_t PackageSampleIntensity;/// intensity
  node_package_payload_t PackageSample; ///< range
} __attribute__((packed));
static_assert(sizeof(node_package_intensity_payload_t) == 3,
              "response scan intensity payload size mismatch.");


struct scan_intensity_packet_t {
  node_package_header_t header;
  node_package_intensity_payload_t  payload[40];
} __attribute__((packed)) ;

static_assert(sizeof(scan_intensity_packet_t) == 130,
              "response scan intensity packet size mismatch.");


struct tsa_node_package_intensity_payload_t {
  uint16_t PackageSampleIntensity;/// intensity
  uint16_t PackageSampleDistance; ///< range
} __attribute__((packed));
static_assert(sizeof(tsa_node_package_intensity_payload_t) == 4,
              "response tsa scan intensity payload size mismatch.");


struct tsa_scan_intensity_packet_t {
  node_package_header_t header;
  tsa_node_package_intensity_payload_t  payload[40];
} __attribute__((packed)) ;

static_assert(sizeof(tsa_scan_intensity_packet_t) == 170,
              "response tsa scan intensity packet size mismatch.");


struct device_info {
  uint8_t   model; ///< 雷达型号
  uint16_t  firmware_version; ///< 固件版本号
  uint8_t   hardware_version; ///< 硬件版本号
  uint8_t   serialnum[16];    ///< 系列号
} __attribute__((packed)) ;
static_assert(sizeof(device_info) == 20,
              "device info size mismatch.");

struct device_health {
  uint8_t   status; ///< 健康状体
  uint16_t  error_code; ///< 错误代码
} __attribute__((packed))  ;

struct sampling_rate_t {
  uint8_t rate;	///< 采样频率
} __attribute__((packed))  ;

struct scan_frequency_t {
  uint32_t frequency;	///< 扫描频率
} __attribute__((packed))  ;

/// LiDAR Zero Offset Angle
struct offset_angle_t {
  int32_t angle;
} __attribute__((packed))  ;

struct cmd_packet_t {
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} __attribute__((packed)) ;

/// LiDAR response Header
struct lidar_ans_header_t {
  uint8_t  syncByte1;
  uint8_t  syncByte2;
  uint32_t size: 30;
  uint32_t subType: 2;
  uint8_t  type;
} __attribute__((packed));

static_assert(sizeof(lidar_ans_header_t) == 7,
              "Ans header size mismatch.");

struct ct_packet_t {
  uint8_t size;
  uint8_t index;
  uint8_t info[100];
  uint8_t crc;
  uint8_t cs;
  uint8_t valid;
} __attribute__((packed)) ;

static_assert(sizeof(ct_packet_t) == 105,
              "ct packet size mismatch.");

#pragma pack()


struct LaserPoint {
  float angle;///< 角度(弧度）
  float range;///< 距离(米）
  float compensation_angle;///< 补偿角(弧度)，已经加在角度上(TSA无)
  uint8_t interference_sign;///< 抗干扰标志(TSA无)
  uint16_t intensity;///< 信号质量(）
  LaserPoint &operator = (const LaserPoint &data) {
    this->angle = data.angle;
    this->range = data.range;
    this->compensation_angle = data.compensation_angle;
    this->interference_sign = data.interference_sign;
    this->intensity = data.intensity;
    return *this;
  }
};

struct LaserFan {
  uint8_t    sync_flag;  //sync flag
  /// Array of lidar points
  ct_packet_t info;
  std::vector<LaserPoint> points;
  LaserFan &operator = (const LaserFan &data) {
    this->sync_flag = data.sync_flag;
    this->info = data.info;
    this->points = data.points;
    return *this;
  }
};


//! A struct for returning configuration from the YDLIDAR
struct LaserConfig {
  //! Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
  float min_angle;
  //! Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
  float max_angle;
  //! Scan resoltuion [s]
  float time_increment;
  //! Time between scans
  float scan_time;
  //! Minimum range [m]
  float min_range;
  //! Maximum range [m]
  float max_range;
  //! fixed resolution size
  int fixed_size;
  LaserConfig &operator = (const LaserConfig &data) {
    min_angle = data.min_angle;
    max_angle = data.max_angle;
    time_increment = data.time_increment;
    scan_time = data.scan_time;
    min_range = data.min_range;
    max_range = data.max_range;
    fixed_size = data.fixed_size;
    return *this;
  }
};


struct LaserScan {
  //! Array of laser point
  std::vector<LaserPoint> data;
  //! System time when first range was measured in nanoseconds
  uint64_t system_time_stamp;
  //! lidar out frequency
  float lidar_scan_frequency;///< 雷达输出的实时扫描频率
  //! Configuration of scan
  LaserConfig config;
  LaserScan &operator = (const LaserScan &data) {
    this->data = data.data;
    this->system_time_stamp = data.system_time_stamp;
    this->lidar_scan_frequency = data.lidar_scan_frequency;
    this->config = data.config;
    return *this;
  }
};
