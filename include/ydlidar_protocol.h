
#pragma once

#include <v8stdint.h>
#include <vector>
#include <ydlidar_cmd.h>

#define SUNNOISEINTENSITY 0xff
#define GLASSNOISEINTENSITY 0xfe

#if defined(_WIN32)
#pragma pack(1)
#endif

struct node_info {
  uint8_t    sync_flag;  //sync flag
  uint16_t   sync_quality;//!信号质量
  uint16_t   angle_q6_checkbit; //!测距点角度
  uint16_t   distance_q2; //! 当前测距点距离
  uint64_t   stamp; //! 时间戳
  uint8_t    scan_frequence;//! 特定版本此值才有效,无效值是0
} __attribute__((packed)) ;

struct PackageNode {
  uint8_t PakageSampleQuality;
  uint16_t PakageSampleDistance;
} __attribute__((packed));

struct node_package {
  uint16_t  package_Head;
  uint8_t   package_CT;
  uint8_t   nowPackageNum;
  uint16_t  packageFirstSampleAngle;
  uint16_t  packageLastSampleAngle;
  uint16_t  checkSum;
  PackageNode  packageSample[PackageSampleMaxLngth];
} __attribute__((packed)) ;

struct node_packages {
  uint16_t  package_Head;
  uint8_t   package_CT;
  uint8_t   nowPackageNum;
  uint16_t  packageFirstSampleAngle;
  uint16_t  packageLastSampleAngle;
  uint16_t  checkSum;
  uint16_t  packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed)) ;


struct device_info {
  uint8_t   model; ///< 雷达型号
  uint16_t  firmware_version; ///< 固件版本号
  uint8_t   hardware_version; ///< 硬件版本号
  uint8_t   serialnum[16];    ///< 系列号
} __attribute__((packed)) ;

struct device_health {
  uint8_t   status; ///< 健康状体
  uint16_t  error_code; ///< 错误代码
} __attribute__((packed))  ;

struct sampling_rate {
  uint8_t rate;	///< 采样频率
} __attribute__((packed))  ;

struct scan_frequency {
  uint32_t frequency;	///< 扫描频率
} __attribute__((packed))  ;

struct scan_rotation {
  uint8_t rotation;
} __attribute__((packed))  ;

struct scan_exposure {
  uint8_t exposure;	///< 低光功率模式
} __attribute__((packed))  ;

struct scan_heart_beat {
  uint8_t enable;	///< 掉电保护状态
} __attribute__((packed));

struct scan_points {
  uint8_t flag;
} __attribute__((packed))  ;

struct function_state {
  uint8_t state;
} __attribute__((packed))  ;

struct cmd_packet {
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} __attribute__((packed)) ;

struct lidar_ans_header {
  uint8_t  syncByte1;
  uint8_t  syncByte2;
  uint32_t size: 30;
  uint32_t subType: 2;
  uint8_t  type;
} __attribute__((packed));

#if defined(_WIN32)
#pragma pack()
#endif


struct LaserPoint {
  float angle;
  float range;
  float intensity;
  LaserPoint &operator = (const LaserPoint &data) {
    angle = data.angle;
    range = data.range;
    intensity = data.intensity;
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
  LaserConfig &operator = (const LaserConfig &data) {
    min_angle = data.min_angle;
    max_angle = data.max_angle;
    time_increment = data.time_increment;
    scan_time = data.scan_time;
    min_range = data.min_range;
    max_range = data.max_range;
    return *this;
  }
};


struct LaserScan {
  //! Array of laser point
  std::vector<LaserPoint> data;
  //! System time when first range was measured in nanoseconds
  uint64_t system_time_stamp;
  //! Configuration of scan
  LaserConfig config;
  LaserScan &operator = (const LaserScan &data) {
    this->data = data.data;
    system_time_stamp = data.system_time_stamp;
    config = data.config;
    return *this;
  }
};
