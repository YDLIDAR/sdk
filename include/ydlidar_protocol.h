#pragma once
#include "v8stdint.h"
#include <vector>

#define PropertyBuilderByName(type, name, access_permission)\
    access_permission:\
        type m_##name;\
    public:\
    inline void set##name(type v) {\
        m_##name = v;\
    }\
    inline type get##name() {\
        return m_##name;\
}\


#if !defined(_countof)
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef M_PI
#define M_PI 3.1415926
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)
#define SUNNOISEINTENSITY 0xff
#define GLASSNOISEINTENSITY 0xfe

#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x92
#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x80
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81
#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT 8

#define LIDAR_CMD_RUN_POSITIVE             0x06
#define LIDAR_CMD_RUN_INVERSION            0x07
#define LIDAR_CMD_SET_AIMSPEED_ADDMIC      0x09
#define LIDAR_CMD_SET_AIMSPEED_DISMIC      0x0A
#define LIDAR_CMD_SET_AIMSPEED_ADD         0x0B
#define LIDAR_CMD_SET_AIMSPEED_DIS         0x0C
#define LIDAR_CMD_GET_AIMSPEED             0x0D

#define LIDAR_CMD_SET_SAMPLING_RATE        0xD0
#define LIDAR_CMD_GET_SAMPLING_RATE        0xD1
#define LIDAR_STATUS_OK                    0x0
#define LIDAR_STATUS_WARNING               0x1
#define LIDAR_STATUS_ERROR                 0x2

#define LIDAR_CMD_ENABLE_LOW_POWER         0x01
#define LIDAR_CMD_DISABLE_LOW_POWER        0x02
#define LIDAR_CMD_STATE_MODEL_MOTOR        0x05
#define LIDAR_CMD_ENABLE_CONST_FREQ        0x0E
#define LIDAR_CMD_DISABLE_CONST_FREQ       0x0F

#define LIDAR_CMD_GET_OFFSET_ANGLE          0x93
#define LIDAR_CMD_GET_RIB_OFFSET_ANGLE      0x94
#define LIDAR_CMD_SET_LOW_EXPOSURE          0x95
#define LIDAR_CMD_ADD_EXPOSURE       	    0x96
#define LIDAR_CMD_DIS_EXPOSURE       	    0x97


#define LIDAR_CMD_SET_HEART_BEAT        0xD9
#define LIDAR_CMD_SET_SETPOINTSFORONERINGFLAG  0xae

#define LIDAR_CMD_ENTER_TEXT 0xfa
#define LIDAR_CMD_EXIT 0x40

#define PackageSampleMaxLngth 0x100
typedef enum {
  CT_Normal = 0,
  CT_RingStart  = 1,
  CT_Tail,
} CT;
#define Node_Default_Quality (10)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA

#if defined(_WIN32)
#pragma pack(1)
#endif

struct node_info {
  uint8_t    sync_flag;  //sync flag
  uint16_t   sync_quality;//!intensity
  uint16_t   angle_q6_checkbit; //!angle
  uint16_t   distance_q; //! distance
  uint64_t   stamp; //! timestamp
  uint8_t    scan_frequence;//! scan frequency
  uint8_t    debugInfo;///< debug information
  uint8_t    index;///< package index
  uint8_t    error_package;///< error package state
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
  uint8_t   model; ///< lidar model
  uint16_t  firmware_version; ///< Firmware Major Version
  uint8_t   hardware_version; ///< Firmware Minjor Version
  uint8_t   serialnum[16];    ///< serial number
} __attribute__((packed)) ;

struct device_health {
  uint8_t   status; ///< health status
  uint16_t  error_code; ///< error code
} __attribute__((packed))  ;

struct sampling_rate {
  uint8_t rate;	///< sampling frequency
} __attribute__((packed))  ;

struct scan_frequency {
  uint32_t frequency;	///< scanning frequency
} __attribute__((packed))  ;

struct scan_rotation {
  uint8_t rotation;
} __attribute__((packed))  ;

struct scan_exposure {
  uint8_t exposure;	///<
} __attribute__((packed))  ;

struct scan_heart_beat {
  uint8_t enable;	///<
} __attribute__((packed));

struct scan_points {
  uint8_t flag;
} __attribute__((packed))  ;

struct function_state {
  uint8_t state;
} __attribute__((packed))  ;

struct offset_angle {
  int32_t angle;
} __attribute__((packed))  ;

struct rib_offset_angle {
  offset_angle angle[16];
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
  //angle[°]
  float angle;
  //range[m]
  float distance;
  float intensity;
//  uint64_t stamp;
  uint64_t stamp;
  LaserPoint &operator = (const LaserPoint &data) {
    this->angle = data.angle;
    this->distance = data.distance;
    this->intensity = data.intensity;
    this->stamp = data.stamp;
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
  //! Time between scans[s]
  float scan_time;
  //! Minimum range [m]
  float min_range;
  //! Maximum range [m]
  float max_range;
  int sequence;
  LaserConfig &operator = (const LaserConfig &data) {
    this->min_angle = data.min_angle;
    this->max_angle = data.max_angle;
    this->time_increment = data.time_increment;
    this->scan_time = data.scan_time;
    this->min_range = data.min_range;
    this->max_range = data.max_range;
    this->sequence = data.sequence;
    return *this;
  }
};

struct LaserScan {
  //! Array of laser data point
  std::vector<LaserPoint> data;
  //! System time when first range was measured in nanoseconds
  uint64_t system_time_stamp;
  //! Configuration of scan
  LaserConfig config;
  LaserScan &operator = (const LaserScan &data) {
    this->data = data.data;
    this->system_time_stamp = data.system_time_stamp;
    this->config = data.config;
    return *this;
  }
};

struct LaserDebug {
  int8_t TranIndex_W2F5CusVerMajor_W5F0CusVerMinor;		 		//用户版本，用于客户做协议兼容			    02[V0.1]
  int8_t TranIndex_W7F0ProtoMode;						 		//协议版本							be[01011111]
  int8_t TranIndex_W7F0Health;							 		//健康信息							0
  int8_t TranIndex_W3F4HardwareVer_W4F0FirewareMajor;			//硬件版本、固件大版本					66[H03V03]
  int8_t TranIndex_W7F0FirewareMinor;							//固件小版本							02[R01]
  int8_t TranIndex_W3F4MeasureMode_W4F0WorkMode;					//测距模式、工作模式						22[4K,Scan]
  int8_t TranIndex_W3F4MasterMode_W4F0PlatFormMode;				//模组型号、底座型号						20[S4,G2]
  int8_t TranIndex_W3F4FactorVer_W4F0LaserMode;					//工厂版本、激光管型号					20[F1,L0]
  int8_t TranIndex_W5F2Year_W2F0SN1;
  int8_t TranIndex_W4F3Month_W3F0SN2;
  int8_t TranIndex_W5F2Day_W2F0SN3;
  int8_t TranIndex_W7F0SN4;
  int8_t TranIndex_W7F0SN5;
  int8_t TranIndex_W7F0PDVal;									//PD电压
  int8_t TranIndex_W7F0LDVal;									//LD电压
  int8_t TranIndex_W7F0REFVal;
  int8_t MaxDebugIndex;
  LaserDebug &operator = (const LaserDebug &other) {
    this->TranIndex_W2F5CusVerMajor_W5F0CusVerMinor = other.TranIndex_W2F5CusVerMajor_W5F0CusVerMinor;
    this->TranIndex_W7F0ProtoMode = other.TranIndex_W7F0ProtoMode;
    this->TranIndex_W7F0Health =
      other.TranIndex_W7F0Health;
    this->TranIndex_W3F4HardwareVer_W4F0FirewareMajor = other.TranIndex_W3F4HardwareVer_W4F0FirewareMajor;
    this->TranIndex_W7F0FirewareMinor = other.TranIndex_W7F0FirewareMinor;
    this->TranIndex_W3F4MeasureMode_W4F0WorkMode = other.TranIndex_W3F4MeasureMode_W4F0WorkMode;
    this->TranIndex_W3F4MasterMode_W4F0PlatFormMode =
      other.TranIndex_W3F4MasterMode_W4F0PlatFormMode;
    this->TranIndex_W3F4FactorVer_W4F0LaserMode = other.TranIndex_W3F4FactorVer_W4F0LaserMode;
    this->TranIndex_W5F2Year_W2F0SN1 = other.TranIndex_W5F2Year_W2F0SN1;
    this->TranIndex_W4F3Month_W3F0SN2 = other.TranIndex_W4F3Month_W3F0SN2;
    this->TranIndex_W5F2Day_W2F0SN3 = other.TranIndex_W5F2Day_W2F0SN3;
    this->TranIndex_W7F0SN4 = other.TranIndex_W7F0SN4;
    this->TranIndex_W7F0SN5 = other.TranIndex_W7F0SN5;
    this->TranIndex_W7F0PDVal = other.TranIndex_W7F0PDVal;
    this->TranIndex_W7F0LDVal = other.TranIndex_W7F0LDVal;
    this->TranIndex_W7F0REFVal = other.TranIndex_W7F0REFVal;
    return *this;
  }
};

typedef enum  {
  NoError,//无错误
  DeviceNotFoundError,//无串口设备
  PermissionError,//串口权限异常
  OpenError,//串口打开失败
  ParityError,//串口校验位
  FramingError,
  BreakConditionError,
  WriteError,//写串口数据错误
  ReadError,//读串口数据异常
  ResourceError,//串口被占用
  UnsupportedOperationError,//不支持的操作
  TimeoutError,//超时(串口异常)
  NotOpenError,//串口没打开
  HeaderError,//包头错误
  FirstSampleAngleError,//采样角错误
  LastSampleAngleError,//采样角错误
  PackageNumberError,//采样数错误
  CheckSumError,//校验和错误
  SensorError,//雷达硬件错误
  EncodeError,//雷达卡主,
  PWRError,//供电异常
  PDError,
  LDError,
  DataError,//激光被遮挡
  TrembleError,//抖动(跳频）
  LidarNotFoundError,//雷达没发现
  UnknownError,//没初始化
} lidar_error_t;

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

inline lidar_error_t convert_ct_packet_to_error(int8_t  health_info) {
  lidar_error_t err = NoError;

  if (health_info & response_health_error::DataError) {
      err = DataError;
  }

  if (health_info & response_health_error::PWRError) {
      err = PWRError;
  }

  if (health_info & response_health_error::PDError) {
      err = PDError;
  }

  if (health_info & response_health_error::LDError) {
      err = LDError;
  }

  if (health_info & response_health_error::EncodeError) {
      err = EncodeError;
  }

  if (health_info & response_health_error::SensorError) {
      err = SensorError;
  }

  if (err == NoError) {
  }

  return err;
}

inline const char *getHealthError(lidar_error_t err) {
  char const *errorString = "Unknown error";

  switch (err) {
    case lidar_error_t::NoError:
      errorString = ("No error");
      break;

    case lidar_error_t::DataError:
      errorString = ("data error");
      break;

    case lidar_error_t::PWRError:
      errorString = ("PWR  error");
      break;


    case lidar_error_t::PDError:
      errorString = ("PDE error");
      break;

    case lidar_error_t::LDError:
      errorString = ("LD  error");
      break;

    case lidar_error_t::EncodeError:
      errorString = ("Encode error");
      break;

    case lidar_error_t::SensorError:
      errorString = ("sensor error");
      break;
   case lidar_error_t::CheckSumError:
      errorString = ("check sum error");

    default:
      // an empty string will be interpreted as "Unknown error"
      break;
  }

  return errorString;
}

/** The numeric version information struct.  */
typedef struct {
  uint8_t  hardware;   /**< Hardware version*/
  uint8_t  firmware;      /**< Firmware Version */
  uint8_t  sn[16];     /**< serial number*/
} LidarVersion;
