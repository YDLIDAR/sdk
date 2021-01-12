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
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>


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


using namespace ydlidar;

class YDLIDAR_API CYdLidar {
  PropertyBuilderByName(float, MaxRange,
                        private) ///< 设置和获取激光最大测距范围
  PropertyBuilderByName(float, MinRange,
                        private) ///< 设置和获取激光最小测距范围
  PropertyBuilderByName(float, MaxAngle,
                        private) ///< 设置和获取激光最大角度, 最大值180度
  PropertyBuilderByName(float, MinAngle,
                        private) ///< 设置和获取激光最小角度, 最小值-180度
  PropertyBuilderByName(float, OffsetTime, private) ///< 设置时间戳偏移值

  PropertyBuilderByName(float, ScanFrequency,
                        private)///< 设置雷达扫描频率(S2Pro雷达支持)

  PropertyBuilderByName(bool, FixedResolution,
                        private) ///< 设置和获取激光是否是固定角度分辨率
  PropertyBuilderByName(bool, Reversion,
                        private) ///< 设置和获取是否旋转激光激光数据180度
  PropertyBuilderByName(bool, AutoReconnect,
                        private) ///< 设置异常是否开启重新连接
  PropertyBuilderByName(bool, GlassNoise,
                        private) ///< 设置是否开启玻璃干扰标志过滤
  PropertyBuilderByName(bool, SunNoise,
                        private) ///< 设置是否开启阳光干扰标志过滤

  PropertyBuilderByName(int, SerialBaudrate,
                        private) ///< 设置和获取激光通讯波特率
  PropertyBuilderByName(std::string, SerialPort,
                        private) ///< 设置和获取激光端口号
  PropertyBuilderByName(std::vector<float>, IgnoreArray,
                        private) ///< 设置和获取激光剔除点
  PropertyBuilderByName(int, AbnormalCheckCount,
                        private) ///< 最大检查雷达数据的是否异常的次数
  PropertyBuilderByName(int, Intensity,
                        private) ///< 是否带信号质量(-1:自动检测(不推荐), 0:不带型号强度, 1:带信号强度)
  ///换算到时间就是(AbnormalCheckCount *500 + 50*(2+...+ AbnormalCheckCount -1))ms
  /// 比如设置到8， 换算到时间=（8*500 + 50*(2+3+4+5+6+7))= 5350ms,
  /// 也就是如果5s中之内没有数据或者有数据雷达数据一直异常，则turnOn失败。
  /// 如果设置到2， 时间=(2*500)= 1000ms, 如果1s之内没有数据或者数据异常，
  /// 则turnOn返回失败。因为启动会有延时， 可能有时启动时间大于1s, 则设置到2时，
  /// 会出现turnOn失败。

 public:
  CYdLidar(); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.

  bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(LaserScan &scan_msg, bool &hardwareError);

  //Turn on the motor enable
  bool  turnOn();  //!< See base class docs
  //Turn off the motor enable and close the scan
  bool  turnOff(); //!< See base class docs

  //!<Power off the lidar
  int  PoweroffPWM();

  // exchange error to error_string
  string getErrorString(int error);

  //initialize pwd path
  void  initPwdPath(int number);

  //get fixed resolution node size
  int getFixedSize() const;

  /**
  * @brief Return LiDAR's version information in a numeric form and whether sn is ok.
  * @param version Pointer to a version structure for returning the version information.
  */
  bool GetLidarVersion(LidarVersion &version);

  /**
   * @brief getDriverError
   * @return
   */
  lidar_error_t getDriverError() const;


  //Turn off lidar connection
  void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

  typedef  enum {
      NoError = 0,
      WriteExportError,
      WriteEnableError,
      WriteDutyError,
      WritePeriodError,
      WriteModeError,
      WriteUnenabelError,
  } PIDError;

  ///<初始化PID调速的参数
  CYdLidar::PIDError initPIDParams();

 protected:
  /** Returns true if communication has been established with the device. If it's not,
    *  try to create a comms channel.
    * \return false on error.
    */
  bool  checkCOMMs();

  /** Returns true if health status and device information has been obtained with the device. If it's not,
    * \return false on error.
    */
  bool  checkStatus();

  /**
   * @brief checkScanFrequency
   * @return
   */
  bool checkScanFrequency();

  /**
   * @brief checkZeroOffsetAngle
   * @return
   */
  bool checkZeroOffsetAngle();

  /** Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  bool checkHardware();

  /**
   * @brief checkHealth
   * @param info
   * @return
   */
  bool checkHealth(ct_packet_t &info);

  /** returns true if the lidar data is normal, If it's not*/
  bool checkLidarAbnormal();

  /** Returns true if the device is in good health, If it's not*/
  bool getDeviceHealth(uint32_t timeout = 500);

  /** Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo(uint32_t timeout = 500);





 private:
  ydlidar::YDlidarDriver *lidarPtr;
  LaserFan               laser_packages;
  uint32_t               point_interval_time;
  uint32_t               package_transfer_time;///零位包传送时间
  uint64_t               last_node_time;
  int                    fixed_size;
  int                    sample_rate;
  float                  frequency_offset;
  float                  zero_offset_angle;
  bool                   single_channel;
  bool                   isScanning;
  bool                   isConnected;
  LidarVersion           m_LidarVersion;      ///< LiDAR Version information
  int16_t                    default_mode_duty;
  bool                   sn_status;

  string            ExportPath;
  string            PeriodPath;
  string            DutyPath;
  string            ModePath;
  string            EnablePath;
  string            PWMExportPath;
  string            UnexportPath;

};	// End of class

