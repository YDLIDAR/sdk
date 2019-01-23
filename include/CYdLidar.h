
#pragma once
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>

using namespace ydlidar;

class YDLIDAR_API CYdLidar {
  PropertyBuilderByName(float, MaxRange, private) ///< 设置和获取激光最大测距范围(m)
  PropertyBuilderByName(float, MinRange, private) ///< 设置和获取激光最小测距范围(m)
  PropertyBuilderByName(float, MaxAngle,
                        private) ///< 设置和获取激光最大角度, 最大值180度(度)
  PropertyBuilderByName(float, MinAngle,
                        private) ///< 设置和获取激光最小角度, 最小值-180度(度)
  PropertyBuilderByName(bool, Intensities,
                        private) ///< 设置和获取激光带信号质量(只有S4B雷达支持)
  PropertyBuilderByName(bool, FixedResolution,
                        private) ///< 设置和获取激光是否是固定角度分辨率
  PropertyBuilderByName(bool, AutoReconnect, private) ///< 设置异常是否开启重新连接
  PropertyBuilderByName(bool, GlassNoise, private) ///< 设置是否关闭玻璃噪声干扰
  PropertyBuilderByName(bool, SunNoise, private) ///< 设置是否关闭太阳干扰
  PropertyBuilderByName(int, SerialBaudrate, private) ///< 设置和获取激光通讯波特率
  PropertyBuilderByName(std::string, SerialPort, private) ///< 设置和获取激光端口号
  PropertyBuilderByName(std::vector<float>, IgnoreArray, private) ///< 设置和获取激光剔除点


 public:
  CYdLidar(); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.
  /**
   * @brief initialize
   * @return
   */
  bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(LaserScan &outscan, bool &hardwareError);

  //Turn on the motor enable
  bool  turnOn();  //!< See base class docs

  //Turn off the motor enable and close the scan
  bool  turnOff(); //!< See base class docs

  //Turn off lidar connection
  void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

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

  /** Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  bool checkHardware();

  /** Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo();


  /** returns true if the lidar data is normal, If it's not*/
  bool checkLidarAbnormal();

 private:
  bool    isScanning;
  int     node_counts ;
  double  each_angle;
  YDlidarDriver *lidarPtr;

};	// End of class

