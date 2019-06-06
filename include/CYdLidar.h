
#pragma once
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>
#include "matrix/math.hpp"

using namespace ydlidar;

class YDLIDAR_API CYdLidar {
  PropertyBuilderByName(float, MaxRange,
                        private) ///< 设置和获取激光最大测距范围(m)
  PropertyBuilderByName(float, MinRange,
                        private) ///< 设置和获取激光最小测距范围(m)
  PropertyBuilderByName(float, MaxAngle,
                        private) ///< 设置和获取激光最大角度, 最大值180度(度)
  PropertyBuilderByName(float, MinAngle,
                        private) ///< 设置和获取激光最小角度, 最小值-180度(度)
  PropertyBuilderByName(bool, FixedResolution,
                        private) ///< 设置和获取激光是否是固定角度分辨率
  PropertyBuilderByName(int, FixedCount,
                        private) ///< 设置固定点数
  PropertyBuilderByName(bool, AutoReconnect,
                        private) ///< 设置异常是否开启重新连接
  PropertyBuilderByName(int, SerialBaudrate,
                        private) ///< 设置和获取激光通讯波特率
  PropertyBuilderByName(int, AbnormalCheckCount,
                        private) ///< Maximum number of abnormal checks
  PropertyBuilderByName(std::string, SerialPort,
                        private) ///< 设置和获取激光端口号
  PropertyBuilderByName(std::vector<float>, IgnoreArray,
                        private) ///< 设置和获取激光剔除点


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

  /**
   * @brief 输入的纠正数据源， imu数据或者误差很小的里程计数据
   * @param odom
   */
  void setOdometry(const odom_t &odom);

  /**
   * @brief 雷达相对于机器人坐标系的位置
   * @param pose
   */
  void setLidarPose(const pose2D_t &pose);

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


  /** Returns true if the device is in good health, If it's not*/
  bool getDeviceHealth();

  /** Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo();

  /** returns true if the lidar data is normal, If it's not*/
  bool checkLidarAbnormal();

 private:
  bool    isScanning;
  int     node_counts ;
  double  each_angle;
  float   frequencyOffset;
  double  m_ScanFrequency;
  uint8_t Major;
  uint8_t Minjor;
  YDlidarDriver *lidarPtr;

  matrix::SquareMatrix<double, 3> sensor_matrix;
  matrix::SquareMatrix<double, 3> sensor_matrix_inv;
  matrix::SquareMatrix<double, 3> robot_matrix;
  matrix::Vector<double, 3> lidar_sensor_vector;
  matrix::Vector<double, 3> current_sensor_vector;

};	// End of class

