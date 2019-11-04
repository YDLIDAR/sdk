﻿
#pragma once
#include "line_feature.h"
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>
#include <SimpleIni.h>
#include "angles.h"
#include <functional>

using namespace ydlidar;
using namespace line_feature;
using namespace angles;

/**
 * @brief LIDARCtrlFreqCallback
 */
typedef std::function<void (bool/*Low frequency control*/)>
LIDARCtrlFreqCallback;

class YDLIDAR_API CYdLidar {
  PropertyBuilderByName(float, MaxRange,
                        private) ///< Constrained maximum distance(m)
  PropertyBuilderByName(float, MinRange,
                        private) ///< Constrained minimum distance(m)
  PropertyBuilderByName(float, MaxAngle,
                        private) ///< constrained maximum angle, Maximum 360 Deg(Deg)
  PropertyBuilderByName(float, MinAngle,
                        private) ///< constrained minimum angle, minmum 0 Deg(Deg)
  PropertyBuilderByName(float, ScanFrequency,
                        private) ///< scan frequency (5HZ~12HZ)(HZ)

  PropertyBuilderByName(float, RobotLidarDifference,
                        private) ///< lidar zero and robot zero difference(°)

  PropertyBuilderByName(bool, Intensities,
                        private) ///< intensity
  PropertyBuilderByName(bool, AutoReconnect,
                        private) ///< whether to support hot swap
  PropertyBuilderByName(bool, GlassNoise,
                        private) ///< whether to close glass noise
  PropertyBuilderByName(bool, SunNoise, private) ///< whether to close sun noise
  PropertyBuilderByName(int, SerialBaudrate, private) ///< serial baudrate
  PropertyBuilderByName(int, SampleRate, private) ///< sampling rate(KHz)
  PropertyBuilderByName(int, AbnormalCheckCount,
                        private) ///< Maximum number of abnormal checks
  PropertyBuilderByName(std::string, CalibrationFileName,
                        private) ///< calibration file
  PropertyBuilderByName(std::string, SerialPort, private) ///< serial port
  PropertyBuilderByName(std::vector<float>, IgnoreArray,
                        private) ///< Culling angle list

  PropertyBuilderByName(bool, CurrentFrequencyStatus,
                        private) ///< intensity
  PropertyBuilderByName(bool, LastFrequencyStatus,
                        private) ///< intensity
  PropertyBuilderByName(bool, ChangeFrequency,
                        private) ///< intensity
  PropertyBuilderByName(bool, CheckFinished,
                        private) ///< intensity
  PropertyBuilderByName(int, MaxLowFrequencyTimes, private) ///< serial baudrate
  PropertyBuilderByName(int, MaxHightFrequencyTimes,
                        private) ///< serial baudrate
  PropertyBuilderByName(int, MaxEchoTimes, private) ///< serial baudrate
  PropertyBuilderByName(float, MinFrequency,
                        private) ///< Constrained minimum distance(m)
  PropertyBuilderByName(float, MaxFrequency,
                        private) ///< Constrained minimum distance(m)
  PropertyBuilderByName(bool, Result,
                        private) ///< 修正是否成功





 public:
  CYdLidar(); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.
  enum CheckStateError {
    NOERROR = 0,//No Error, 没有错误
    FREQUENCYOUT = 1, //Frequency out of range, 频率超时范围
    JUMPFREQUENCY = 2, // jump frequency. 调频错误
    MINFREQERROR = 3, //min frequency error...., 最小频率错误
    MAXFREQERROR = 4,//max frequency error...., 最大频率错误
    MINMAXFREQERROR = 5,//min max frequency error, 最小最大频率错误
    OUTOFRANGE   = 6,//out of range, 超出距离范围
    TIMEOUT      = 7,//timeout, 超时
    MAXSTDEVERROR = 8,//数据抖动
    AUTOCHECKFAILED = 9,//雷达自检错误
    NOISE           = 10,//干扰出现
  };

  enum CheckFreqState {
    ENTER_STATE = 0,
    SAVE_STATE = 1,
    CANCEL_STATE = 2,
    SENSOR_STATE = 3,
    ONETIMES_STATE = 4,
  };
  /**
   * @brief initialize
   * @return
   */
  bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

  /** Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  bool checkHardware();

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(LaserScan &outscan, bool &hardwareError);

  //Turn on the motor enable
  bool  turnOn();  //!< See base class docs

  //Turn off the motor enable and close the scan
  bool  turnOff(); //!< See base class docs

  //Turn off lidar connection
  void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

  //lidar pointer
  YDlidarDriver *getYdlidarDriver();

  //get zero angle offset value
  float getAngleOffset() const;

  //Whether the zero offset angle is corrected?
  bool isAngleOffetCorrected() const;

  // get lidar relative robot offset angle
  float getRobotAngleOffset() const;

  // start lidar is corrected relative to the robot.
  void setStartRobotAngleOffset();

  //Whether the lidar relative robot offset angle is corrected?
  //After the corrrection is started,
  //the currect interface can be used to datermine when the correction is completed.
  bool isRobotAngleOffsetCorrected() const;

  /**
   * @brief RegisterContrlFreqCallback
   * @param callback
   */
  void RegisterCtrlFreqCallback(LIDARCtrlFreqCallback callback);

  CheckStateError getCheckStateError() const {//获取错误状态
    return m_check_state_error;
  }

  bool getSartUpState() const {//获取当前开启状态是否正在进行中
    return m_action_startup;
  }

  //是否在修正中
  bool IsCheckingFinished();


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
   * @brief checkSampleRate
   */
  void checkSampleRate();

  /**
   * @brief checkCalibrationAngle
   */
  void checkCalibrationAngle(const std::string &serialNumber);

  /**
   * @brief checkRobotOffsetAngleCorrected
   * @param serialNumber
   */
  void checkRobotOffsetAngleCorrected(const std::string &serialNumber);

  /** Returns true if the device is in good health, If it's not*/
  bool getDeviceHealth();

  /** Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo();

  /** Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
  bool checkScanFrequency();

  /** returns true if the lidar data is normal, If it's not*/
  bool checkLidarAbnormal();

  /**
   * @brief saveRobotOffsetAngle
   * @return
   */
  void saveRobotOffsetAngle();

  /**
   * @brief hangleRobotOffsetAngle
   */
  void handleRobotOffsetAngle();

 private:

  void lowSpeed();//雷达低速命令
  void hightSpeed();//雷达高速命令

  void startCorrectionMod();//开启修正模式
  void OnEnter(double frequency);//进入开启修正模式
  void ActionStateUpdate(double frequency);
  void setActionState(bool isLowerSpeed = true);
  bool CheckStateTimeout(bool isLowerSpeed = true);


  int m_action_step;//当前循环次数
  bool m_action_startup;//是否开启修正
  bool has_check_flag;//是否有修正模式
  int m_action_state;//当前开启修正模式状态
  uint64_t action_check_time;//开始修正模式计数时时间

  const int64_t min_check_time = 45 * 1e8;//最小开启修正模式等待时间
  const int64_t max_check_time = 13 * 1e9;//最大开启修正模式等待时间
  const int max_action_step = 3;//开启修正模式最大循环次数
  const double m_min_action_frequency = 6.8;//开启修正模式最小频率
  const double m_max_action_frequency = 11.7;//开启修正模式最大频率

  const double m_min_check_frequency = 6.8;//修正最下频率
  const double m_max_check_frequency = 11.7;//修正最大频率

  const int MaxPlusTimes = 5;
  const int MaxCancelPlusTimes = 50;//修正值错误,取消重新修正


 private:
  std::vector<double> m_angle_threshold;//角度区域通讯值
  std::vector<int> check_queue_size,
      auto_check_sum_queue, auto_check_distance;//处理对应角度区域距离值
  double current_frequency;//当前雷达频率
  double last_frequency;//上一次雷达频率
  bool has_check_state;//是否在修正模式
  CheckState m_state;//当前修正状态
  CheckState m_last_state;//最新修正状态
  CheckStateError m_check_state_error;//修正状态错误代码

  const int MAXSTARTCHECKCOUNT = 5;//最大收集次数
  int start_check_count;//收集雷达次数(算距离均值)
  int change_frequency_times;//切换频率次数
  int echo_frequency_times;//切换频率循环次数
  uint64_t last_change_time;//开始修正模式计数时时间

  float getCurrectTimeDiff() const;


  std::vector<double> m_Calibration_angle, m_Calibration_distance,
      m_last_check_distance, m_percentage;//修正角度和修正距离容器,
  std::vector<double> m_pass_percentage, m_check_percentage;
  std::vector<std::vector<double>> m_distance_queue,
      m_mean_distance_queue;//距离队列和距离均值队列
  const double MaxStdDev = 120;//最大数据抖动
  const double MaxEntryDiff = 500;//最大距离误差

  CheckFreqState m_check_freq_state;//切换频率状态机
  bool last_check_state;//
  int state_distance;

  void retSetData();
  void resetCheckState();
  void handleScanData(double angle, double distance);
  void handleCheckData();
  void handleCheckStep();
  void handleCheckDis();
  void handleLidarDis(int distance);
  void handleLidarSureDis(int distance);
  void handleCheckState();
  void updateCheckState();
  void handleCalibrationState();//处理修正状态
  void handleFrequencySM(double frequency);
  void autoChangeFequency(double frequency);//切换雷达频率
  void checkFrequencyState(double frequency);//检测切换频率是否异常


 private:
  bool    isScanning;
  float   frequencyOffset;
  float   m_AngleOffset;
  bool    m_isAngleOffsetCorrected;
  float   m_LRRAngleOffset;
  bool    m_isLRRAngleOffsetCorrected;//lidar relative robot angle offset
  bool    m_startRobotAngleOffset;
  uint8_t Major;
  uint8_t Minjor;
  CSimpleIniA ini;
  YDlidarDriver *lidarPtr;
  LineFeature line_feature_;
  std::string m_serial_number;

  std::vector<double> bearings;
  std::vector<unsigned int> indices;
  RangeData range_data;

  //controller frequency callback
  LIDARCtrlFreqCallback m_freq_callback;//频率切换回调函数

  node_info *nodes;


};	// End of class

