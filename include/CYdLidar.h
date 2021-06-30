
#pragma once
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>
#include <SimpleIni.h>
#include "angles.h"

using namespace ydlidar;
using namespace angles;

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


 public:
  CYdLidar(); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.
  /**
   * @brief initialize
   * @return
   */
  bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

  /** Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  int checkHardware();

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(LaserScan &outscan, bool &hardwareError,lidar_error_t &healthError);

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

  /**
   * @brief Return LiDAR's version information in a numeric form.
   * @param version Pointer to a version structure for returning the version information.
   */
  void GetLidarVersion(LidarVersion &version);

  /**
  * @brief fetches zero angle tolerance values(±1.5°↓) from lidar’s internal memory while lidar assembly \n
  * @param[in] angle　　　   zero offset angle
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  Failed
  * @retval RESULT_FAILE    Angle is not calibrated
  * @note Non-scan state, perform currect operation.
  */
  result_t getZeroOffsetAngle(offset_angle &angle,
                              uint32_t timeout = 5 * YDlidarDriver::DEFAULT_TIMEOUT);

  /**
  * @brief save robot zero offset angle \n
  * @param[in] angle　　　   robot offset angle
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  Failed
  * @retval RESULT_FAILE    Angle is not Saved
  * @note Non-scan state, perform currect operation.
  */
  result_t saveRobotOffsetAngle(offset_angle &angle,
                                uint32_t timeout = 5 * YDlidarDriver::DEFAULT_TIMEOUT);

  /**
   * @brief getRibOffsetAngle
   * @param angle
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_TIMEOUT  Failed
   * @retval RESULT_FAILE    Rib is not calibrated
   * @return
   */
  result_t getRibOffsetAngle(std::vector<offset_angle> &angle,
                             uint32_t timeout = 5 * YDlidarDriver::DEFAULT_TIMEOUT);

  /**
   * @brief saveRibOffsetAngle
   * @param angle
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_TIMEOUT  Failed
   * @retval RESULT_FAILE    Rib is not Saved
   * @return
   */
  result_t saveRibOffsetAngle(std::vector<offset_angle> &angle,
                              uint32_t timeout = 5 * YDlidarDriver::DEFAULT_TIMEOUT);

 protected:
  /** Returns true if communication has been established with the device. If it's not,
    *  try to create a comms channel.
    * \return false on error.
    */
  bool  checkCOMMs();

  /**
   * @brief checkLidarModel
   * @return
   */
  bool checkLidarModel();

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
  bool checkCalibrationAngle(const std::string &serialNumber);

  /**
   * @brief checkRibOffsetAngle
   */
  void checkRibOffsetAngle();

  /** Returns true if the device is in good health, If it's not*/
  bool getDeviceHealth();

  /** Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo();

  /** print Version information */
  bool printfVersionInfo(const device_info &info);

  /** parsing version by package*/
  void handleVersionInfoByPackage(const LaserDebug &debug);

  /** Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
  bool checkScanFrequency();

  /** returns true if the lidar data is abnormal, If it's not*/
  int checkLidarAbnormal();

  /** returns true if the lidar data is nornal*/
  bool checkLidarData();

 private:
  bool    isScanning;
  float   frequencyOffset;
  float   m_AngleOffset;
  bool    m_isAngleOffsetCorrected;
  bool    m_isRibOffsetCorrected;
  uint8_t Major;
  uint8_t Minjor;
  uint8_t m_Model;
  CSimpleIniA ini;
  YDlidarDriver *lidarPtr;
  Locker         lidar_lock;
  Locker         del_lidar_lock;
  std::string m_serial_number;
  node_info *nodes;
  bool m_ParseSuccess;
  bool m_SingleChannel;
  bool m_PrintError;
  uint32_t errTime;
  uint32_t laserFailureTime;
  uint32_t hasLaserFailure;
  LidarVersion m_LidarVersion;      ///< LiDAR Version information
  Thread data_thread_;				///< thread id
  Locker data_lock_;
  bool check_thread_running_;
  bool lidar_data_nomal;
  uint32_t m_turn_on_time;
};	// End of class

