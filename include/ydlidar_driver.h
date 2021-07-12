/** @mainpage

    <table>
        <tr><th>Library     <td>YDlidarDriver
        <tr><th>File        <td>YDlidarDriver.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/sdk
        <tr><th>Version     <td>2.0.8
    </table>

    Jump to the @link ::ydlidar::YDlidarDriver @endlink and @link ::CYdLidar @endlink interface documentation.

*/

#ifndef YDLIDAR_DRIVER_H
#define YDLIDAR_DRIVER_H
#include <stdlib.h>
#include <atomic>
#include <map>
#include "serial.h"
#include "locker.h"
#include "thread.h"
#include "ydlidar_protocol.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif


using namespace std;
using namespace serial;

namespace ydlidar {

std::string format(const char *fmt, ...);


class YDlidarDriver {
  ///< single-channel
  PropertyBuilderByName(bool, SingleChannel, private)

 public:
  enum DriverError {
    NoError = 0,
    DeviceNotFoundError,
    PermissionError,
    UnsupportedOperationError,
    UnknownError,
    TimeoutError,
    NotOpenError,
    BlockError,
    NotBufferError,
    TrembleError,
    LaserFailureError,
    CheckSumError,
  };

  static const char *DescribeError(DriverError err);

 public:
  /**
  * A constructor.
  * A more elaborate description of the constructor.
  */
  YDlidarDriver();

  /**
  * A destructor.
  * A more elaborate description of the destructor.
  */
  virtual ~YDlidarDriver();

  /**
  * @brief Connecting Lidar \n
  * After the connection if successful, you must use ::disconnect to close
  * @param[in] port_path    serial port
  * @param[in] baudrate    serial baudrate，YDLIDAR-SS：
  *     230400 G2-SS-1 R2-SS-1
  * @return connection status
  * @retval 0     success
  * @retval < 0   failed
  * @note After the connection if successful, you must use ::disconnect to close
  * @see function ::YDlidarDriver::disconnect ()
  */
  result_t connect(const char *port_path, uint32_t baudrate);

  /**
  * @brief disconnect th lidar
  */
  void disconnect();

  /**
  * @brief Get SDK Version \n
  * static function
  * @return Version
  */
  static std::string getSDKVersion();

  /**
  * @brief lidarPortList Get Lidar Port lists
  * @return online lidars
  */
  static std::map<std::string, std::string> lidarPortList();


  /**
  * @brief Is the Lidar in the scan \n
  * @return scanning status
  * @retval true     scanning
  * @retval false    non-scanning
  */
  bool isscanning() const;

  /**
  * @brief Is it connected to the lidar \n
  * @return connection status
  * @retval true     connected
  * @retval false    Non-connected
  */
  bool isconnected() const;

  /**
  * @brief Is there intensity \n
  * @param[in] isintensities    intentsity
  *   true	intensity
  *	  false no intensity
  */
  void setIntensities(const bool &isintensities);

  /**
  * @brief whether to support hot plug \n
  * @param[in] enable    hot plug :
  *   true	support
  *	  false no support
  */
  void setAutoReconnect(const bool &enable);

  /**
   * @brief setIgnoreArray
   * set the range of angles that need to be removed.
   * @param ignore_array
   * usage: [0, 10, 50, 65, 180, 189, 348, 359]
   * angle list
   */
  void setIgnoreArray(const std::vector<float> ignore_array);

  /**
  * @brief get Health status \n
  * @return result status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
  */
  result_t getHealth(device_health &health, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief get Device information \n
  * @param[in] info     Device information
  * @param[in] timeout  timeout
  * @return result status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
  */
  result_t getDeviceInfo(device_info &info, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief Turn on scanning \n
  * @param[in] force    Scan mode
  * @param[in] timeout  timeout
  * @return result status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Just turn it on once
  */
  result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

  /**
  * @brief turn off scanning \n
  * @return result status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t stop();


  /**
  * @brief Get a circle of laser data \n
  * @param[in] nodebuffer Laser data
  * @param[in] count      one circle of laser points
  * @param[in] timeout    timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Before starting, you must start the start the scan successfully with the ::startScan function
  */
  result_t grabScanData(node_info *nodebuffer, size_t &count, int *const seq,
                        uint32_t timeout = DEFAULT_TIMEOUT) ;


  /**
  * @brief Normalized angle \n
  * Normalize the angel between 0 and 360
  * @param[in] nodebuffer Laser data
  * @param[in] count      one circle of laser points
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Before the normalization, you must use the ::grabScanData function to get the laser data successfully.
  */
  result_t ascendScanData(node_info *nodebuffer, size_t count);

  /**
  * @brief reset lidar \n
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Non-scan state, perform currect operation.
  */
  result_t reset(uint32_t timeout = DEFAULT_TIMEOUT);


  /**
   * @brief start motor \n
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  result_t startMotor();

  /**
   * @brief stop motor \n
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  result_t stopMotor();

  /**
  * @brief Get lidar scan frequency \n
  * @param[in] frequency    scanning frequency
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Non-scan state, perform currect operation.
  */
  result_t getScanFrequency(scan_frequency &frequency,
                            uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief Increase the scanning frequency by 1.0 HZ \n
  * @param[in] frequency    scanning frequency
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Non-scan state, perform currect operation.
  */
  result_t setScanFrequencyAdd(scan_frequency &frequency,
                               uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief Reduce the scanning frequency by 1.0 HZ \n
  * @param[in] frequency    scanning frequency
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Non-scan state, perform currect operation.
  */
  result_t setScanFrequencyDis(scan_frequency &frequency,
                               uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief Increase the scanning frequency by 0.1 HZ \n
  * @param[in] frequency    scanning frequency
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Non-scan state, perform currect operation.
  */
  result_t setScanFrequencyAddMic(scan_frequency &frequency,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief Reduce the scanning frequency by 0.1 HZ \n
  * @param[in] frequency    scanning frequency
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Non-scan state, perform currect operation.
  */
  result_t setScanFrequencyDisMic(scan_frequency &frequency,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief Get lidar sampling frequency \n
  * @param[in] frequency    sampling frequency
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Non-scan state, perform currect operation.
  */
  result_t getSamplingRate(sampling_rate &rate,
                           uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief Set the lidar sampling frequency \n
  * @param[in] rate    　　　sampling frequency
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Non-scan state, perform currect operation.
  */
  result_t setSamplingRate(sampling_rate &rate,
                           uint32_t timeout = DEFAULT_TIMEOUT);

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
                              uint32_t timeout = 5 * DEFAULT_TIMEOUT);

  /**
  * @brief get lidar zero offset angle \n
  * @param[in] angle　　　   zero offset angle
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  Failed
  * @retval RESULT_FAILE    Angle is not calibrated
  * @note Non-scan state, perform currect operation.
  */
  result_t getLidarZeroOffsetAngle(offset_angle &angle,
                                   uint32_t timeout = DEFAULT_TIMEOUT);

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
                                uint32_t timeout = 5 * DEFAULT_TIMEOUT);

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
                             uint32_t timeout = 5 * DEFAULT_TIMEOUT);

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
                              uint32_t timeout = 5 * DEFAULT_TIMEOUT);

  /**
   * @brief getSystemError
   * @param systemErrorCode
   * @return
   */
  YDlidarDriver::DriverError getSystemError();

 protected:

  /**
  * @brief Data parsing thread \n
  * @note Before you create a dta parsing thread, you must use the ::startScan function to start the lidar scan successfully.
  */
  result_t createThread();


  /**
  * @brief Automatically reconnect the lidar \n
  * @param[in] force    scan model
  * @param[in] timeout  timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Lidar abnormality automatically reconnects.
  */
  result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

  /**
  * @brief stopScan
  * @param timeout
  * @return
  */
  result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief Unpacking \n
  * @param[in] node lidar point information
  * @param[in] timeout     timeout
  */
  result_t waitPackage(node_info *node, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief get unpacked data \n
  * @param[in] nodebuffer laser node
  * @param[in] count      lidar points size
  * @param[in] timeout      timeout
  * @return result status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  timeout
  * @retval RESULT_FAILE    failed
  */
  result_t waitScanData(node_info *nodebuffer, size_t &count,
                        uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief data parsing thread \n
  */
  int cacheScanData();

  /**
  * @brief send data to lidar \n
  * @param[in] cmd 	 command code
  * @param[in] payload      payload
  * @param[in] payloadsize      payloadsize
  * @return result status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t sendCommand(uint8_t cmd, const void *payload = NULL,
                       size_t payloadsize = 0);

  /**
  * @brief waiting for package header \n
  * @param[in] header 	 package header
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  timeout
  * @retval RESULT_FAILE    failed
  * @note when timeout = -1, it will block...
  */
  result_t waitResponseHeader(lidar_ans_header *header,
                              uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief Waiting for the specified size data from the lidar \n
  * @param[in] data_count 	 wait max data size
  * @param[in] timeout    	 timeout
  * @param[in] returned_size   really data size
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  wait timeout
  * @retval RESULT_FAILE    failed
  * @note when timeout = -1, it will block...
  */
  result_t waitForData(size_t data_count, uint32_t timeout = DEFAULT_TIMEOUT,
                       size_t *returned_size = NULL);

  /**
  * @brief get data from serial \n
  * @param[in] data 	 data
  * @param[in] size    date size
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t getData(uint8_t *data, size_t size);

  /**
  * @brief send data to serial \n
  * @param[in] data 	 data
  * @param[in] size    data size
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t sendData(const uint8_t *data, size_t size);


  /**
  * @brief checkTransferDelay
  */
  void checkTransferDelay();

  /**
  * @brief disable Data scan channel \n
  */
  void disableDataGrabbing();

  /**
  * @brief set DTR \n
  */
  void setDTR();

  /**
  * @brief clear DTR \n
  */
  void clearDTR();

  /**
   * @brief flushSerial
   */
  void flushSerial();

  void UpdateDriverError(const YDlidarDriver::DriverError &error);

  /**
   * @brief checkAutoConnecting
   */
  result_t checkAutoConnecting(bool error = false);

  /**
   * @brief checkLaserFailure
   */
  void checkLaserFailure();

  /**
  * @brief get lidar zero offset angle \n
  * @param[in] angle　　　   zero offset angle
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  Failed
  * @retval RESULT_FAILE    Angle is not calibrated
  * @note Non-scan state, perform currect operation.
  */
  result_t getZeroOffsetZone(offset_angle &angle,
                             uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief save lidar zero offset angle to lidar memory\n
  * @param[in] angle　　　   zero offset angle
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  Failed
  * @retval RESULT_FAILE    Angle is not calibrated
  * @note Non-scan state, perform currect operation.
  */
  result_t saveZeroOffsetZone(offset_angle &angle,
                              uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief get lidar rib zone \n
  * @param[in] angle　　　   rib offset angle
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  Failed
  * @retval RESULT_FAILE    Angle is not calibrated
  * @note Non-scan state, perform currect operation.
  */
  result_t getRibOffsetZone(rib_offset_angle &angle,
                            uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief save lidar rib zone to lidar memory\n
  * @param[in] angle　　　   rib offset angle
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  Failed
  * @retval RESULT_FAILE    Angle is not calibrated
  * @note Non-scan state, perform currect operation.
  */
  result_t saveRibOffsetZone(rib_offset_angle &angle,
                             uint32_t timeout = DEFAULT_TIMEOUT);


  /**
  * @brief save robot offset zone to lidar memory\n
  * @param[in] angle　　　   robot offset angle
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  Failed
  * @retval RESULT_FAILE    Angle is not calibrated
  * @note Non-scan state, perform currect operation.
  */
  result_t saveRobotOffsetZone(offset_angle &angle,
                               uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief get robot offset zone \n
  * @param[in] angle　　　   robot offset angle
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  Failed
  * @retval RESULT_FAILE    Angle is not calibrated
  * @note Non-scan state, perform currect operation.
  */
  result_t getRobotOffsetZone(offset_angle &angle,
                              uint32_t timeout = DEFAULT_TIMEOUT);

 private:
  /**
   * @brief setLidarReboot
   * @param timeout
   * @return
   */
  result_t setLidarReboot(uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief setScanSwitchModel
   * @param buffer
   * @param size
   * @param count
   * @param timeout
   * @return
   */
  result_t setScanSwitchModel(std::string &buffer, size_t &size,
                              uint32_t count = DEFAULT_WAIT_COUNT,
                              uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief setScanExitModel
   * @param buffer
   * @param size
   * @param count
   * @param timeout
   * @return
   */
  result_t setScanExitModel(std::string &buffer, size_t &size,
                            uint32_t count = DEFAULT_WAIT_COUNT,
                            uint32_t timeout = DEFAULT_TIMEOUT);
  /**
   * @brief setScanSendText
   * @param data
   * @param buffer
   * @param size
   * @param count
   * @param timeout
   * @return
   */
  result_t setScanSendText(const std::string &data,  std::string &buffer,
                           size_t &size,
                           uint32_t count = DEFAULT_WAIT_COUNT,
                           uint32_t timeout = DEFAULT_TIMEOUT);

 public:
  std::atomic<bool>     isConnected;  ///<
  std::atomic<bool>     isScanning;   ///<
  std::atomic<bool>     isAutoReconnect;  ///<
  std::atomic<bool>     isAutoconnting;  ///<

  enum {
    DEFAULT_TIMEOUT = 2000,    /**< default timeout. */
    DEFAULT_HEART_BEAT = 1000, /**< default heatbeat timeout. */
    MAX_SCAN_NODES = 2048,	   /**< . */
    DEFAULT_TIMEOUT_COUNT = 1,
    DEFAULT_WAIT_COUNT = 10,
    LOCAL_MAX_SCAN_NODES = 128,
  };
  enum {
    YDLIDAR_F4 = 1,
    YDLIDAR_T1 = 2,
    YDLIDAR_F2 = 3,
    YDLIDAR_S4 = 4,
    YDLIDAR_G4 = 5,
    YDLIDAR_X4 = 6,
    YDLIDAR_G4PRO = 7,
    YDLIDAR_F4PRO = 8,
    YDLIDAR_R2_SS_1 = 9,//230400
    YDLIDAR_G10 = 10, //256000
    YDLIDAR_S4B = 11,//153600
    YDLIDAR_S2 = 12,//115200
    YDLIDAR_G6 = 13,//512000
    YDLIDAR_Tail,
  };

  enum {
    YDLIDAR_RATE_4K = 0,
    YDLIDAR_RATE_8K = 1,
    YDLIDAR_RATE_9K = 2,
    YDLIDAR_RATE_10K = 3,
  };

//  node_info      scan_node_buf[2048];  ///<
  node_info      *scan_node_buf;  ///<
  node_info      local_buf[LOCAL_MAX_SCAN_NODES];
  node_info      *local_scan;
  size_t         scan_node_count;      ///<
  Event          _dataEvent;			 ///< data event
  Locker         _lock;				///< thread lock
  Locker         _serial_lock;		///< serial lock
  Thread 	     _thread;				///< thread id
  Locker         _error_lock;		///< error lock

 private:
  int PackageSampleBytes;             ///<
  serial::Serial *_serial;			///< serial
  bool m_intensities;					///< intensity
  uint32_t m_baudrate;				///< serial baudrate
  bool isSupportMotorCtrl;			///<
  uint64_t m_node_time_ns;			///< time stamp
  uint64_t m_node_last_time_ns;       ///< time stamp
  uint32_t m_pointTime;				///< two laser point time intervals
  uint32_t trans_delay;				///< serial transfer on byte time
  int m_sampling_rate;					///< sample rate
  int model; ///< lidar model
  uint8_t m_Maxjor, m_Minjor;

  node_package package;
  node_packages packages;

  uint16_t package_Sample_Index;
  float IntervalSampleAngle;
  float IntervalSampleAngle_LastPackage;
  uint16_t FirstSampleAngle;
  uint16_t LastSampleAngle;
  uint16_t CheckSum;
  uint8_t scan_frequence;

  uint16_t CheckSumCal;
  uint16_t SampleNumlAndCTCal;
  uint16_t LastSampleAngleCal;
  bool CheckSumResult;
  bool LastCheckSumResult;
  uint16_t Valu8Tou16;
  uint8_t *recvBuffer;

  std::string serial_port;///< lidar serial port
  std::vector<float> m_IgnoreArray;//

  int package_index;
  bool data_header_error;
  bool m_SupportMotorDtrCtrl;
  int m_reconnectCount;
  size_t buffer_size;
  size_t m_NoZeroNodeCount;

  DriverError m_driverErrno;       /// number of last error
  uint32_t m_autoTime;
  int sequence;
};

inline bool isNoRibOffsetAngleLidar(int model, uint8_t Maxjor, uint8_t Minjor) {
  bool ret = true;

  if (model == YDlidarDriver::YDLIDAR_R2_SS_1) {
    ret = false;

    if (Maxjor < 1 || (Maxjor == 1 && Minjor <= 7)) {
      ret = true;
    }
  }

  return ret;
}

}

#endif // YDLIDAR_DRIVER_H
