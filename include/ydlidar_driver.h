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
  *     230400 G2-SS-1 R2-SS
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
  * @brief 获取激光数据 \n
  * @param[in] nodebuffer 激光点信息
  * @param[in] count      一圈激光点数
  * @param[in] timeout    超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE    获取失败
  * @note 获取之前，必须使用::startScan函数开启扫描
  */
  result_t grabScanData(node_info *nodebuffer, size_t &count,
                        uint32_t timeout = DEFAULT_TIMEOUT) ;


  /**
  * @brief 补偿激光角度 \n
  * 把角度限制在0到360度之间
  * @param[in] nodebuffer 激光点信息
  * @param[in] count      一圈激光点数
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 补偿之前，必须使用::grabScanData函数获取激光数据成功
  */
  result_t ascendScanData(node_info *nodebuffer, size_t count);

  /**
  * @brief 重置激光雷达 \n
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作, 如果在扫描中调用::stop函数停止扫描
  */
  result_t reset(uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 获取激光雷达当前扫描频率 \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t getScanFrequency(scan_frequency &frequency,
                            uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置增加扫描频率1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyAdd(scan_frequency &frequency,
                               uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置减小扫描频率1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyDis(scan_frequency &frequency,
                               uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置增加扫描频率0.1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyAddMic(scan_frequency &frequency,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置减小扫描频率0.1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyDisMic(scan_frequency &frequency,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 获取激光雷达当前采样频率 \n
  * @param[in] frequency    采样频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t getSamplingRate(sampling_rate &rate,
                           uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置激光雷达当前采样频率 \n
  * @param[in] rate    　　　采样频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setSamplingRate(sampling_rate &rate,
                           uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 获取激光雷达当前零位角 \n
  * @param[in] angle　　　   零位偏移角
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t getZeroOffsetAngle(offset_angle &angle,
                              uint32_t timeout = DEFAULT_TIMEOUT);

 protected:

  /**
  * @brief 创建解析雷达数据线程 \n
  * @note 创建解析雷达数据线程之前，必须使用::startScan函数开启扫图成功
  */
  result_t createThread();


  /**
  * @brief 重新连接开启扫描 \n
  * @param[in] force    扫描模式
  * @param[in] timeout  超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       开启成功
  * @retval RESULT_FAILE    开启失败
  * @note sdk 自动重新连接调用
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
  * @brief 等待激光数据包头 \n
  * @param[in] header 	 包头
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    获取失败
  * @note 当timeout = -1 时, 将一直等待
  */
  result_t waitResponseHeader(lidar_ans_header *header,
                              uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 等待固定数量串口数据 \n
  * @param[in] data_count 	 等待数据大小
  * @param[in] timeout    	 等待时间
  * @param[in] returned_size   实际数据大小
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    获取失败
  * @note 当timeout = -1 时, 将一直等待
  */
  result_t waitForData(size_t data_count, uint32_t timeout = DEFAULT_TIMEOUT,
                       size_t *returned_size = NULL);

  /**
  * @brief 获取串口数据 \n
  * @param[in] data 	 数据指针
  * @param[in] size    数据大小
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE    获取失败
  */
  result_t getData(uint8_t *data, size_t size);

  /**
  * @brief 串口发送数据 \n
  * @param[in] data 	 发送数据指针
  * @param[in] size    数据大小
  * @return 返回执行结果
  * @retval RESULT_OK       发送成功
  * @retval RESULT_FAILE    发送失败
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

  /**
   * @brief checkAutoConnecting
   */
  result_t checkAutoConnecting();

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
    YDLIDAR_G2_SS_1 = 9,//230400
    YDLIDAR_G10 = 10, //256000
    YDLIDAR_S4B = 11,//153600
    YDLIDAR_S2 = 12,//115200
    YDLIDAR_G25 = 13,//512000
    YDLIDAR_Tail,
  };

  enum {
    YDLIDAR_RATE_4K = 0,
    YDLIDAR_RATE_8K = 1,
    YDLIDAR_RATE_9K = 2,
    YDLIDAR_RATE_10K = 3,
  };


  node_info      scan_node_buf[2048];  ///<
  size_t         scan_node_count;      ///<
  Event          _dataEvent;			 ///< data event
  Locker         _lock;				///< thread lock
  Locker         _serial_lock;		///< serial lock
  Thread 	       _thread;				///< thread id

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

  std::string serial_port;///< lidar serial port
  std::vector<float> m_IgnoreArray;//

};
}

#endif // YDLIDAR_DRIVER_H
