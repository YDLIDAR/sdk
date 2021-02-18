#include "CYdLidar.h"
#include "common.h"
#include "help_info.h"
#include <map>
#include <regex>

using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(nullptr) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 230400;
  m_Intensities       = false;
  m_AutoReconnect     = false;
  m_MaxAngle          = 360.f;
  m_MinAngle          = 0.f;
  m_MaxRange          = 16.0;
  m_MinRange          = 0.08;
  m_SampleRate        = 5;
  m_ScanFrequency     = 7;
  m_AngleOffset       = 0.0;
  m_isAngleOffsetCorrected = false;
  m_isRibOffsetCorrected = false;
  m_GlassNoise        = true;
  m_SunNoise          = true;
  isScanning          = false;
  frequencyOffset     = 0.4;
  m_CalibrationFileName = "";
  m_AbnormalCheckCount  = 6;
  Major               = 0;
  Minjor              = 0;
  m_IgnoreArray.clear();
  ini.SetUnicode();
  m_serial_number.clear();
  nodes = new node_info[YDlidarDriver::MAX_SCAN_NODES];
  m_ParseSuccess      = false;
  m_SingleChannel     = false;
  m_PrintError        = false;
  m_Model             = YDlidarDriver::YDLIDAR_R2_SS_1;
  errTime             = getms();
  laserFailureTime    = getms();
  hasLaserFailure     = false;
  memset(&m_LidarVersion, 0, sizeof(LidarVersion));
  check_thread_running_ = false;
  lidar_data_nomal      = false;
  m_turn_on_time        = getms();
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();

  if (nodes) {
    delete[] nodes;
    nodes = nullptr;
  }
}

void CYdLidar::disconnecting() {
  ScopedLocker l(lidar_lock);

  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = nullptr;
  }

//  {
//    ScopedLocker lck(data_lock_);
//    check_thread_running_ = false;
//    lidar_data_nomal = false;
//  }

  isScanning = false;
//  data_thread_.join();
}

//lidar pointer
YDlidarDriver *CYdLidar::getYdlidarDriver() {
  return lidarPtr;
}

//get zero angle offset value
float CYdLidar::getAngleOffset() const {
  return m_AngleOffset;
}

bool CYdLidar::isAngleOffetCorrected() const {
  return m_isAngleOffsetCorrected;
}

void CYdLidar::GetLidarVersion(LidarVersion &version) {
  memcpy(&version, &m_LidarVersion, sizeof(LidarVersion));
}

result_t CYdLidar::getZeroOffsetAngle(offset_angle &angle, uint32_t timeout) {
  if (!lidarPtr) {
    return RESULT_FAIL;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return RESULT_FAIL;
  }

  return lidarPtr->getZeroOffsetAngle(angle, timeout);
}

result_t CYdLidar::saveRobotOffsetAngle(offset_angle &angle, uint32_t timeout) {
  if (!lidarPtr) {
    return RESULT_FAIL;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return RESULT_FAIL;
  }

  return lidarPtr->saveRobotOffsetAngle(angle, timeout);
}

result_t CYdLidar::getRibOffsetAngle(std::vector<offset_angle> &angle,
                                     uint32_t timeout) {
  if (!lidarPtr) {
    return RESULT_FAIL;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return RESULT_FAIL;
  }

  return lidarPtr->getRibOffsetAngle(angle, timeout);

}

result_t CYdLidar::saveRibOffsetAngle(std::vector<offset_angle> &angle,
                                      uint32_t timeout) {
  if (!lidarPtr) {
    return RESULT_FAIL;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return RESULT_FAIL;
  }

  return lidarPtr->saveRibOffsetAngle(angle, timeout);
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  int ret = checkHardware();

  if (ret != 1) {
    if (ret > 3) {
      hardwareError = true;
    }

    outscan.data.clear();
    return false;
  }

  size_t   count = YDlidarDriver::MAX_SCAN_NODES;

  int sequence;
  //wait Scan data:
  result_t op_result =  lidarPtr->grabScanData(nodes, count, &sequence);

  // Fill in scan data:
  if (IS_OK(op_result)) {
    uint64_t tim_scan_start = nodes[0].stamp;
    uint64_t tim_scan_end   = nodes[count - 1].stamp;
    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;
    LaserScan scan_msg;
    scan_msg.config.sequence = sequence;
    outscan.config.sequence = sequence;
    LaserPoint point;
    LaserDebug debug;
    memset(&debug, 0, sizeof(debug));

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    for (int i = 0; i < count; i++) {
      angle = (float)((nodes[i].angle_q6_checkbit >>
                       LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f) + m_AngleOffset;

      range = (float)nodes[i].distance_q / 1000.f;

      angle = normalize_angle_positive_from_degree(angle);

      uint8_t intensities = (uint8_t)(nodes[i].sync_quality >>
                                      LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
      intensity = (float)intensities;

      if (m_GlassNoise && intensities == GLASSNOISEINTENSITY) {
        intensity = 0.0;
        range     = 0.0;
      }

      if (m_SunNoise && intensities == SUNNOISEINTENSITY) {
        intensity = 0.0;
        range     = 0.0;
      }

      if (range > m_MaxRange || range < m_MinRange) {
        range = 0.0;
      }

      if (angle >= m_MinAngle && angle <= m_MaxAngle) {
        point.angle = angle;
        point.distance = range;
        point.intensity = intensity;
        scan_msg.data.push_back(point);
      }

      if (nodes[i].stamp < tim_scan_start) {
        tim_scan_start = nodes[i].stamp;
      }

      if (nodes[i].stamp > tim_scan_end) {
        tim_scan_end = nodes[i].stamp;
      }

      parsePackageNode(nodes[i], debug);

      if (nodes[i].error_package) {
        debug.MaxDebugIndex = 255;
      }
    }

    double scan_time = (tim_scan_end - tim_scan_start) / 1e9;
    scan_msg.system_time_stamp = tim_scan_start;
    scan_msg.config.min_angle = (m_MinAngle);
    scan_msg.config.max_angle = (m_MaxAngle);

    if (scan_msg.config.max_angle - scan_msg.config.min_angle == 360) {
      scan_msg.config.time_increment = scan_time / (double)count;
    } else {
      scan_msg.config.time_increment = scan_time / (double)(count - 1);
    }

    scan_msg.config.scan_time = scan_time;
    scan_msg.config.min_range = m_MinRange;
    scan_msg.config.max_range = m_MaxRange;
    outscan = scan_msg;

    //parsing version
    if (m_SingleChannel) {
      handleVersionInfoByPackage(debug);
    }

    return true;

  } else {
    if (IS_FAIL(op_result)) {
      // Error? Retry connection
    }
  }

  return false;

}

/*-------------------------------------------------------------
                    handleVersionInfoByPackage
-------------------------------------------------------------*/
void CYdLidar::handleVersionInfoByPackage(const LaserDebug &debug) {
  if (m_ParseSuccess) {
    return;
  }

  device_info info;

  if (ParseLaserDebugInfo(debug, info)) {
    if (printfVersionInfo(info)) {
    }
  }
}

/*-------------------------------------------------------------
            turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  ScopedLocker l(lidar_lock);

  if (!lidarPtr) {
    return false;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

//  if (lidarPtr->isscanning()) {
//    bool thread_running = true;
//    {
//      ScopedLocker lck(data_lock_);
//      thread_running = check_thread_running_;
//    }

//    if (!thread_running) {
//      data_thread_.join();

//      if (!lidar_data_nomal) {
//        lidarPtr->stop();
//        fprintf(stderr,
//                "[CYdLidar][%fs] Failed to turn on the Lidar, because the lidar is [%s].\n",
//                (getms() - m_turn_on_time) / 1000.0,
//                YDlidarDriver::DescribeError(lidarPtr->getSystemError()));
//        isScanning = false;
//      } else {
//        printf("[YDLIDAR INFO][%fs] Now YDLIDAR is scanning ......\n",
//               (getms() - m_turn_on_time) / 1000.0);
//        fflush(stdout);
//      }

//      isScanning = lidar_data_nomal;
//      return lidar_data_nomal;
//    }

//    delay(10);
//    return 2;//processing
//  }

  hasLaserFailure     = false;
  m_turn_on_time = getms();
  lidarPtr->setAutoReconnect(false);
//  {
//    ScopedLocker lck(data_lock_);
//    lidar_data_nomal = false;
//    check_thread_running_ = false;
//  }
//  data_thread_.join();
  // start scan...
  result_t op_result = lidarPtr->startScan(false, 500);

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan(false, 350);

    if (!IS_OK(op_result)) {
      lidarPtr->stop();
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %d\n", op_result);
      isScanning = false;
      return false;
    }
  }

  laserFailureTime = getms();

  if (!checkLidarData()) {
    lidarPtr->stop();
    fprintf(stderr,
            "[CYdLidar][%fs] Failed to turn on the Lidar, because the lidar is [%s].\n",
            (getms() - m_turn_on_time) / 1000.0,
            YDlidarDriver::DescribeError(lidarPtr->getSystemError()));
    isScanning = false;
    return false;
//    lidarPtr->setAutoReconnect(m_AutoReconnect);
//    check_thread_running_ = true;
//    data_thread_ = CLASS_THREAD(CYdLidar, checkLidarAbnormal);

//    if (data_thread_.getHandle() == 0) {
//      lidarPtr->stop();
//      fprintf(stderr, "[CYdLidar] Failed to start check data thread: %x\n",
//              op_result);
//      lidar_data_nomal = false;
//      check_thread_running_ = false;
//      return false;
//    }

//    return 2;//processing
  }

//  if (checkLidarAbnormal()) {
//    lidarPtr->stop();
//    fprintf(stderr,
//            "[CYdLidar][%fs] Failed to turn on the Lidar, because the lidar is [%s].\n",
//            (getms() - start_ts) / 1000.0,
//            YDlidarDriver::DescribeError(lidarPtr->getSystemError()));
//    isScanning = false;
//    return false;
//  }

//  check_thread_running_ = false;
//  lidar_data_nomal = true;
  laserFailureTime = getms();
  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO][%fs] Now YDLIDAR is scanning ......\n",
         (getms() - m_turn_on_time) / 1000.0);
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
            turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff() {
  ScopedLocker l(lidar_lock);

  if (lidarPtr) {
    lidarPtr->stop();
  } else {
    return false;
  }

//  {
//    ScopedLocker lck(data_lock_);
//    check_thread_running_ = false;
//    lidar_data_nomal = false;
//  }

  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
  }

  isScanning = false;
  hasLaserFailure     = false;
//  data_thread_.join();
  return true;
}

bool CYdLidar::checkLidarData() {
  result_t op_result = RESULT_FAIL;
  YDlidarDriver::DriverError err = YDlidarDriver::NoError;
//  uint32_t startTime = getms();
  int m_generatedData = 0;

  while ((getms() - m_turn_on_time) < 2900 && lidarPtr->isscanning()) {
    size_t   count = YDlidarDriver::MAX_SCAN_NODES;
    int seq = 0;
    op_result =  lidarPtr->grabScanData(nodes, count, &seq, 460);
    err = lidarPtr->getSystemError();

    if (IS_OK(op_result)) {
      if (err != YDlidarDriver::LaserFailureError) {
        m_generatedData++;

        if (m_generatedData > 1) {
          return true;
        }

        continue;
      } else {
        m_generatedData = 0;
        fprintf(stderr, "[CYdLidar][ERROR][%fs]: %s\n",
                (getms() - laserFailureTime) / 1000.0, YDlidarDriver::DescribeError(err));
        fflush(stderr);

        if ((getms() - laserFailureTime) > 3 * 1000) {
          op_result = RESULT_FAIL;
          break;
        }

        op_result = RESULT_FAIL;
        continue;
      }
    }

    m_generatedData = 0;
  }

  if (!IS_OK(op_result) && err != YDlidarDriver::NoError) {
    fprintf(stderr, "[CYdLidar][WARNNING][%fs]: %s\n",
            (getms() - laserFailureTime) / 1000.0, YDlidarDriver::DescribeError(err));
    fflush(stderr);
  }

  return IS_OK(op_result);
}

int CYdLidar::checkLidarAbnormal() {
  int check_abnormal_count = 0;
  result_t op_result = RESULT_FAIL;
  YDlidarDriver::DriverError err = YDlidarDriver::NoError;
  int m_generatedData = 0;
  uint32_t startTime = getms();

  while ((getms() - startTime) < 30000 && lidarPtr->isscanning()) {
    {
      ScopedLocker lck(data_lock_);

      if (!check_thread_running_) {
        lidar_data_nomal = false;
        return RESULT_FAIL;
      }
    }

    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0 && !IS_OK(op_result)) {
      delay(100);

      if (!lidarPtr->isscanning()) {
        ScopedLocker lck(data_lock_);
        check_thread_running_ = false;
        lidar_data_nomal = false;
        return RESULT_FAIL;
      }

//      delay(check_abnormal_count * 1190);
    }


    size_t   count = YDlidarDriver::MAX_SCAN_NODES;
    int seq;
    op_result =  lidarPtr->grabScanData(nodes, count, &seq, 500);
    err = lidarPtr->getSystemError();

    if (IS_OK(op_result)) {
      if (err != YDlidarDriver::LaserFailureError) {
        m_generatedData++;

        if (m_generatedData > 1) {
          ScopedLocker lck(data_lock_);
          check_thread_running_ = false;
          lidar_data_nomal = true;
          return RESULT_OK;
        }

        continue;
      } else {
        m_generatedData = 0;
        fprintf(stderr, "[CYdLidar][ERROR][%fs]: %s\n",
                (getms() - laserFailureTime) / 1000.0, YDlidarDriver::DescribeError(err));
        fflush(stderr);

        if (getms() - laserFailureTime > 29 * 1000) {
          op_result = RESULT_FAIL;
          break;
        }

        continue;
      }
    } else {
      if (check_abnormal_count % 3 == 0 && err != YDlidarDriver::NoError) {
        fprintf(stderr, "[CYdLidar][WARNNING][%fs]: %s\n",
                (getms() - laserFailureTime) / 1000.0, YDlidarDriver::DescribeError(err));
        fflush(stderr);
      }
    }

    m_generatedData = 0;
    check_abnormal_count++;
  }

  if (err == YDlidarDriver::LaserFailureError) {
    op_result = RESULT_FAIL;
  }

  {
    ScopedLocker lck(data_lock_);
    check_thread_running_ = false;
    lidar_data_nomal = false;

    if (IS_OK(op_result)) {
      lidar_data_nomal = true;
    }
  }

  return op_result;
}

/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth() {
  if (!lidarPtr) {
    return false;
  }

  lidarPtr->stop();
  result_t op_result;
  device_health healthinfo;
  printf("[YDLIDAR]:SDK Version: %s\n", YDlidarDriver::getSDKVersion().c_str());
  op_result = lidarPtr->getHealth(healthinfo, 500);

  if (IS_OK(op_result)) {
    printf("[YDLIDAR]:Lidar running correctly ! The health status: %s\n",
           (int)healthinfo.status == 0 ? "good" : "bad");

    if (healthinfo.status == 2) {
      fprintf(stderr,
              "Error, Yd Lidar internal error detected. Please reboot the device to retry.\n");
      return false;
    } else {
      return true;
    }

  } else {
    if (m_PrintError) {
      fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
    }

    return false;
  }

}

bool CYdLidar::getDeviceInfo() {
  if (!lidarPtr) {
    return false;
  }

  bool ret = false;
  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo, 500);

  if (!IS_OK(op_result)) {
    if (m_PrintError) {
      fprintf(stderr, "get Device Information Error\n");
    }

    return false;
  }

  if (devinfo.model != YDlidarDriver::YDLIDAR_R2_SS_1 &&
      devinfo.model != YDlidarDriver::YDLIDAR_G4 &&
      devinfo.model != YDlidarDriver::YDLIDAR_G4PRO &&
      devinfo.model != YDlidarDriver::YDLIDAR_S4) {
    printf("[YDLIDAR INFO] Current SDK does not support current lidar models[%d]\n",
           devinfo.model);
    return false;
  }

  ret = true;
  m_Model = devinfo.model;
  m_LidarVersion.hardware = devinfo.hardware_version;
  m_LidarVersion.firmware = uint8_t((uint8_t)(devinfo.firmware_version >> 8) * 100
                                    + (uint8_t)(devinfo.firmware_version & 0xff));
  std::string model = "R2-SS-1";
  int m_samp_rate = 5;

  switch (devinfo.model) {
    case YDlidarDriver::YDLIDAR_G4:
      model = "G4";
      break;

    case YDlidarDriver::YDLIDAR_R2_SS_1:
      model = "R2-SS-1";
      m_samp_rate = 5;
      m_SampleRate = m_samp_rate;
      break;

    case YDlidarDriver::YDLIDAR_S4:
    case YDlidarDriver::YDLIDAR_S2:
      model = "S2";
      m_samp_rate = 3;
      m_SampleRate = m_samp_rate;
      break;

    default:
      break;
  }

  if (printfVersionInfo(devinfo)) {
    std::regex
    rx("^2(\\d{3})(0\\d{1}|1[0-2])(0\\d{1}|[12]\\d{1}|3[01])(\\d{4})(\\d{4})$");
    std::smatch result;

    if (!regex_match(m_serial_number, result, rx)) {
      fprintf(stderr, "Invalid lidar serial number!!!\n");
      return false;
    }

    memcpy(&m_LidarVersion.sn[0], &devinfo.serialnum[0], 16);

    if (devinfo.model == YDlidarDriver::YDLIDAR_R2_SS_1) {
      ret &= checkCalibrationAngle(m_serial_number);
//      checkRibOffsetAngle();
    } else {
      m_isAngleOffsetCorrected = true;
      m_isRibOffsetCorrected = true;

      if (devinfo.model != YDlidarDriver::YDLIDAR_S4 &&
          devinfo.model != YDlidarDriver::YDLIDAR_S2) {
        checkSampleRate();
      }
    }

    if (devinfo.model != YDlidarDriver::YDLIDAR_S4 &&
        devinfo.model != YDlidarDriver::YDLIDAR_S2) {
      checkScanFrequency();
    }
  }

  printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n", m_SampleRate);
  fflush(stdout);
  return ret;
}

bool CYdLidar::printfVersionInfo(const device_info &info) {
  if (info.firmware_version == 0 &&
      info.hardware_version == 0) {
    return false;
  }

  std::string model = "R2-SS-1";

  switch (info.model) {
    case YDlidarDriver::YDLIDAR_G4:
      model = "G4";
      break;

    case YDlidarDriver::YDLIDAR_R2_SS_1:
      model = "R2-SS-1";
      break;

    default:
      break;
  }


  m_ParseSuccess = true;
  Major = (uint8_t)(info.firmware_version >> 8);
  Minjor = (uint8_t)(info.firmware_version & 0xff);
  printf("[YDLIDAR] Connection established in [%s][%d]:\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n"
         "Serial: ",
         m_SerialPort.c_str(),
         m_SerialBaudrate,
         Major,
         Minjor,
         (unsigned int)info.hardware_version,
         model.c_str());
  std::string serial_number;

  for (int i = 0; i < 16; i++) {
    serial_number += format("%01X", info.serialnum[i] & 0xff);
    printf("%01X", info.serialnum[i] & 0xff);
  }

  m_serial_number = serial_number;
  printf("\n");
  return true;
}

void CYdLidar::checkSampleRate() {
  sampling_rate _rate;
  int _samp_rate = 9;
  int try_count;
  result_t ans = lidarPtr->getSamplingRate(_rate);

  if (IS_OK(ans)) {
    switch (m_SampleRate) {
      case 4:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_4K;
        break;

      case 8:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_8K;
        break;

      case 9:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_9K;
        break;

      default:
        _samp_rate = _rate.rate;
        break;
    }

    while (_samp_rate != _rate.rate) {
      ans = lidarPtr->setSamplingRate(_rate);

      if (!IS_OK(ans)) {
        try_count++;

        if (try_count > 3) {
          break;
        }
      }
    }

    switch (_rate.rate) {
      case YDlidarDriver::YDLIDAR_RATE_4K:
        _samp_rate = 4;
        break;

      case YDlidarDriver::YDLIDAR_RATE_8K:
        _samp_rate = 8;
        break;

      case YDlidarDriver::YDLIDAR_RATE_9K:
        _samp_rate = 9;
        break;

      default:
        break;
    }
  }

  m_SampleRate = _samp_rate;

}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency() {
  float frequency = 7.4f;
  scan_frequency _scan_frequency;
  float hz = 0;
  result_t ans = RESULT_FAIL;
  m_ScanFrequency += frequencyOffset;

  if (5.0 - frequencyOffset <= m_ScanFrequency &&
      m_ScanFrequency <= 12 + frequencyOffset) {
    ans = lidarPtr->getScanFrequency(_scan_frequency) ;

    if (IS_OK(ans)) {
      frequency = _scan_frequency.frequency / 100.f;
      hz = m_ScanFrequency - frequency;

      if (hz > 0) {
        while (hz > 0.95) {
          lidarPtr->setScanFrequencyAdd(_scan_frequency);
          hz = hz - 1.0;
        }

        while (hz > 0.09) {
          lidarPtr->setScanFrequencyAddMic(_scan_frequency);
          hz = hz - 0.1;
        }

        frequency = _scan_frequency.frequency / 100.0f;
      } else {
        while (hz < -0.95) {
          lidarPtr->setScanFrequencyDis(_scan_frequency);
          hz = hz + 1.0;
        }

        while (hz < -0.09) {
          lidarPtr->setScanFrequencyDisMic(_scan_frequency);
          hz = hz + 0.1;
        }

        frequency = _scan_frequency.frequency / 100.0f;
      }
    }
  } else {
    fprintf(stderr, "current scan frequency[%f] is out of range.",
            m_ScanFrequency - frequencyOffset);
  }

  ans = lidarPtr->getScanFrequency(_scan_frequency);

  if (IS_OK(ans)) {
    frequency = _scan_frequency.frequency / 100.0f;
    m_ScanFrequency = frequency;
  }

  m_ScanFrequency -= frequencyOffset;
  printf("[YDLIDAR INFO] Current Scan Frequency: %fHz\n", m_ScanFrequency);
  return true;
}

/*-------------------------------------------------------------
                        checkCalibrationAngle
-------------------------------------------------------------*/
bool CYdLidar::checkCalibrationAngle(const std::string &serialNumber) {
  bool ret = false;
  m_AngleOffset = 0.0;
  result_t ans;
  offset_angle angle;
  int retry = 0;
  m_isAngleOffsetCorrected = false;

  while (retry < 2 && (Major > 1 || (Major >= 1 && Minjor > 1))) {
    ans = lidarPtr->getLidarZeroOffsetAngle(angle);

    if (IS_OK(ans)) {
      if (angle.angle > 36000 || angle.angle < -36000) {
        ans = lidarPtr->getLidarZeroOffsetAngle(angle);

        if (!IS_OK(ans)) {
          retry++;
          continue;
        }
      }

      if (isNoRibOffsetAngleLidar(m_Model, Major, Minjor)) {
        m_isAngleOffsetCorrected = (angle.angle != 720);
        m_AngleOffset = angle.angle / 4.0;
      } else {
        m_isAngleOffsetCorrected = (angle.angle != 18000);
        m_AngleOffset = angle.angle / 100.0;
      }

      printf("[YDLIDAR INFO] Successfully obtained the %s offset angle[%f] from the lidar[%s]\n"
             , m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset,
             serialNumber.c_str());
      ret = true;
      return ret;
    }

    retry++;
  }

  if (ydlidar::fileExists(m_CalibrationFileName)) {
    SI_Error rc = ini.LoadFile(m_CalibrationFileName.c_str());

    if (rc >= 0) {
      m_isAngleOffsetCorrected = true;
      double default_value = 179.6;
      m_AngleOffset = ini.GetDoubleValue("CALIBRATION", serialNumber.c_str(),
                                         default_value);

      if (fabs(m_AngleOffset - default_value) < 0.01) {
        m_isAngleOffsetCorrected = false;
        m_AngleOffset = 0.0;
      }

      printf("[YDLIDAR INFO] Successfully obtained the %s offset angle[%f] from the calibration file[%s]\n"
             , m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset,
             m_CalibrationFileName.c_str());

    } else {
      printf("[YDLIDAR INFO] Failed to open calibration file[%s]\n",
             m_CalibrationFileName.c_str());
    }
  } else {
    printf("[YDLIDAR INFO] Calibration file[%s] does not exist\n",
           m_CalibrationFileName.c_str());
  }

  printf("[YDLIDAR INFO] Current %s AngleOffset : %f°\n",
         m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset);
  return ret;
}

void CYdLidar::checkRibOffsetAngle() {
  m_isRibOffsetCorrected = false;

  if (!isNoRibOffsetAngleLidar(m_Model, Major, Minjor)) {
    std::vector<offset_angle> rib_offset;
    result_t ans = lidarPtr->getRibOffsetAngle(rib_offset);

    if (IS_OK(ans)) {
      m_isRibOffsetCorrected = true;
    }
  }
}

/*-------------------------------------------------------------
            checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  ScopedLocker l(lidar_lock);

  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new YDlidarDriver();

    if (!lidarPtr) {
      fprintf(stderr, "Create Driver fail\n");
      return false;
    }
  }

  if (lidarPtr->isconnected()) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3) {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm') {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4') {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  uint32_t startTs = getms();
  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result)) {
    fprintf(stderr,
            "[CYdLidar][%dms] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
            getms() - startTs,
            m_SerialPort.c_str(), m_SerialBaudrate);
    return false;
  }

  lidarPtr->setIntensities(m_Intensities);
  lidarPtr->setIgnoreArray(m_IgnoreArray);
  lidarPtr->setSingleChannel(m_SingleChannel);
  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus() {
  if (m_ParseSuccess) {
    return true;
  }

  if (!checkCOMMs()) {
    return false;
  }

  bool ret = getDeviceHealth();

  if (!ret) {
    delay(100);
  }

  if (!getDeviceInfo()) {
    delay(100);
    ret = getDeviceInfo();

    if (!ret) {
      return false;
    }
  }

  return true;
}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
int CYdLidar::checkHardware() {
  ScopedLocker l(lidar_lock);
  int ret = 0;

  if (!lidarPtr) {
    return ret;
  }

  if (isScanning && lidarPtr->isscanning()) {
    YDlidarDriver::DriverError err = lidarPtr->getSystemError();

    if (err == YDlidarDriver::NoError) {
      hasLaserFailure = false;
      return 1;
    } else  if (err == YDlidarDriver::TimeoutError) {
      hasLaserFailure = false;
      ret = 2;
    } else if (err == YDlidarDriver::LaserFailureError) {
      if (!hasLaserFailure) {
        laserFailureTime = getms();
      }

      ret = 3;
      hasLaserFailure = true;

      if ((getms() - laserFailureTime) > 29 * 1000) {
        ret = 4;
      }
    } else {
      hasLaserFailure = false;
      ret = 4;
    }

    if ((getms() - errTime) > YDlidarDriver::DEFAULT_TIMEOUT / 20) {
      fprintf(stderr, "[YDLIDAR][ERROR][%fs]: %s\n", (getms() - errTime) / 1000.0,
              YDlidarDriver::DescribeError(err));
      fflush(stderr);
      errTime = getms();
    }
  }

  return ret;
}

bool CYdLidar::checkLidarModel() {
  m_SerialBaudrate = 230400;

  if (!checkCOMMs()) {
    fprintf(stderr,
            "[CYdLidar::checkLidarModel] Error initializing YDLIDAR check Lidar Model.\n");
    fflush(stderr);
    return false;
  }

  bool ret = getDeviceHealth();

  if (!ret) {
    delay(200);
  }

  if (!getDeviceInfo() && !ret) {
    m_SerialBaudrate = 115200;
    m_SingleChannel = true;

    disconnecting();
    delay(20);
  }

  m_PrintError = true;
  return true;
}

/*-------------------------------------------------------------
            initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize() {

  if (!checkCOMMs()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check Comms.\n");
    fflush(stderr);
    return false;
  }

  uint32_t startTs = getms();

  if (!checkStatus()) {
    fprintf(stderr,
            "[CYdLidar::initialize][%dms] Error initializing YDLIDAR check status.\n",
            getms() - startTs);
    fflush(stderr);
    return false;
  }

  return true;
}
