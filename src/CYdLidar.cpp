#include "CYdLidar.h"
#include "common.h"
#include <map>

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
  m_GlassNoise        = true;
  m_SunNoise          = true;
  isScanning          = false;
  frequencyOffset     = 0.4;
  m_AbnormalCheckCount  = 2;
  m_Reversion         = true;
  m_OffsetTime        = 0.0;
  Major               = 0;
  Minjor              = 0;
  m_pointTime         = 1e9 / 5000;
  last_node_time      = getTime();
  m_FixedSize         = 500;
  m_IgnoreArray.clear();
  m_serial_number.clear();
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();
}

void CYdLidar::disconnecting() {
  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = nullptr;
  }

  isScanning = false;
}

int CYdLidar::getFixedSize() const {
  return m_FixedSize;
}

//get zero angle offset value
float CYdLidar::getAngleOffset() const {
  return m_AngleOffset;
}

bool CYdLidar::isAngleOffetCorrected() const {
  return m_isAngleOffsetCorrected;
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &scan_msg, bool &hardwareError) {
  hardwareError			= false;
  scan_msg.data.clear();

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    delay(1000 / m_ScanFrequency);
    return false;
  }

  node_info nodes[2048];
  size_t   count = _countof(nodes);


  //wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(nodes, count);
  uint64_t tim_scan_end   = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;
    LaserPoint point;

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    uint64_t scan_time = m_pointTime * (count - 1);
    tim_scan_end += m_OffsetTime * 1e9;
    tim_scan_end -= m_pointTime;
    tim_scan_start = tim_scan_end -  scan_time ;

    if (tim_scan_start < last_node_time) {
      tim_scan_start = last_node_time;
      tim_scan_end = tim_scan_start + scan_time;
    }

    last_node_time = tim_scan_end;
    scan_msg.config.scan_time = 1.0 * scan_time / 1e9;

    scan_msg.config.min_angle = angles::from_degrees(m_MinAngle);
    scan_msg.config.max_angle = angles::from_degrees(m_MaxAngle);
    scan_msg.config.time_increment = scan_msg.config.scan_time / (double)count;
    scan_msg.system_time_stamp = tim_scan_start;
    scan_msg.config.min_range = m_MinRange;
    scan_msg.config.max_range = m_MaxRange;
    int min_index = count;


    for (int i = 0; i < count; i++) {
      angle = (float)((nodes[i].angle_q6_checkbit >>
                       LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f) + m_AngleOffset;
      range = (float)nodes[i].distance_q / 1000.f;

      angle = angles::from_degrees(angle);

      if (m_Reversion) {
        angle += M_PI;
      }

      angle = 2 * M_PI - angle;

      angle = angles::normalize_angle(angle);

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

      if (angle >= scan_msg.config.min_angle && angle <= scan_msg.config.max_angle) {
        if (i < min_index) {
          min_index = i;
        }

        point.angle = angle;
        point.range = range;
        point.intensity = intensity;
        scan_msg.data.push_back(point);
      }

    }

    scan_msg.system_time_stamp = tim_scan_start + min_index * m_pointTime;
    return true;

  } else {
    if (IS_FAIL(op_result)) {
      // Error? Retry connection
    }
  }

  return false;

}


/*-------------------------------------------------------------
            turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  if (!lidarPtr) {
    return false;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
      isScanning = false;
      return false;
    }
  }

  if (checkLidarAbnormal()) {
    lidarPtr->stop();
    fprintf(stderr,
            "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
    isScanning = false;
    return false;
  }

  isScanning = true;
  m_pointTime = lidarPtr->getPointTime();
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
            turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff() {
  if (lidarPtr) {
    lidarPtr->stop();
  } else {
    return false;
  }

  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
  }

  isScanning = false;
  return true;
}

bool CYdLidar::checkLidarAbnormal() {
  node_info nodes[2048];
  size_t   count = _countof(nodes);
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;

  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0) {
      delay(check_abnormal_count * 1000);
    }

    op_result =  lidarPtr->grabScanData(nodes, count);

    if (IS_OK(op_result)) {
      return false;
    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
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
  op_result = lidarPtr->getHealth(healthinfo);

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
    fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
    return false;
  }

}

bool CYdLidar::getDeviceInfo() {
  if (!lidarPtr) {
    return false;
  }

  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo);

  if (!IS_OK(op_result)) {
    fprintf(stderr, "get Device Information Error\n");
    return false;
  }

  if (devinfo.model != YDlidarDriver::YDLIDAR_R2 &&
      devinfo.model != YDlidarDriver::YDLIDAR_G4) {
    printf("[YDLIDAR INFO] Current SDK does not support current lidar models[%d]\n",
           devinfo.model);
    return false;
  }

  std::string model = "R2";
  int m_samp_rate = 5;

  switch (devinfo.model) {
    case YDlidarDriver::YDLIDAR_G4:
      model = "G4";
      break;

    case YDlidarDriver::YDLIDAR_R2:
      model = "R2";
      m_samp_rate = 5;
      m_SampleRate = m_samp_rate;
      break;

    default:
      break;
  }

  Major = (uint8_t)(devinfo.firmware_version >> 8);
  Minjor = (uint8_t)(devinfo.firmware_version & 0xff);
  std::string serial_number;
  printf("[YDLIDAR] Connection established in [%s][%d]:\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n"
         "Serial: ",
         m_SerialPort.c_str(),
         m_SerialBaudrate,
         Major,
         Minjor,
         (unsigned int)devinfo.hardware_version,
         model.c_str());

  for (int i = 0; i < 16; i++) {
    serial_number += format("%01X", devinfo.serialnum[i] & 0xff);
  }

  printf("%s\n", serial_number.c_str());
  m_serial_number = serial_number;

  if (devinfo.model == YDlidarDriver::YDLIDAR_R2) {
    checkCalibrationAngle(serial_number);
  } else {
    m_isAngleOffsetCorrected = true;
    checkSampleRate();
  }

  printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n", m_SampleRate);
  checkScanFrequency();
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
  m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
  printf("[YDLIDAR INFO] Current Scan Frequency: %fHz\n", m_ScanFrequency);
  return true;
}

/*-------------------------------------------------------------
                        checkCalibrationAngle
-------------------------------------------------------------*/
void CYdLidar::checkCalibrationAngle(const std::string &serialNumber) {
  m_AngleOffset = 0.0;
  result_t ans;
  offset_angle angle;
  int retry = 0;
  m_isAngleOffsetCorrected = false;

  while (retry < 2) {
    ans = lidarPtr->getZeroOffsetAngle(angle);

    if (IS_OK(ans)) {
      m_isAngleOffsetCorrected = (angle.angle != 720);
      m_AngleOffset = angle.angle / 4.0;
      printf("[YDLIDAR INFO] Successfully obtained the %s offset angle[%f] from the lidar[%s]\n"
             , m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset,
             serialNumber.c_str());
      return;
    }

    retry++;
  }

  fprintf(stderr, "[YDLIDAR ERROR] Failed to obtained AngleOffset\n");
}

/*-------------------------------------------------------------
            checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
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

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result)) {
    fprintf(stderr,
            "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
            m_SerialPort.c_str(), m_SerialBaudrate);
    return false;
  }

  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus() {

  if (!checkCOMMs()) {
    return false;
  }

  bool ret = getDeviceHealth();

  if (!ret) {
    delay(1000);
  }

  if (!getDeviceInfo()) {
    delay(2000);
    ret = getDeviceInfo();

    if (!ret) {
      return false;
    }
  }

  lidarPtr->setIntensities(m_Intensities);
  lidarPtr->setIgnoreArray(m_IgnoreArray);
  return true;
}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  if (!lidarPtr) {
    return false;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  return false;
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

  if (!checkStatus()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check status.\n");
    fflush(stderr);
    return false;
  }

  return true;
}
