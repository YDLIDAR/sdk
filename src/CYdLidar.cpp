#include "CYdLidar.h"
#include "common.h"
#include <map>
#include <angles.h>
#include <numeric>

using namespace std;
using namespace ydlidar;
using namespace impl;
using namespace angles;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(nullptr) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 115200;
  m_FixedResolution   = true;
  m_Reversion         = false;
  m_AutoReconnect     = true;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 12.0;
  m_MinRange          = 0.08;
  m_ScanFrequency     = 8;
  isScanning          = false;
  m_AbnormalCheckCount  = 4;
  m_Intensity         = false;
  Major               = 0;
  Minjor              = 0;
  m_IgnoreArray.clear();
  m_pointTime         = 1e9 / 3000;
  last_node_time      = getTime();
  m_FixedSize         = 360;
  m_SampleRate        = 3;
  print               = false;
  m_GlassNoise        = true;
  m_SunNoise          = true;
  global_nodes = new node_info[YDlidarDriver::MAX_SCAN_NODES];
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();

  if (global_nodes) {
    delete[] global_nodes;
    global_nodes = NULL;
  }
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

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &scan_msg, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    delay(400 / m_ScanFrequency);
    return false;
  }

  size_t   count = YDlidarDriver::MAX_SCAN_NODES;

  //wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(global_nodes, count);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0f;
    LaserPoint point;

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    uint64_t scan_time = m_pointTime * (count - 1);
    tim_scan_end -= m_pointTime;
    tim_scan_end -= global_nodes[0].dstamp;
    tim_scan_start = tim_scan_end -  scan_time ;

    int64_t timeDiff = tim_scan_start - last_node_time;

    if (timeDiff < 0 && last_node_time < tim_scan_end) {
      tim_scan_start = last_node_time + m_pointTime;
      tim_scan_end = tim_scan_start + scan_time;
    }

    last_node_time = tim_scan_end;
    scan_msg.config.scan_time = 1.0 * scan_time / 1e9;

    scan_msg.config.min_angle = angles::from_degrees(m_MinAngle);
    scan_msg.config.max_angle = angles::from_degrees(m_MaxAngle);
    scan_msg.config.time_increment = scan_msg.config.scan_time / (double)(
                                       count - 1);
    scan_msg.system_time_stamp = tim_scan_start;
    scan_msg.config.min_range = m_MinRange;
    scan_msg.config.max_range = m_MaxRange;

    for (int i = 0; i < count; i++) {
      angle = (float)((global_nodes[i].angle_q6_checkbit >>
                       LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
      range = (float)global_nodes[i].distance_q2 / 4.0f / 1000.f;

      angle = angles::from_degrees(angle);

      if (m_Reversion) {
        angle += M_PI;
      }

      angle = 2 * M_PI - angle;

      angle = angles::normalize_angle(angle);

      uint8_t intensities = (uint8_t)(global_nodes[i].sync_quality >>
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
        point.angle = angle;
        point.range = range;
        point.intensity = intensity;
        scan_msg.data.push_back(point);
      }

    }

//    scan_msg.system_time_stamp = tim_scan_start + min_index * m_pointTime;
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
  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  lidarPtr->setIntensities(m_Intensity);
  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
      lidarPtr->stop();
      isScanning = false;
      return false;
    }
  }

  m_pointTime = lidarPtr->getPointTime();

  if (checkLidarAbnormal()) {
    lidarPtr->stop();
    fprintf(stderr,
            "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
    isScanning = false;
    return false;
  }

  isScanning = true;
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
  }

  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
  }

  isScanning = false;
  return true;
}

bool CYdLidar::checkLidarAbnormal() {

  size_t   count = YDlidarDriver::MAX_SCAN_NODES;
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;
  std::vector<int> data;
  int buffer_count  = 0;

  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0) {
      delay(check_abnormal_count * 1000);
    }

    float scan_time = 0.0;
    uint64_t start_time = 0;
    uint64_t end_time = 0;
    op_result = RESULT_OK;

    while (buffer_count < 10 && scan_time < 0.05 && IS_OK(op_result)) {
      start_time = getTime();
      count = YDlidarDriver::MAX_SCAN_NODES;
      op_result =  lidarPtr->grabScanData(global_nodes, count);
      end_time = getTime();
      scan_time = 1.0 * static_cast<int64_t>(end_time - start_time) / 1e9;
      buffer_count++;
    }

    if (IS_OK(op_result)) {
      data.push_back(count);
      int collection = 0;

      while (collection < 5) {
        count = YDlidarDriver::MAX_SCAN_NODES;
        start_time = getTime();
        op_result =  lidarPtr->grabScanData(global_nodes, count);
        end_time = getTime();

        if (IS_OK(op_result)) {
          if (abs(static_cast<int>(data.front() - count)) > 20) {
            data.erase(data.begin());
          }

          scan_time = 1.0 * static_cast<int64_t>(end_time - start_time) / 1e9;

          if (scan_time > 0.05 && scan_time < 0.3) {
            m_SampleRate = static_cast<int>((count / scan_time + 500) / 1000);
            int samplerate = m_SampleRate;

            if (global_nodes[0].scan_frequence > 0) {
              samplerate = (global_nodes[0].scan_frequence / 10.0 * count + 500) / 1000;
            }

            if (samplerate == m_SampleRate) {
              m_pointTime = 1e9 / (m_SampleRate * 1000);
              lidarPtr->updatePointTime(m_pointTime);
            }
          }

          data.push_back(count);
        }

        collection++;
      }

      if (data.size() > 1) {
        int total = accumulate(data.begin(), data.end(), 0);
        int mean =  total / data.size(); //均值
        m_FixedSize = (static_cast<int>((mean + 5) / 10)) * 10;
        printf("[YDLIDAR]:Fixed Size: %d\n", m_FixedSize);
        printf("[YDLIDAR]:Sample Rate: %dK\n", m_SampleRate);
        return false;

      }

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
    if (print) {
      fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n",
              op_result);
    }

    return false;
  }

}

bool CYdLidar::getDeviceInfo() {
  if (!lidarPtr) {
    return false;
  }

  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo, 500);

  if (!IS_OK(op_result)) {
    if (print) {
      fprintf(stderr, "get Device Information Error\n");
    }

    return false;
  }

  if (devinfo.model != YDlidarDriver::YDLIDAR_S4 &&
      devinfo.model != YDlidarDriver::YDLIDAR_G4 &&
      !lidarPtr->isSingleChannel()) {
    printf("[YDLIDAR INFO] Current SDK does not support current lidar models[%d]\n",
           devinfo.model);
    return false;
  }

  std::string model = "S4";

  switch (devinfo.model) {
    case YDlidarDriver::YDLIDAR_G4:
      model = "S4";
      break;

    default:
      break;
  }

  if (lidarPtr->isSingleChannel()) {
    model = "S2K";
    printf("[YDLIDAR] Connection established in [%s][%d]:\n",
           m_SerialPort.c_str(),
           m_SerialBaudrate);
    m_FixedSize         = 360;
  } else {
    Major = (uint8_t)(devinfo.firmware_version >> 8);
    Minjor = (uint8_t)(devinfo.firmware_version & 0xff);
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
      printf("%01X", devinfo.serialnum[i] & 0xff);
    }
  }

  printf("\n");
  return true;
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

  print = false;
  bool ret = getDeviceHealth();

  if (!ret) {
    delay(500);
    print = true;
    ret = getDeviceHealth();

    if (!ret) {
      delay(500);
    }
  }

  print = false;

  if (!getDeviceInfo()) {
    delay(1000);
    print = true;
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

  if (!turnOn()) {
    return false;
  }

  return true;
}
