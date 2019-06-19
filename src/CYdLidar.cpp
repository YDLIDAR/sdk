#include "CYdLidar.h"
#include "common.h"
#include <map>
#include "angles.h"


using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(0) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 115200;
  m_FixedResolution   = false;
  m_Reversion         = false;
  m_AutoReconnect     = false;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 12.0;
  m_MinRange          = 0.08;
  m_AbnormalCheckCount = 2;
  isScanning          = false;
  isConnected         = false;
  m_GlassNoise        = true;
  m_SunNoise          = true;
  m_OffsetTime        = 0.0;
  m_pointTime         = 1e9 / 3000;
  last_node_time      = getTime();
  m_FixedSize         = 500;
  m_IgnoreArray.clear();
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
    lidarPtr    = 0;
    isConnected = false;
  }
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
    delay(50);
    hardwareError = true;
    return false;
  }

  node_info nodes[2048];
  size_t   count = _countof(nodes);
  //  wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(nodes, count);
  uint64_t tim_scan_end = getTime();

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

    if (tim_scan_start - last_node_time > -2e6 &&
        tim_scan_start - last_node_time < 0) {
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
                       LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
      range = (float)nodes[i].distance_q2 / 4.0f / 1000.f;

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

  if (isScanning && lidarPtr->isScanning()) {
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
    lidarPtr->stopMotor();
  } else {
    return false;
  }

  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
  }

  isScanning = false;
  return true;
}


/*-------------------------------------------------------------
            checkLidarAbnormal
-------------------------------------------------------------*/
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

    printf("[YDLIDAR INFO] Now YDLIDAR SDK VERSION: %s\n",
           YDlidarDriver::getSDKVersion().c_str());
    fflush(stdout);

  }

  if (isConnected) {
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

  printf("[YDLIDAR INFO] Connection established in %s[%d]:\n",
         m_SerialPort.c_str(),
         m_SerialBaudrate);
  fflush(stdout);

  isConnected = true;
  return true;
}


/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  if (!lidarPtr) {
    return false;
  }

  if (isScanning && lidarPtr->isScanning()) {
    return true;
  }

  return false;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize() {
  bool ret = true;

  if (!checkCOMMs()) {
    fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
    fflush(stderr);
    return false;
  }

  if (!turnOn()) {
    fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
    fflush(stderr);
    ret = false;

  }

  return ret;

}
