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
  m_SerialBaudrate    = 214285;
  m_Intensities       = true;
  m_FixedResolution   = false;
  m_AutoReconnect     = false;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 16.0;
  m_MinRange          = 0.08;
  m_GlassNoise        = true;
  m_SunNoise          = true;
  isScanning          = false;
  node_counts         = 720;
  each_angle          = 0.5;
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
    lidarPtr = nullptr;
  }

  isScanning = false;
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    delay(80);
    return false;
  }

  node_info nodes[2048];
  size_t   count = _countof(nodes);
  size_t all_nodes_counts = node_counts;

  //wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(nodes, count);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    op_result = lidarPtr->ascendScanData(nodes, count);
    tim_scan_start = nodes[0].stamp;
    tim_scan_end   = nodes[0].stamp;

    double scan_time = tim_scan_end - tim_scan_start;

    if (IS_OK(op_result)) {
      if (!m_FixedResolution) {
        all_nodes_counts = count;
      } else {
        all_nodes_counts = node_counts;
      }

      each_angle = 360.0 / all_nodes_counts;

      node_info *angle_compensate_nodes = new node_info[all_nodes_counts];
      memset(angle_compensate_nodes, 0, all_nodes_counts * sizeof(node_info));
      unsigned int i = 0;

      for (; i < count; i++) {
        if (nodes[i].distance_q != 0) {
          float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);

          int index = (int)(angle / each_angle);
          float angle_pre = angle - index * each_angle;
          float angle_next = (index + 1) * each_angle - angle;

          if (angle_pre < angle_next) {
            if (index < all_nodes_counts) {
              angle_compensate_nodes[index] = nodes[i];
            }
          } else {
            if (index < all_nodes_counts - 1) {
              angle_compensate_nodes[index + 1] = nodes[i];
            }
          }
        }

        if (nodes[i].stamp < tim_scan_start) {
          tim_scan_start = nodes[i].stamp;
        }

        if (nodes[i].stamp > tim_scan_end) {
          tim_scan_end = nodes[i].stamp;
        }

      }

      LaserScan scan_msg;

      if (m_MaxAngle < m_MinAngle) {
        float temp = m_MinAngle;
        m_MinAngle = m_MaxAngle;
        m_MaxAngle = temp;
      }


      int counts = all_nodes_counts * ((m_MaxAngle - m_MinAngle) / 360.0f);
      int angle_start = 180 + m_MinAngle;
      int node_start = all_nodes_counts * (angle_start / 360.0f);

      scan_time = (tim_scan_end - tim_scan_start)/1e9;
      scan_msg.ranges.resize(counts);
      scan_msg.intensities.resize(counts);
      float range = 0.0;
      float intensity = 0.0;
      int index = 0;

      for (size_t i = 0; i < all_nodes_counts; i++) {
        range = (float)angle_compensate_nodes[i].distance_q / 1000.f;
        uint8_t intensities = (uint8_t)(angle_compensate_nodes[i].sync_quality >>
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

        if (i < all_nodes_counts / 2) {
          index = all_nodes_counts / 2 - 1 - i;
        } else {
          index = all_nodes_counts - 1 - (i - all_nodes_counts / 2);
        }

        if (m_IgnoreArray.size() != 0) {
          float angle = (float)((angle_compensate_nodes[i].angle_q6_checkbit >>
                                 LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);

          if (angle > 180) {
            angle = 360 - angle;
          } else {
            angle = -angle;
          }

          for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2) {
            if ((m_IgnoreArray[j] < angle) && (angle <= m_IgnoreArray[j + 1])) {
              range = 0.0;
              break;
            }
          }
        }

        if (range > m_MaxRange || range < m_MinRange) {
          range = 0.0;
        }

        int pos = index - node_start ;

        if (0 <= pos && pos < counts) {
          scan_msg.ranges[pos] =  range;
          scan_msg.intensities[pos] = intensity;
        }
      }

      scan_msg.system_time_stamp = tim_scan_start;
      scan_msg.self_time_stamp = tim_scan_start;
      scan_msg.config.min_angle = DEG2RAD(m_MinAngle);
      scan_msg.config.max_angle = DEG2RAD(m_MaxAngle);

      if (scan_msg.config.max_angle - scan_msg.config.min_angle == 2 * M_PI) {
        scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) /
                                        (double)counts;
        scan_msg.config.time_increment = scan_time / (double)counts;
      } else {
        scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) /
                                        (double)(counts - 1);
        scan_msg.config.time_increment = scan_time / (double)(counts - 1);
      }

      scan_msg.config.scan_time = scan_time;
      scan_msg.config.min_range = m_MinRange;
      scan_msg.config.max_range = m_MaxRange;
      outscan = scan_msg;
      delete[] angle_compensate_nodes;
      return true;


    }

  } else {
    if (IS_FAIL(op_result)) {
      // Error? Retry connection
      //this->disconnect();
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
      fprintf(stderr, "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
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
  if(isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
  }
  isScanning = false;
  return true;
}

bool CYdLidar::checkLidarAbnormal() {
  node_info nodes[2048];
  size_t   count = _countof(nodes);
  result_t op_result =  lidarPtr->grabScanData(nodes, count);
  if (IS_OK(op_result)) {
    return false;
  }
  op_result =  lidarPtr->grabScanData(nodes, count);
  return !IS_OK(op_result);
}

bool CYdLidar::getDeviceInfo() {
  if (!lidarPtr) {
    return false;
  }
  bool ret = false;
  std::string buffer;
  size_t size;
  result_t op_result = lidarPtr->getDeviceInfo(buffer, size);
  if (!IS_OK(op_result)) {
    fprintf(stderr, "get Device Information Error\n");
  }

  std::vector<std::string> result;
  std::string delim = "EAI&TEMI]:";
  ydlidar::split(buffer, result, delim);
  for(int i = 0; i < result.size(); i++) {
      std::string string_buf = result[i];
      string_buf = ydlidar::trim(string_buf);
      delim = '!';
      string_buf.erase(string_buf.find_last_not_of(delim) + 1);
      delim = "\n";
      string_buf.erase(string_buf.find_last_not_of(delim) + 1);
      delim = "[";
      string_buf.erase(string_buf.find_last_not_of(delim) + 1);
      delim = "EAI&TEMI]:";
      string_buf.erase(string_buf.find_last_not_of(delim) + 1);
      fprintf(stdout, "match: %s\n", string_buf.c_str());
      fflush(stdout);
      ret = true;
  }
  return ret;
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
    fprintf(stderr, "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
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
  getDeviceInfo();
  lidarPtr->setIntensities(m_Intensities);
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
    fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR check Comms.\n");
    fflush(stderr);
    return false;
  }

  if(!checkStatus()) {
    fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR check status.\n");
    fflush(stderr);
    return false;
  }
  if(!turnOn()) {
    fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR Turn ON.\n");
    fflush(stderr);
  }

  return true;
}
