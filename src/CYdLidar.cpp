#include "CYdLidar.h"
#include "common.h"
#include <map>
#include <angles.h>

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
  m_AutoReconnect     = true;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 12.0;
  m_MinRange          = 0.08;
  m_ScanFrequency     = 8;
  isScanning          = false;
  node_counts         = 720;
  each_angle          = 0.5;
  m_AbnormalCheckCount  = 4;
  m_FixedCount        = -1;
  Major               = 0;
  Minjor              = 0;
  m_IgnoreArray.clear();
  m_pointTime         = 1e9 / 4000;
  last_node_time = getTime();
  print = false;
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

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    delay(400 / m_ScanFrequency);
    return false;
  }

  size_t   count = YDlidarDriver::MAX_SCAN_NODES;
  size_t all_nodes_counts = node_counts;

  //wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(global_nodes, count);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    op_result = lidarPtr->ascendScanData(global_nodes, count);
    uint32_t scan_time = m_pointTime * (count - 1);
    tim_scan_end -= m_pointTime;
    tim_scan_start = tim_scan_start -  scan_time ;

    if (static_cast<int>(tim_scan_start - last_node_time) > -20e6 &&
        static_cast<int>(tim_scan_start - last_node_time) < 0) {
      tim_scan_start = last_node_time + m_pointTime;
      tim_scan_end = tim_scan_start + scan_time;
    }

    if (static_cast<int>(tim_scan_start + scan_time  - tim_scan_end) >
        0) {
      tim_scan_end = tim_scan_end - m_pointTime;
      tim_scan_start = tim_scan_end -  scan_time;
    }

    last_node_time = tim_scan_end;

    float range = 0.0;


    if (IS_OK(op_result) && count > 1) {
      if (!m_FixedResolution) {
        all_nodes_counts = count;
      } else {
        all_nodes_counts = node_counts;

        if (m_FixedCount > 0) {
          all_nodes_counts = m_FixedCount;
        }
      }

      each_angle = 360.0 / all_nodes_counts;
      node_info *angle_compensate_nodes = new node_info[all_nodes_counts];
      memset(angle_compensate_nodes, 0, all_nodes_counts * sizeof(node_info));
      unsigned int i = 0;

      for (; i < count; i++) {
        if (global_nodes[i].distance_q2 != 0) {
          float angle = static_cast<float>((global_nodes[i].angle_q6_checkbit >>
                                            LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);

          int inter = static_cast<int>(angle / each_angle);
          float angle_pre = angle - inter * each_angle;
          float angle_next = (inter + 1) * each_angle - angle;

          if (angle_pre < angle_next) {
            if (inter < all_nodes_counts) {
              angle_compensate_nodes[inter] = global_nodes[i];
            }
          } else {
            if (inter < all_nodes_counts - 1) {
              angle_compensate_nodes[inter + 1] = global_nodes[i];
            }
          }
        }

        if (global_nodes[i].stamp < tim_scan_start) {
          tim_scan_start = global_nodes[i].stamp;
        }

        if (global_nodes[i].stamp > tim_scan_end) {
          tim_scan_end = global_nodes[i].stamp;
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

      scan_msg.ranges.resize(counts, 0.0);
      scan_msg.intensities.resize(counts, 0.0);
      float range = 0.0;
      float intensity = 0.0;
      int index = 0;

      for (size_t i = 0; i < all_nodes_counts; i++) {
        range = static_cast<float>(angle_compensate_nodes[i].distance_q2 / 4.f / 1000);
        intensity = static_cast<float>(angle_compensate_nodes[i].sync_quality >>
                                       LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

        if (i < all_nodes_counts / 2) {
          index = all_nodes_counts / 2 - 1 - i;
        } else {
          index = all_nodes_counts - 1 - (i - all_nodes_counts / 2);
        }

        if (m_IgnoreArray.size() != 0) {
          float angle = static_cast<float>((angle_compensate_nodes[i].angle_q6_checkbit >>
                                            LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);

          if (angle > 180) {
            angle = 360 - angle;
          } else {
            angle = -angle;
          }

          for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2) {
            if ((m_IgnoreArray[j] < angle) && (angle <= m_IgnoreArray[j + 1])) {
              range = 0.0;
              intensity = 0.0;
              break;
            }
          }
        }

        if (range > m_MaxRange || range < m_MinRange) {
          range = 0.0;
          intensity = 0.0;
        }

        int pos = index - node_start ;

        if (0 <= pos && pos < counts) {
          scan_msg.ranges[pos] =  range;
          scan_msg.intensities[pos] = intensity;
        }
      }




      scan_msg.system_time_stamp = tim_scan_start;
      scan_msg.self_time_stamp = tim_scan_start;
      scan_msg.config.min_angle = from_degrees(m_MinAngle);
      scan_msg.config.max_angle = from_degrees(m_MaxAngle);

      if (scan_msg.config.max_angle - scan_msg.config.min_angle == 2 * M_PI) {
        scan_msg.config.ang_increment = static_cast<double>(scan_msg.config.max_angle -
                                        scan_msg.config.min_angle) /
                                        (double)counts;
      } else {
        scan_msg.config.ang_increment = static_cast<double>(scan_msg.config.max_angle -
                                        scan_msg.config.min_angle) /
                                        (double)(counts - 1);
      }

      scan_msg.config.time_increment = scan_time / (double)counts;
      scan_msg.config.time_increment /= 1e9;
      scan_msg.config.scan_time = static_cast<float>(scan_time / 1e9);
      scan_msg.config.min_range = m_MinRange;
      scan_msg.config.max_range = m_MaxRange;
      outscan = scan_msg;
      delete[] angle_compensate_nodes;
      return true;


    }

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

  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0) {
      delay(check_abnormal_count * 1000);
    }

    op_result =  lidarPtr->grabScanData(global_nodes, count);

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
  result_t op_result = lidarPtr->getDeviceInfo(devinfo);

  if (!IS_OK(op_result)) {
    if (print) {
      fprintf(stderr, "get Device Information Error\n");
    }

    return false;
  }

  if (devinfo.model != YDlidarDriver::YDLIDAR_S4 &&
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
    printf("[YDLIDAR] Connection established in [%s][%d]:\n"
           "Model: %s",
           m_SerialPort.c_str(),
           m_SerialBaudrate,
           model.c_str());
    node_counts         = 360;
    each_angle          = 1.0;
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
      delay(1000);
    }
  }

  print = false;

  if (!getDeviceInfo()) {
    delay(2000);
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
