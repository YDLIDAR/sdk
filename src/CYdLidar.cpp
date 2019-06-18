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
  m_AbnormalCheckCount  = 4;
  m_FixedCount        = -1;
  Major               = 0;
  Minjor              = 0;
  m_IgnoreArray.clear();
  node_duration = 1e9 / 4000;
  m_OffsetTime = 0.0;
  last_node_time = getTime();

  sensor_matrix.setIdentity();
  sensor_matrix_inv.setIdentity();
  robot_matrix.setIdentity();
  current_sensor_vector.setOne();
  lidar_sensor_vector.setOne();
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

void CYdLidar::setOdometry(const odom_t &odom) {
  if (!lidarPtr) {
    return;
  }

  ScopedLocker l(queue_lock);

  if (imu_queue.size() > MAX_QUEUE_SIZE) {
    imu_queue.pop_front();
  }

  imu_queue.push_back(odom);
}

void CYdLidar::setLidarPose(const pose2D_t &pose) {
  sensor_matrix(0, 0) = cos(pose.theta);
  sensor_matrix(0, 1) = -sin(pose.theta);
  sensor_matrix(0, 2) = pose.x;
  sensor_matrix(1, 0) = sin(pose.theta);
  sensor_matrix(1, 1) = cos(pose.theta);
  sensor_matrix(1, 2) = pose.y;
  sensor_matrix(2, 0) = 0;
  sensor_matrix(2, 1) = 0;
  sensor_matrix(2, 2) = 1;
  sensor_matrix_inv = matrix::inv(sensor_matrix);
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &scan_msg, bool &hardwareError) {
  hardwareError			= false;

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
  uint64_t tim_scan_end = getTime();

  if (IS_OK(op_result)) {
    uint64_t scan_time = node_duration * (count - 1);
    tim_scan_end += m_OffsetTime * 1e9;
    tim_scan_end -= node_duration;
    tim_scan_start = tim_scan_end -  scan_time ;

    if (tim_scan_start < last_node_time) {
      tim_scan_start = last_node_time;
      tim_scan_end = tim_scan_start + scan_time;
    }

    last_node_time = tim_scan_end;
    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;
    uint64_t current_node_time = getTime();
    odom_t last_imu;
    memset(&last_imu, 0, sizeof(last_imu));
    odom_t current_imu;
    memset(&current_imu, 0, sizeof(current_imu));

    bool find_frist_node = false;

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    scan_msg.system_time_stamp = tim_scan_start;
    scan_msg.config.min_angle = angles::from_degrees(m_MinAngle);
    scan_msg.config.max_angle =  angles::from_degrees(m_MaxAngle);
    scan_msg.config.scan_time =  scan_time / 1e9;
    scan_msg.config.time_increment = scan_time / (double)(count - 1);
    scan_msg.config.min_range = m_MinRange;
    scan_msg.config.max_range = m_MaxRange;

    {
      ScopedLocker l(queue_lock);

      for (int i = 0; i < count; i++) {
        angle = (float)((nodes[i].angle_q6_checkbit >>
                         LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
        range = (float)nodes[i].distance_q2 / 1000.f;
        intensity = (float)(nodes[i].sync_quality);
        angle = angles::from_degrees(angle);
        current_node_time = tim_scan_start + i * node_duration;

        if (range > m_MaxRange || range < m_MinRange) {
          range = 0.0;
          intensity = 0.0;
        }

        //逆时针
        angle = 2 * M_PI - angle;
        angle = angles::normalize_angle(angle);

        current_sensor_vector(0) = range * cos(angle);
        current_sensor_vector(1) = range * sin(angle);
        current_sensor_vector(2) = 1;

        //对齐时间戳
        if (i % 4 == 0) {
          if (!imu_queue.empty()) {
            uint64_t min = 1e9;

            for (std::list<odom_t>::const_iterator it = imu_queue.begin();
                 it != imu_queue.end();
                 ++it) {
              uint64_t diff =  current_node_time - (*it).time_now;

              if (abs(diff) < min) {
                min = abs(diff);
                last_imu = *it;

                if (i == 0) {
                  current_imu = last_imu;
                  find_frist_node = true;
                }
              }
            }
          }
        }

        double dth = 0.0;

        if (find_frist_node) {
          dth = last_imu.theta - current_imu.theta;
        }

        robot_matrix.setIdentity();
        robot_matrix(0, 0) = cos(dth);
        robot_matrix(0, 1) = sin(dth);
        robot_matrix(0, 2) = 0;
        robot_matrix(1, 0) = -sin(dth);
        robot_matrix(1, 1) = cos(dth);
        robot_matrix(1, 2) = 0;

        //robot frame
        lidar_sensor_vector = robot_matrix * current_sensor_vector;

        double lx = lidar_sensor_vector(0);
        double ly = lidar_sensor_vector(1);
        double new_range = hypot(lx, ly);

        if (range < scan_msg.config.min_range) {
          new_range = 0.0;
          intensity = 0.0;
        }

        double new_angle = atan2(ly, lx);
        new_angle = angles::normalize_angle(new_angle);

        if (new_angle >= scan_msg.config.min_angle &&
            new_angle <= scan_msg.config.max_angle) {
          LaserPoint data;
          data.angle = new_angle;
          data.intensity = intensity;
          data.range = new_range;
          scan_msg.data.push_back(data);
        }

      }
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
    fprintf(stderr,
            "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
    isScanning = false;
    return false;
  }

  node_duration = lidarPtr->getPointTime();
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

  if (devinfo.model != YDlidarDriver::YDLIDAR_S4) {
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

  bool ret = getDeviceHealth();

  if (!ret) {
    delay(2000);
    ret = getDeviceHealth();

    if (!ret) {
      delay(1000);
    }
  }

  if (!getDeviceInfo()) {
    delay(2000);
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

  return true;
}
