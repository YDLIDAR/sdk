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
  m_MaxRange          = 6.0;
  m_MinRange          = 0.08;
  m_AbnormalCheckCount = 2;
  isScanning          = false;
  isConnected         = false;
  m_GlassNoise        = true;
  m_SunNoise          = true;
  m_OffsetTime        = 0.0;
  m_pointTime         = 1e9 / 3000;
  last_node_time      = getTime();
  m_FixedSize         = 600;
  m_IgnoreArray.clear();
  nodes = new node_info[YDlidarDriver::MAX_SCAN_NODES];
  ini.SetUnicode();
  m_CalibrationFileName = "/oem/laserconfig.ini";
  m_AngleOffset = 0.0;
  m_isAngleOffsetCorrected = false;
  m_StartAngleOffset = false;
  m_RobotLidarDifference = 0;


}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();

  if (nodes) {
    delete[] nodes;
    nodes = NULL;
  }
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


//获取零位修正值
float CYdLidar::getAngleOffset() const {
  return m_AngleOffset;
}

//判断零位角度是否修正
bool CYdLidar::isAngleOffetCorrected() const {
  return m_isAngleOffsetCorrected && !m_StartAngleOffset;
}


/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &scan_msg, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    delay(20);
    hardwareError = true;
    return false;
  }

  size_t   count = YDlidarDriver::MAX_SCAN_NODES;
  //fit line
  //line feature
  std::vector<double> bearings;
  std::vector<unsigned int> indices;
  indices.clear();
  bearings.clear();
  RangeData range_data;
  range_data.clear();

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

    if (static_cast<int>(tim_scan_start - last_node_time) > -20e6 &&
        tim_scan_start < last_node_time) {
      tim_scan_start = last_node_time;
      tim_scan_end = tim_scan_start + scan_time;
    }

    if (static_cast<int>(tim_scan_start + scan_time  - tim_scan_end) >
        0) {
      tim_scan_end = tim_scan_end - m_pointTime;
      tim_scan_start = tim_scan_end -  scan_time ;
    }

    last_node_time = tim_scan_end;
    scan_msg.config.scan_time = static_cast<float>(1.0 * scan_time / 1e9);

    scan_msg.config.min_angle = angles::from_degrees(m_MinAngle);
    scan_msg.config.max_angle = angles::from_degrees(m_MaxAngle);
    scan_msg.config.time_increment = static_cast<float>(m_pointTime / 1e9);
    scan_msg.system_time_stamp = tim_scan_start;
    scan_msg.config.min_range = m_MinRange;
    scan_msg.config.max_range = m_MaxRange;
    int min_index = count;


    for (int i = 0; i < count; i++) {
      angle = static_cast<float>((nodes[i].angle_q6_checkbit >>
                                  LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f) +
              m_AngleOffset;//是否在加入修正后的零位
      range = static_cast<float>(nodes[i].distance_q2 / 4.0f / 1000.f);

      //度转换为弧度
      angle = angles::from_degrees(angle);

      //知否雷达数据旋转180度
      if (m_Reversion) {
        angle += M_PI;
      }

      //转换成右手坐标系
      angle = 2 * M_PI - angle;
      //序列化到［-M_PI, M_PI]
      angle = angles::normalize_angle(angle);

      uint8_t intensities = static_cast<uint8_t>(nodes[i].sync_quality >>
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

      //加入拟合直线数据
      if (range >= m_MinRange) {
        bearings.push_back(angle);
        indices.push_back(indices.size());
        range_data.ranges.push_back(range);
        range_data.xs.push_back(cos(angle)*range);
        range_data.ys.push_back(sin(angle)*range);
      }

    }

    scan_msg.system_time_stamp = tim_scan_start + min_index * m_pointTime;

    //当设置启动零位修正，　开始拟合直线．
    if (m_StartAngleOffset) {
      line_feature_.setCachedRangeData(bearings, indices, range_data);
      std::vector<gline> glines;
      line_feature_.extractLines(glines);
      bool find_line_angle = false;
      gline max;
      max.distance = 0.0;
      double line_angle = 0.0;

      if (glines.size()) {
        max = glines[0];
      }

      float offsetAngle = 0.0;

      for (std::vector<gline>::const_iterator it = glines.begin();
           it != glines.end(); ++it) {
        line_angle = M_PI_2 - it->angle;
        line_angle -= M_PI;
        line_angle += angles::from_degrees(m_RobotLidarDifference);
        line_angle = angles::normalize_angle(line_angle);

        //可以再加入别的策略来约束，比如直线到中心点的距离
        //角度差小于Ｍ_PI/4的直线，才算修正直线
        //m_RobotLidarDifference 值表示雷达零度与机器人然零度之间的理论差值（0,180)四个值
        //如果雷达零度和机器人零度在一个方向，当前值设置位零，如果差90就是90　反方向就是180
        if (fabs(line_angle) < fabs(angles::from_degrees(m_RobotLidarDifference) -
                                    M_PI / 4)) {
          //直线距离大于0.5的直线才算修正直线
          if (it->distance > 1.0 && it->distance > max.distance) {
            max = (*it);
            find_line_angle = true;
//              line_angle -= angles::from_degrees(m_RobotLidarDifference);
            offsetAngle = -angles::to_degrees(line_angle);
          }
        }

      }

      //找到拟合的直线，　保存修正值到文件
      if (find_line_angle) {
        m_AngleOffset += offsetAngle;
        saveOffsetAngle();
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
  if (isScanning && lidarPtr->isScanning()) {
    return true;
  }

  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      lidarPtr->stop();
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
      isScanning = false;
      return false;
    }
  }

  //检测雷达，　是否异常
  if (checkLidarAbnormal()) {
    lidarPtr->stop();
    fprintf(stderr,
            "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
    isScanning = false;
    return false;
  }

  {
    handleDeviceStatus();
  }

  //启动雷达，　　检测零位修正值
  checkCalibrationAngle();

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
  size_t   count = YDlidarDriver::MAX_SCAN_NODES;
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;
  m_pointTime = lidarPtr->getPointTime();
  std::vector<int> data;

  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0) {
      delay(check_abnormal_count * 1000);
    }

    op_result =  lidarPtr->grabScanData(nodes, count);

    if (IS_OK(op_result)) {
      data.push_back(count);
      //收集数据确认固定分辨率时，最优数据个数
      int collection = 0;

      while (collection < 5) {
        count = YDlidarDriver::MAX_SCAN_NODES;
        op_result =  lidarPtr->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
          if (abs(data.front() - count) > 20) {
            data.erase(data.begin());
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
        fflush(stdout);
      }

      return false;
    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
}

bool CYdLidar::getDeviceHealth() {
  if (!lidarPtr) {
    return false;
  }

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
    //fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
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
    //fprintf(stderr, "get Device Information Error\n");
    return false;
  }

  std::string model = "MD01";

  switch (devinfo.model) {
    case YDlidarDriver::YDLIDAR_S4:
      model = "MD01";
      break;

    default:
      break;
  }

  uint8_t Major = (uint8_t)(devinfo.firmware_version >> 8);
  uint8_t Minjor = (uint8_t)(devinfo.firmware_version & 0xff);
  printf("[YDLIDAR] Device Info:\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n"
         "Serial: ",
         Major,
         Minjor,
         (unsigned int)devinfo.hardware_version,
         model.c_str());

  for (int i = 0; i < 16; i++) {
    printf("%01X", devinfo.serialnum[i] & 0xff);
  }

  printf("\n");
  printf("[YDLIDAR INFO] Current Sampling Rate : 3K\n");
  return true;
}

bool CYdLidar::handleDeviceStatus() {
  if (!lidarPtr) {
    return false;
  }

  bool ret = getDeviceHealth();
  ret |= getDeviceInfo();

  return ret;
}

bool CYdLidar::checkCalibrationAngle() {
  m_AngleOffset = 0.0;
  m_isAngleOffsetCorrected = false;
  bool ret = false;

  if (ydlidar::fileExists(m_CalibrationFileName)) {
    SI_Error rc = ini.LoadFile(m_CalibrationFileName.c_str());

    if (rc >= 0) {
      m_isAngleOffsetCorrected = true;
      double default_value = 179.6;
      m_AngleOffset = ini.GetDoubleValue(selectionName.c_str(), paramName.c_str(),
                                         default_value);

      if (fabs(m_AngleOffset - default_value) < 0.01) {
        m_isAngleOffsetCorrected = false;
        m_AngleOffset = 0.0;
      }

      printf("[YDLIDAR INFO] Successfully obtained the %s offset angle[%f] from the calibration file[%s]\n"
             , m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset,
             m_CalibrationFileName.c_str());
      ret = true;

    } else {
      printf("[YDLIDAR INFO] Failed to open calibration file[%s]\n",
             m_CalibrationFileName.c_str());
    }
  } else {
    printf("[YDLIDAR INFO] Calibration file[%s] does not exist\n",
           m_CalibrationFileName.c_str());
  }

  return ret;
}

/*-------------------------------------------------------------
                        saveRobotOffsetAngle
-------------------------------------------------------------*/
bool CYdLidar::saveOffsetAngle() {
  bool ret = true;

  if (m_StartAngleOffset) {
    ini.SetDoubleValue(selectionName.c_str(), paramName.c_str(),
                       m_AngleOffset);
    SI_Error rc = ini.SaveFile(m_CalibrationFileName.c_str());

    if (rc >= 0) {
      m_StartAngleOffset = false;
      m_isAngleOffsetCorrected = true;
      printf("[YDLIDAR INFO] Current robot offset correction value[%f] is saved\n",
             m_AngleOffset);
    } else {
      fprintf(stderr, "Saving correction value[%f] failed\n",
              m_AngleOffset);
      m_isAngleOffsetCorrected = false;
      ret = false;
    }
  }


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
