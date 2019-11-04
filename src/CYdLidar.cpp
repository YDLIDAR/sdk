#include "CYdLidar.h"
#include "common.h"
#include <map>
#include <regex>

using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(nullptr), m_freq_callback(nullptr) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 214285;
  m_Intensities       = true;
  m_FixedResolution   = false;
  m_AutoReconnect     = false;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180;
  m_MaxRange          = 16.0;
  m_MinRange          = 0.08;
  m_SampleRate        = 5;
  m_GlassNoise        = true;
  m_SunNoise          = true;
  isScanning          = false;
  m_AbnormalCheckCount  = 2;
  m_IgnoreArray.clear();
  m_angle_threshold.resize(2 * MAXCHECKTIMES);
  m_angle_threshold[CHECK_ANGLE_MIN] = (225);
  m_angle_threshold[CHECK_ANGLE_MAX] = (270);
  m_angle_threshold[SIGMA_ANGLE_MIN] = (270);
  m_angle_threshold[SIGMA_ANGLE_MAX] = (295);
  m_angle_threshold[SIGN_ANGLE_MIN]  = (295);
  m_angle_threshold[SIGN_ANGLE_MIN]  = (315);
  m_angle_threshold[SURE_ANGLE_MIN]  = (315);
  m_angle_threshold[SURE_ANGLE_MAX]  = (335);

  check_queue_size.resize(MAXCHECKTIMES);
  auto_check_sum_queue.resize(MAXCHECKTIMES);
  auto_check_distance.resize(MAXCHECKTIMES);
  m_Calibration_angle.resize(MAXCALIBRATIONRANGE);
  m_Calibration_distance.resize(MAXCALIBRATIONRANGE);
  m_distance_queue.resize(MAXCALIBRATIONRANGE);
  m_last_check_distance.resize(MAXCALIBRATIONRANGE);
  m_percentage.resize(MAXCALIBRATIONRANGE);
  m_check_percentage.resize(MAXCALIBRATIONRANGE);
  m_pass_percentage.resize(MAXCALIBRATIONRANGE);
  m_check_percentage[0] = 4.9;//修正合格率
  m_pass_percentage[0] = 4.9;//
  m_check_percentage[1] = 0.9;//修正后合格率
  m_pass_percentage[1] = 0.9;


  m_mean_distance_queue.resize(MAXCALIBRATIONRANGE);
  m_Calibration_angle[0] = 0;
  m_Calibration_angle[1] = 10;
  m_Calibration_distance[0] = 3000.0;
  m_Calibration_distance[1] = 8000.0;

  m_action_startup = false;
  has_check_flag = false;
  current_frequency = 0.0;
  last_frequency = 0.0;
  m_action_step = 0;
  m_action_state = 0;
  action_check_time = getTime();
  has_check_state = false;
  m_state  = NORMAL;
  m_last_state = IDEL;
  m_check_state_error = NOERROR;
  resetCheckState();
  setMaxLowFrequencyTimes(MaxPlusTimes);
  setMaxHightFrequencyTimes(MaxPlusTimes);
  setMaxEchoTimes(2);
  setChangeFrequency(false);
  setCheckFinished(true);
  setLastFrequencyStatus(true);
  setCurrentFrequencyStatus(true);
  setResult(false);
  m_FixedSize = 721;
  nodes = new node_info[YDlidarDriver::MAX_SCAN_NODES];
  m_pointTime         = 1e9 / 5000;
  m_packageTime       = 0;
  last_node_time      = getTime();
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
  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = nullptr;
  }

  isScanning = false;
}


void CYdLidar::RegisterCtrlFreqCallback(LIDARCtrlFreqCallback callback) {
  m_freq_callback = callback;
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    delay(20);
    return false;
  }

  size_t   count = YDlidarDriver::MAX_SCAN_NODES;
  size_t all_nodes_counts = m_FixedSize;
  //wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(nodes, count);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    uint64_t scan_time = m_pointTime * (count - 1);
    tim_scan_end -= m_packageTime;
    tim_scan_end -= m_pointTime;
    tim_scan_start = tim_scan_end -  scan_time ;
    last_node_time = tim_scan_end;
    outscan.config.scan_time = static_cast<float>(1.0 * scan_time / 1e9);
    outscan.config.time_increment = static_cast<float>(m_pointTime / 1e9);


    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;

    if (!m_FixedResolution) {
      all_nodes_counts = count;
    } else {
      all_nodes_counts = m_FixedResolution;
    }

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    outscan.config.min_angle = angles::from_degrees(m_MinAngle);
    outscan.config.max_angle = angles::from_degrees(m_MaxAngle);
    int counts = all_nodes_counts * ((m_MaxAngle - m_MinAngle) / 360.0f);
    outscan.config.ang_increment = (outscan.config.max_angle -
                                    outscan.config.min_angle) / (counts - 1);
    outscan.intensities.resize(counts, 0.0);
    outscan.ranges.resize(counts, 0.0);
    retSetData();

    for (int i = 0; i < count; i++) {
      angle = (float)((nodes[i].angle_q6_checkbit >>
                       LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
      range = (float)nodes[i].distance_q / 1000.f;

      handleScanData(angle, range);
      angle = angles::from_degrees(angle);
      angle = 2 * M_PI - angle;

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

      if (angle >= outscan.config.min_angle && angle <= outscan.config.max_angle) {

        int index = (angle - outscan.config.min_angle) / outscan.config.ang_increment +
                    0.5;

        if (index >= 0 && index < counts) {
          outscan.ranges[index] = range;
          outscan.intensities[index] = intensity;
        }
      }

    }


    outscan.system_time_stamp = tim_scan_start;
    outscan.config.min_range = m_MinRange;
    outscan.config.max_range = m_MaxRange;
    {
      current_frequency = 1.0 / outscan.config.scan_time;
      handleCheckData();
      OnEnter(current_frequency);
      last_frequency = current_frequency;
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
      lidarPtr->stop();
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
      isScanning = false;
      return false;
    }
  }

  m_pointTime = lidarPtr->getPointTime();
  m_packageTime = lidarPtr->getPackageTime();

  if (checkLidarAbnormal()) {
    lidarPtr->stop();
    fprintf(stderr,
            "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
    isScanning = false;
    return false;
  }

  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  lowSpeed();
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

    count = YDlidarDriver::MAX_SCAN_NODES;
    op_result =  lidarPtr->grabScanData(nodes, count);

    if (IS_OK(op_result)) {
      return false;
    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
}


bool CYdLidar::getDeviceInfo() {
  if (!lidarPtr) {
    return false;
  }

  bool ret = false;
  std::string buffer;
  buffer.clear();
  size_t size = 0;
  result_t op_result = lidarPtr->getDeviceInfo(buffer, size);

  if (!IS_OK(op_result)) {
    fprintf(stderr, "get Device Information Error\n");
  }

  std::vector<std::string> result;
  std::string delim = "EAI&TEMI]:";
  ydlidar::split(buffer, result, delim);

  for (int i = 0; i < result.size(); i++) {
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


//雷达低速命令小于6.7hz
void CYdLidar::lowSpeed() {
  if (!lidarPtr) {
    return;
  }

  if (!isScanning || !lidarPtr->isscanning()) {
    return ;
  }

  if (m_freq_callback) {
    m_freq_callback(true);
  }

}


//雷达高速命令大于11.7hz
void CYdLidar::hightSpeed() {
  if (!lidarPtr) {
    return;
  }

  if (!isScanning || !lidarPtr->isscanning()) {
    return ;
  }

  if (m_freq_callback) {
    m_freq_callback(false);
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

  getDeviceInfo();
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
