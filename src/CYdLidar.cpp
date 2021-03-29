//
// The MIT License (MIT)
//
// Copyright (c) 2020 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#include "CYdLidar.h"
#include "common.h"
#include <map>
#include<numeric>
#include "angles.h"


using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(0) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 256000;
  m_FixedResolution   = false;
  m_Reversion         = false;
  m_AutoReconnect     = false;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 12.0;
  m_MinRange          = 0.05;
  m_AbnormalCheckCount = 3;
  isScanning          = false;
  isConnected         = false;
  m_OffsetTime        = 0.0;
  package_transfer_time = 0;
  last_node_time      = getTime();
  sample_rate         = 3121;
  point_interval_time = 1e9 / sample_rate;
  m_ScanFrequency     = 6.0;
  fixed_size          = 520;
  frequency_offset    = 0.0;
  zero_offset_angle   = 0.0;
  single_channel      = false;
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
  return fixed_size;
}

lidar_error_t CYdLidar::getDriverError() const {
  if (lidarPtr) {
    return lidarPtr->getDriverError();
  }

  return UnknownError;
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &scan_msg, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    scan_msg.data.clear();
    delay(200 / m_ScanFrequency);
    hardwareError = false;
    return false;
  }

  //  wait Scan data:
  uint64_t tim_scan_start = getTime();
  laser_packages.points.clear();
  result_t op_result = lidarPtr->grabScanData(&laser_packages);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    LaserPoint point;

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    size_t count = laser_packages.points.size();
    uint64_t scan_time = point_interval_time * (count - 1);
    tim_scan_end -= package_transfer_time;
    tim_scan_end += m_OffsetTime * 1e9;
    tim_scan_end -= point_interval_time;
    tim_scan_start = tim_scan_end -  scan_time ;
    last_node_time = tim_scan_end;
    int all_node_count = count;

    if (m_FixedResolution) {
      all_node_count = fixed_size;
    }

    scan_msg.data.clear();
    scan_msg.config.scan_time = static_cast<float>(1.0 * scan_time / 1e9);
    scan_msg.config.fixed_size = all_node_count;
    scan_msg.config.min_angle = angles::from_degrees(m_MinAngle);
    scan_msg.config.max_angle = angles::from_degrees(m_MaxAngle);
    scan_msg.config.time_increment = scan_msg.config.scan_time / (double)(
                                       all_node_count - 1);
    scan_msg.system_time_stamp = tim_scan_start;
    scan_msg.config.min_range = m_MinRange;
    scan_msg.config.max_range = m_MaxRange;
    scan_msg.lidar_scan_frequency = laser_packages.info.info[0] / 10.0;

    if (!checkHealth(laser_packages.info)) {
      hardwareError = true;
      return false;
    }

    for (int i = 0; i < count; ++i) {
      point = laser_packages.points[i];
      point.range = point.range / 1000.f;
      point.angle = angles::from_degrees(point.angle + zero_offset_angle);
      point.compensation_angle = angles::from_degrees(point.compensation_angle);

      if (m_Reversion) {
        point.angle += M_PI;
      }

      point.angle = 2 * M_PI - point.angle;
      point.compensation_angle *= -1.0;
      point.angle = angles::normalize_angle(point.angle);

      if (point.range > m_MaxRange || point.range < m_MinRange) {
        point.range = 0.0;
      }

      if (point.angle >= scan_msg.config.min_angle &&
          point.angle <= scan_msg.config.max_angle) {
        if (scan_msg.data.empty()) {
          scan_msg.system_time_stamp = tim_scan_start + i * point_interval_time;
        }

        scan_msg.data.push_back(point);
      }
    }

    if (m_FixedResolution) {
      scan_msg.data.resize(all_node_count);
    }

    return true;

  } else {
    if (IS_FAIL(op_result)) {
      // Error? Retry connection
    }
  }

  return false;
}

bool CYdLidar::checkHealth(const ct_packet_t &info) {
  bool ret = true;

  if (IS_OK(ydlidar::protocol::check_ct_packet_t(info))) {
    lidar_error_t err = ydlidar::protocol::convert_ct_packet_to_error(info);

    if (err != NoError) {
      ret = false;
    }
  }

  return ret;
}

/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  if (isScanning && lidarPtr->isScanning()) {
    return true;
  }

  uint32_t startTs = getms();
  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x, %s\n", op_result,
              ydlidar::protocol::DescribeError(lidarPtr->getDriverError()));
      lidarPtr->stop();
      isScanning = false;
      return false;
    }
  }

  point_interval_time = lidarPtr->getPointIntervalTime();
  package_transfer_time = lidarPtr->getPackageTransferTime();

  if (checkLidarAbnormal()) {
    fprintf(stderr,
            "[CYdLidar][%fs] Failed to turn on the Lidar, because %s.\n",
            (getms() - startTs) / 1000.0,
            ydlidar::protocol::DescribeError(lidarPtr->getDriverError()));
    lidarPtr->stop();
    isScanning = false;
    return false;
  }

  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO][%fs] Now YDLIDAR is scanning ......\n",
         (getms() - startTs) / 1000.0);
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


/*-------------------------------------------------------------
            checkLidarAbnormal
-------------------------------------------------------------*/
bool CYdLidar::checkLidarAbnormal() {
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;

  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 1) {
      delay(check_abnormal_count * 50);
    }

    LaserFan packages;
    packages.points.clear();
    op_result =  lidarPtr->grabScanData(&packages);

    if (IS_OK(op_result)) {
      return !IS_OK(op_result);
    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
}

bool CYdLidar::getDeviceHealth(uint32_t timeout) {
  if (!lidarPtr) {
    return false;
  }

  uint32_t startTs = getms();
  result_t op_result;
  device_health healthinfo;
  op_result = lidarPtr->getHealth(healthinfo, timeout);

  if (IS_OK(op_result)) {
    printf("[YDLIDAR][%fs]:Lidar running correctly ! The health status: %s\n",
           (getms() - startTs) / 1000.0,
           (int)healthinfo.status == 0 ? "good" : "bad");

    if (healthinfo.status == 2) {
      fprintf(stderr,
              "Error, YDLIDAR internal error detected. Please reboot the device to retry.\n");
      return false;
    } else {
      return true;
    }

  } else {
//    fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
    return false;
  }
}

bool CYdLidar::getDeviceInfo(uint32_t timeout) {
  if (!lidarPtr) {
    return false;
  }

  uint32_t startTs = getms();
  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo, timeout);

  if (!IS_OK(op_result)) {
    fprintf(stderr, "get Device Information Error: %s\n",
            ydlidar::protocol::DescribeError(lidarPtr->getDriverError()));
    fflush(stderr);
    return false;
  }

  std::string model = format("S2Pro[%d]", devinfo.model);

  if (devinfo.model == YDlidarDriver::YDLIDAR_TSA) {
    model = format("TSA[%d]", devinfo.model);
  }

  uint8_t Major = (uint8_t)(devinfo.firmware_version >> 8);
  uint8_t Minjor = (uint8_t)(devinfo.firmware_version & 0xff);
  printf("[YDLIDAR][%fs] Device Info:\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n"
         "Serial: ",
         (getms() - startTs) / 1000.0,
         Major,
         Minjor,
         (unsigned int)devinfo.hardware_version,
         model.c_str());

  for (int i = 0; i < 16; i++) {
    printf("%01X", devinfo.serialnum[i] & 0xff);
  }

  printf("\n");
  checkScanFrequency();
  //checkZeroOffsetAngle();
  printf("[YDLIDAR INFO] Current Sampling Rate : %d/s\n", sample_rate);
  return true;
}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency() {
  uint32_t startTs = getms();
  float frequency = 6.4f;
  scan_frequency_t _scan_frequency;
  float hz = 0.f;
  m_ScanFrequency += frequency_offset;
  result_t ans = lidarPtr->getScanFrequency(_scan_frequency) ;

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

  ans = lidarPtr->getScanFrequency(_scan_frequency);

  if (IS_OK(ans)) {
    frequency = _scan_frequency.frequency / 100.0f;
    m_ScanFrequency = frequency;
  }

  m_ScanFrequency -= frequency_offset;
  fixed_size = sample_rate / (m_ScanFrequency - 0.1);
  printf("[YDLIDAR INFO][%fs] Current Scan Frequency: %fHz\n",
         (getms() - startTs) / 1000.0, m_ScanFrequency);
  return true;
}

bool CYdLidar::checkZeroOffsetAngle() {
  bool ret = false;
  zero_offset_angle = 0.0;
  uint32_t startTs = getms();
  result_t ans;
  offset_angle_t angle;
  ans = lidarPtr->getZeroOffsetAngle(angle);

  if (IS_OK(ans)) {
    zero_offset_angle = angle.angle / 4.0;
    printf("[YDLIDAR INFO][%fs] Obtained Zero Offset Angle[%f°] \n",
           (getms() - startTs) / 1000.0, zero_offset_angle);
    fflush(stdout);
    ret = true;
  } else {
    printf("[YDLIDAR ERROR][%fs] Failed to Get Zero Offset Angle[%f°] \n",
           (getms() - startTs) / 1000.0, zero_offset_angle);
    fflush(stdout);
  }

  return ret;
}

bool CYdLidar::checkStatus() {
  uint32_t startTs = getms();

  if (!checkCOMMs()) {
    return false;
  }

  single_channel = false;
  bool ret  = getDeviceHealth();

  if (!ret) {
    ret = getDeviceHealth(100);
  }

  if (!getDeviceInfo()) {
    if (!ret) {
      ret = getDeviceInfo();

      if (!ret) {
        single_channel = true;
        ret = true;
      }
    } else {
      ret = getDeviceInfo();
    }
  } else {
    ret = true;
  }

  printf("[YDLIDAR INFO][%fs] single channel:  %d\n",
         (getms() - startTs) / 1000.0, single_channel);
  fflush(stdout);
  lidarPtr->setSingleChannel(single_channel);
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

  if (!checkStatus()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check status under [%s] and [%d].\n",
            m_SerialPort.c_str(), m_SerialBaudrate);
    fflush(stderr);
    return false;
  }

  printf("LiDAR init success!\n");
  fflush(stdout);
  return ret;
}
