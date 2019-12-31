﻿/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
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
  m_SerialBaudrate    = 230400;
  m_FixedResolution   = true;
  m_Reversion         = true;
  m_Inverted          = true;//
  m_AutoReconnect     = true;
  m_SingleChannel     = false;
  m_TOFLidar          = false;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 32.0;
  m_MinRange          = 0.1;
  m_SampleRate        = 5;
  m_ScanFrequency     = 10;
  isScanning          = false;
  m_FixedSize         = 720;
  frequencyOffset     = 0.4;
  m_AbnormalCheckCount  = 4;
  Major               = 0;
  Minjor              = 0;
  m_IgnoreArray.clear();
  m_PointTime         = 1e9 / 5000;
  m_OffsetTime        = 0.0;
  m_AngleOffset       = 0.0;
  lidar_model = YDlidarDriver::YDLIDAR_G2B;
  last_node_time = getTime();
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

//get zero angle offset value
float CYdLidar::getAngleOffset() const {
  return m_AngleOffset;
}

bool CYdLidar::isAngleOffetCorrected() const {
  return m_isAngleOffsetCorrected;
}

bool CYdLidar::isRangeValid(double reading) const {
  if (reading >= m_MinRange && reading <= m_MaxRange) {
    return true;
  }

  return false;
}

bool CYdLidar::isRangeIgnore(double angle) const {
  bool ret = false;

  for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2) {
    if ((angles::from_degrees(m_IgnoreArray[j]) <= angle) &&
        (angle <= angles::from_degrees(m_IgnoreArray[j + 1]))) {
      ret = true;
      break;
    }
  }

  return ret;
}


/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan,
                                bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    delay(200 / m_ScanFrequency);
    return false;
  }

  size_t   count = YDlidarDriver::MAX_SCAN_NODES;

  //wait Scan data:
  uint64_t tim_scan_start = getTime();
  uint64_t startTs = tim_scan_start;
  result_t op_result =  lidarPtr->grabScanData(global_nodes, count);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    uint64_t scan_time = m_PointTime * (count - 1);
    tim_scan_end += m_OffsetTime * 1e9;
    tim_scan_end -= m_PointTime;
    tim_scan_end -= global_nodes[0].stamp;
    tim_scan_start = tim_scan_end -  scan_time ;

    if (tim_scan_start < startTs) {
      tim_scan_start = startTs;
      tim_scan_end = tim_scan_start + scan_time;
    }

    if ((last_node_time + m_PointTime) >= tim_scan_start) {
      tim_scan_start = last_node_time + m_PointTime;
      tim_scan_end = tim_scan_start + scan_time;
    }

    last_node_time = tim_scan_end;

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    int all_node_count = count;

    outscan.config.min_angle = angles::from_degrees(m_MinAngle);
    outscan.config.max_angle =  angles::from_degrees(m_MaxAngle);
    outscan.config.scan_time =  static_cast<float>(scan_time * 1.0 / 1e9);
    outscan.config.time_increment = outscan.config.scan_time / (double)(count - 1);
    outscan.config.min_range = m_MinRange;
    outscan.config.max_range = m_MaxRange;
    outscan.stamp = tim_scan_start;
    outscan.points.clear();

    if (m_FixedResolution) {
      all_node_count = m_FixedSize;
    }
    outscan.config.angle_increment = (outscan.config.max_angle -
                                      outscan.config.min_angle) / (all_node_count - 1);

    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;

    for (int i = 0; i < count; i++) {
      angle = static_cast<float>((global_nodes[i].angle_q6_checkbit >>
                                  LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f) + m_AngleOffset;

      if (isOctaveLidar(lidar_model)) {
        range = static_cast<float>(global_nodes[i].distance_q2 / 2000.f);
      } else {
        if (m_TOFLidar) {
          range = static_cast<float>(global_nodes[i].distance_q2 / 1000.f);
        } else {
          range = static_cast<float>(global_nodes[i].distance_q2 / 4000.f);
        }

      }

      intensity = static_cast<float>(global_nodes[i].sync_quality);
      angle = angles::from_degrees(angle);

      //Rotate 180 degrees or not
      if (m_Reversion) {
        angle = angle + M_PI;
      }

      //Is it counter clockwise
      if (m_Inverted) {
        angle = 2 * M_PI - angle;
      }

      angle = angles::normalize_angle(angle);

      //ignore angle
      if (isRangeIgnore(angle)) {
        range = 0.0;
      }

      //valid range
      if (!isRangeValid(range)) {
        range = 0.0;
        intensity = 0.0;
      }

      if (angle >= outscan.config.min_angle &&
          angle <= outscan.config.max_angle) {
        LaserPoint point;
        point.angle = angle;
        point.range = range;
        point.intensity = intensity;

        if (outscan.points.empty()) {
          outscan.stamp = tim_scan_start + i * m_PointTime;
        }

        outscan.points.push_back(point);
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
      lidarPtr->stop();
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
      isScanning = false;
      return false;
    }
  }

  m_PointTime = lidarPtr->getPointTime();

  if (checkLidarAbnormal()) {
    lidarPtr->stop();
    fprintf(stderr,
            "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
    isScanning = false;
    return false;
  }

  if (m_SingleChannel) {
    handleSingleChannelDevice();
  }

  m_PointTime = lidarPtr->getPointTime();
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

      if (!m_SingleChannel && IS_OK(op_result)) {
        return !IS_OK(op_result);
      }
    }

    if (IS_OK(op_result) && lidarPtr->getSingleChannel()) {
      data.push_back(count);
      int collection = 0;

      while (collection < 5) {
        count = YDlidarDriver::MAX_SCAN_NODES;
        start_time = getTime();
        op_result =  lidarPtr->grabScanData(global_nodes, count);
        end_time = getTime();


        if (IS_OK(op_result)) {
          if (abs((long)data.front() - (long)count) > 10) {
            data.erase(data.begin());
          }

          scan_time = 1.0 * static_cast<int64_t>(end_time - start_time) / 1e9;

          if (scan_time > 0.05 && scan_time < 0.5 && lidarPtr->getSingleChannel()) {
            m_SampleRate = static_cast<int>((count / scan_time + 500) / 1000);
            m_PointTime = 1e9 / (m_SampleRate * 1000);
            lidarPtr->setPointTime(m_PointTime);
          }

          data.push_back(count);
        }

        collection++;
      }

      if (data.size() > 1) {
        int total = accumulate(data.begin(), data.end(), 0);
        int mean =  total / data.size(); //mean value
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

  if (!isSupportLidar(devinfo.model)) {
    printf("[YDLIDAR INFO] Current SDK does not support current lidar models[%s]\n",
           lidarModelToString(devinfo.model).c_str());
    return false;
  }

  frequencyOffset     = 0.4;
  std::string model = "G2";
  lidar_model = devinfo.model;
  model = lidarModelToString(devinfo.model);
  bool intensity = hasIntensity(devinfo.model);
  int defalutSampleRate = lidarModelDefaultSampleRate(devinfo.model);

  std::string serial_number;
  lidarPtr->setIntensities(intensity);
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
    serial_number += std::to_string(devinfo.serialnum[i] & 0xff);
  }

  printf("\n");

  if (hasSampleRate(devinfo.model)) {
    checkSampleRate();
  } else {
    m_SampleRate = defalutSampleRate;
  }

  printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n", m_SampleRate);

  if (hasScanFrequencyCtrl(devinfo.model)) {
    checkScanFrequency();
  }

  if (hasZeroAngle(devinfo.model)) {
    checkCalibrationAngle(serial_number);
  }

  return true;
}

void CYdLidar::handleSingleChannelDevice() {
  if (!lidarPtr || !lidarPtr->getSingleChannel()) {
    return;
  }

  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo);

  if (!IS_OK(op_result)) {
    return;
  }

  lidar_model = devinfo.model;
  uint8_t Major = (uint8_t)(devinfo.firmware_version >> 8);
  uint8_t Minjor = (uint8_t)(devinfo.firmware_version & 0xff);
  printf("[YDLIDAR] Device Info:\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %dK\n"
         "Serial: ",
         Major,
         Minjor,
         (unsigned int)devinfo.hardware_version,
         m_SampleRate);

  for (int i = 0; i < 16; i++) {
    printf("%01X", devinfo.serialnum[i] & 0xff);
  }

  printf("\n");
  printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n", m_SampleRate);
  return;
}

void CYdLidar::checkSampleRate() {
  sampling_rate _rate;
  _rate.rate = 3;
  int _samp_rate = 20;
  int try_count = 0;
  m_FixedSize = 2880;
  result_t ans = lidarPtr->getSamplingRate(_rate);

  if (IS_OK(ans)) {
    switch (m_SampleRate) {
      case 10:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_4K;
        break;

      case 16:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_8K;
        break;

      case 18:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_9K;
        break;

      case 20:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_10K;
        break;

      default:
        _samp_rate = _rate.rate;
        break;
    }

    if (!isOctaveLidar(lidar_model)) {
      _rate.rate = 2;
      _samp_rate = 9;
      try_count = 0;
      m_FixedSize = 1440;

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

      if (lidar_model == YDlidarDriver::YDLIDAR_F4PRO) {
        _rate.rate = 0;
        _samp_rate = 4;
        try_count = 0;
        m_FixedSize = 720;

        switch (m_SampleRate) {
          case 4:
            _samp_rate = YDlidarDriver::YDLIDAR_RATE_4K;
            break;

          case 6:
            _samp_rate = YDlidarDriver::YDLIDAR_RATE_8K;
            break;

          default:
            _samp_rate = _rate.rate;
            break;
        }

      }
    }

    while (_samp_rate != _rate.rate) {
      ans = lidarPtr->setSamplingRate(_rate);
      try_count++;

      if (try_count > 6) {
        break;
      }
    }

    switch (_rate.rate) {
      case YDlidarDriver::YDLIDAR_RATE_4K:
        _samp_rate = 10;
        m_FixedSize = 1440;

        if (!isOctaveLidar(lidar_model)) {
          _samp_rate = 4;
          m_FixedSize = 720;
        }

        break;

      case YDlidarDriver::YDLIDAR_RATE_8K:
        m_FixedSize = 2400;
        _samp_rate = 16;

        if (!isOctaveLidar(lidar_model)) {
          _samp_rate = 8;
          m_FixedSize = 1440;

          if (lidar_model == YDlidarDriver::YDLIDAR_F4PRO) {
            _samp_rate = 6;
            m_FixedSize = 720;
          }
        }

        break;

      case YDlidarDriver::YDLIDAR_RATE_9K:
        m_FixedSize = 2600;
        _samp_rate = 18;

        if (!isOctaveLidar(lidar_model)) {
          _samp_rate = 9;
          m_FixedSize = 1440;
        }

        break;

      case YDlidarDriver::YDLIDAR_RATE_10K:
        m_FixedSize = 2800;
        _samp_rate = 20;

        if (!isOctaveLidar(lidar_model)) {
          _samp_rate = 10;
          m_FixedSize = 1440;
        }

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

  if (isSupportScanFrequency(lidar_model, m_ScanFrequency)) {
    m_ScanFrequency += frequencyOffset;
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
    m_ScanFrequency += frequencyOffset;
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
  result_t ans = RESULT_FAIL;
  offset_angle angle;
  int retry = 0;
  m_isAngleOffsetCorrected = false;

  while (retry < 2) {
    ans = lidarPtr->getZeroOffsetAngle(angle);

    if (IS_OK(ans)) {
      if (angle.angle > 720 || angle.angle < -720) {
        ans = lidarPtr->getZeroOffsetAngle(angle);

        if (!IS_OK(ans)) {
          continue;
          retry++;
        }
      }

      m_isAngleOffsetCorrected = (angle.angle != 720);
      m_AngleOffset = angle.angle / 4.0;
      printf("[YDLIDAR INFO] Successfully obtained the %s offset angle[%f] from the lidar[%s]\n"
             , m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset,
             serialNumber.c_str());
      return;
    }

    retry++;
  }

  printf("[YDLIDAR INFO] Current %s AngleOffset : %f°\n",
         m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset);
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

  lidarPtr->setSingleChannel(m_SingleChannel);

  if (!m_SingleChannel) {
    m_TOFLidar = false;
  }

  lidarPtr->setTOFLidar(m_TOFLidar);

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
