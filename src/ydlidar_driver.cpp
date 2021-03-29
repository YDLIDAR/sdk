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
#include "ydlidar_driver.h"
#include "common.h"
#include <math.h>
using namespace impl;

namespace ydlidar {

// A sprintf-like function for std::string
std::string format(const char *fmt, ...) {
  if (!fmt) {
    return std::string();
  }

  int   result = -1, length = 2048;
  std::string buffer;

  while (result == -1) {
    buffer.resize(length);

    va_list args;  // This must be done WITHIN the loop
    va_start(args, fmt);
    result = ::vsnprintf(&buffer[0], length, fmt, args);
    va_end(args);

    // Truncated?
    if (result >= length) {
      result = -1;
    }

    length *= 2;

    // Ok?
    if (result >= 0) {
      buffer.resize(result);
    }
  }

  return buffer;
}

YDlidarDriver::YDlidarDriver():
  _serial(NULL) {
  m_isConnected       = false;
  m_isScanning        = false;
  //串口配置参数
  isAutoReconnect     = true;
  isAutoconnting      = false;
  baudrate_           = 256000;
  isSupportMotorCtrl  = true;
  single_channel      = false;
  point_interval_time = 1e9 / 3121;
  transfer_delay      = 0;
  m_error_info        = NoError;
  m_new_protocol      = false;
  valid_data_count    = 0;
  package_transfer_time       = 0;
  ydlidar::protocol::reset_ct_packet_t(m_global_ct);
}

YDlidarDriver::~YDlidarDriver() {
  {
    m_isScanning = false;
  }

  isAutoReconnect = false;
  _thread.join();
  ScopedLocker lk(_serial_lock);

  if (_serial) {
    if (_serial->isOpen()) {
      _serial->flush();
      _serial->closePort();
    }
  }

  if (_serial) {
    delete _serial;
    _serial = NULL;
  }
}

result_t YDlidarDriver::connect(const char *port_path, uint32_t baudrate) {
  baudrate_  = baudrate;
  serial_port = string(port_path);
  ScopedLocker lk(_serial_lock);

  if (!_serial) {
    _serial = new serial::Serial(port_path, baudrate_,
                                 serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
  }

  if (!_serial->open()) {
    return RESULT_FAIL;
  }

  transfer_delay = _serial->getByteTime();
  package_transfer_time = transfer_delay * (sizeof(node_package_header_t) +
                          sizeof(node_package_payload_t));
  m_isConnected = true;
  stopScan();
  delay(50);
  clearDTR();
  return RESULT_OK;
}


void YDlidarDriver::setDTR() {
  if (!m_isConnected) {
    return ;
  }

  if (_serial) {
    _serial->setDTR(1);
  }

}

void YDlidarDriver::clearDTR() {
  if (!m_isConnected) {
    return ;
  }

  if (_serial) {
    _serial->setDTR(0);
  }
}

void YDlidarDriver::flushSerial() {
  if (!m_isConnected) {
    return;
  }

  size_t len = _serial->available();

  if (len) {
    _serial->read(len);
    delay(5);
  }
}

void YDlidarDriver::setDriverError(const lidar_error_t &er) {
  ScopedLocker l(_error_lock);
  m_error_info = er;
}

result_t YDlidarDriver::startMotor() {
  if (!single_channel) {
    return RESULT_OK;
  }

  ScopedLocker l(_lock);

  if (isSupportMotorCtrl) {
    setDTR();
    delay(100);
  } else {
    clearDTR();
    delay(100);
  }

  return RESULT_OK;
}

result_t YDlidarDriver::stopMotor() {
  if (!single_channel) {
    return RESULT_OK;
  }

  ScopedLocker l(_lock);

  if (isSupportMotorCtrl) {
    clearDTR();
    delay(100);
  } else {
    setDTR();
    delay(100);
  }

  return RESULT_OK;
}

void YDlidarDriver::flush() {
  if (!m_isConnected) {
    return ;
  }

  if (_serial) {
    _serial->flush();
  }

  delay(30);
}

void YDlidarDriver::disconnect() {
  isAutoReconnect = false;

  if (!m_isConnected) {
    return ;
  }

  stop();
  ScopedLocker l(_serial_lock);

  if (_serial) {
    if (_serial->isOpen()) {
      _serial->flush();
      _serial->closePort();
    }
  }

  m_isConnected = false;

}


void YDlidarDriver::disableDataGrabbing() {
  {
    if (m_isScanning) {
      m_isScanning = false;
      _dataEvent.set();
    }
  }
  _thread.join();
}

bool YDlidarDriver::isConnected() const {
  return m_isConnected;
}

bool YDlidarDriver::isScanning() const {
  return m_isScanning;
}

uint32_t YDlidarDriver::getPointIntervalTime() const {
  return point_interval_time;
}

uint32_t YDlidarDriver::getPackageTransferTime() const {
  return package_transfer_time;
}

lidar_error_t YDlidarDriver::getDriverError() {
  ScopedLocker l(_error_lock);
  return m_error_info;
}

result_t YDlidarDriver::getHealth(device_health &health, uint32_t timeout) {
  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  result_t ans = RESULT_FAIL;
  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH)) != RESULT_OK) {
      return ans;
    }

    lidar_error_t err = NoError;
    ans =  ydlidar::protocol::read_response_health_t(_serial, health, err,
           timeout);
    this->setDriverError(err);

    if (!IS_OK(ans)) {
      printf("[YDLIDAR ERROR][Device health]: %s\n",
             ydlidar::protocol::DescribeError(err));
      fflush(stdout);
    }
  }

  return ans;
}

result_t YDlidarDriver::getDeviceInfo(device_info &info, uint32_t timeout) {
  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  result_t ans = RESULT_FAIL;
  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO)) != RESULT_OK) {
      return ans;
    }

    lidar_error_t err = NoError;
    ans =  ydlidar::protocol::read_response_device_info_t(_serial, info, err,
           timeout);
    this->setDriverError(err);

    if (!IS_OK(ans)) {
      printf("[YDLIDAR ERROR][Device info]: %s\n",
             ydlidar::protocol::DescribeError(err));
      fflush(stdout);
    }
  }

  return ans;
}

/************************************************************************/
/* get the current scan frequency of lidar                              */
/************************************************************************/
result_t YDlidarDriver::getScanFrequency(scan_frequency_t &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_AIMSPEED)) != RESULT_OK) {
      return ans;
    }

    lidar_error_t err = NoError;
    ans =  ydlidar::protocol::read_response_scan_frequency_t(_serial, frequency,
           err,
           timeout);
    this->setDriverError(err);

    if (!IS_OK(ans)) {
      printf("[YDLIDAR ERROR][get scan frequency]: %s\n",
             ydlidar::protocol::DescribeError(err));
      fflush(stdout);
    }
  }
  return ans;
}

/************************************************************************/
/* add the scan frequency by 1Hz each time                              */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyAdd(scan_frequency_t &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADD)) != RESULT_OK) {
      return ans;
    }

    lidar_error_t err = NoError;
    ans =  ydlidar::protocol::read_response_scan_frequency_t(_serial, frequency,
           err,
           timeout);
    this->setDriverError(err);

    if (!IS_OK(ans)) {
      printf("[YDLIDAR ERROR][get scan frequency add]: %s\n",
             ydlidar::protocol::DescribeError(err));
      fflush(stdout);
    }
  }
  return ans;
}

/************************************************************************/
/* decrease the scan frequency by 1Hz each time                         */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyDis(scan_frequency_t &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DIS)) != RESULT_OK) {
      return ans;
    }

    lidar_error_t err = NoError;
    ans =  ydlidar::protocol::read_response_scan_frequency_t(_serial, frequency,
           err,
           timeout);
    this->setDriverError(err);

    if (!IS_OK(ans)) {
      printf("[YDLIDAR ERROR][get scan frequency dis]: %s\n",
             ydlidar::protocol::DescribeError(err));
      fflush(stdout);
    }
  }
  return ans;
}

/************************************************************************/
/* add the scan frequency by 0.1Hz each time                            */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyAddMic(scan_frequency_t &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADDMIC)) != RESULT_OK) {
      return ans;
    }

    lidar_error_t err = NoError;
    ans =  ydlidar::protocol::read_response_scan_frequency_t(_serial, frequency,
           err,
           timeout);
    this->setDriverError(err);

    if (!IS_OK(ans)) {
      printf("[YDLIDAR ERROR][get scan frequency add mic]: %s\n",
             ydlidar::protocol::DescribeError(err));
      fflush(stdout);
    }
  }
  return ans;
}

/************************************************************************/
/* decrease the scan frequency by 0.1Hz each time                       */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyDisMic(scan_frequency_t &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DISMIC)) != RESULT_OK) {
      return ans;
    }

    lidar_error_t err = NoError;
    ans =  ydlidar::protocol::read_response_scan_frequency_t(_serial, frequency,
           err,
           timeout);
    this->setDriverError(err);

    if (!IS_OK(ans)) {
      printf("[YDLIDAR ERROR][get scan frequency dis mic]: %s\n",
             ydlidar::protocol::DescribeError(err));
      fflush(stdout);
    }
  }
  return ans;
}

/************************************************************************/
/* get zero offset angle                                               */
/************************************************************************/
result_t YDlidarDriver::getZeroOffsetAngle(offset_angle_t &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_OFFSET_ANGLE)) != RESULT_OK) {
      return ans;
    }

    lidar_error_t err = NoError;
    ans =  ydlidar::protocol::read_response_offset_angle_t(_serial, angle,
           err,
           timeout);
    this->setDriverError(err);

    if (!IS_OK(ans)) {
      printf("[YDLIDAR ERROR][get zero offset angle]: %s\n",
             ydlidar::protocol::DescribeError(err));
      fflush(stdout);
    }
  }
  return ans;
}


result_t YDlidarDriver::sendCommand(uint8_t cmd, const void *payload,
                                    size_t payloadsize) {
  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  uint8_t pkt_header[10];
  cmd_packet_t *header = reinterpret_cast<cmd_packet_t * >(pkt_header);
  uint8_t checksum = 0;

  if (payloadsize && payload) {
    cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
  }

  header->syncByte = LIDAR_CMD_SYNC_BYTE;
  header->cmd_flag = cmd;
  sendData(pkt_header, 2) ;

  if ((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD) && payloadsize && payload) {
    checksum ^= LIDAR_CMD_SYNC_BYTE;
    checksum ^= cmd;
    checksum ^= (payloadsize & 0xFF);

    for (size_t pos = 0; pos < payloadsize; ++pos) {
      checksum ^= ((uint8_t *)payload)[pos];
    }

    uint8_t sizebyte = (uint8_t)(payloadsize);
    sendData(&sizebyte, 1);

    sendData((const uint8_t *)payload, sizebyte);

    sendData(&checksum, 1);
  }

  return RESULT_OK;
}

result_t YDlidarDriver::sendData(const uint8_t *data, size_t size) {
  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  if (data == NULL || size == 0) {
    return RESULT_FAIL;
  }

  while (size) {
    size_t r = _serial->write(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;

}

result_t YDlidarDriver::getData(uint8_t *data, size_t size) {
  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  while (size) {
    size_t r = _serial->read(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;

  }

  return RESULT_OK;

}

result_t YDlidarDriver::waitForData(size_t data_count, uint32_t timeout,
                                    size_t *returned_size) {
  size_t length = 0;

  if (returned_size == NULL) {
    returned_size = (size_t *)&length;
  }

  return (result_t)_serial->waitfordata(data_count, timeout, returned_size);
}

int YDlidarDriver::cacheScanData() {
  LaserFan       local_fan;
  LaserFan       local_scan;
  result_t       ans    = RESULT_FAIL;
  local_fan.sync_flag   = Node_NotSync;
  local_scan.sync_flag  = Node_NotSync;
  local_fan.points.clear();
  local_scan.points.clear();
  m_error_info          = NoError;
  m_new_protocol        = false;
  valid_data_count      = 0;
  ydlidar::protocol::reset_ct_packet_t(m_global_ct);
  waitScanData(local_fan);
  int timeout_count     = 0;
  int package_error     = 0;
  lidar_error_t m_last_error = m_error_info;

  while (m_isScanning) {
    ans = waitScanData(local_fan);

    if (!IS_OK(ans)) {
      if (!IS_TIMEOUT(ans) || timeout_count > DEFAULT_TIMEOUT_COUNT) {
        if (!isAutoReconnect) {
          fprintf(stderr, "exit scanning thread!!\n");
          fflush(stderr);
          {
            m_isScanning = false;
          }
          return RESULT_FAIL;
        } else {//做异常处理, 重新连接
          isAutoconnting = true;
          printf("Starting automatic reconnection.....\n");
          fflush(stdout);

          while (isAutoReconnect && isAutoconnting) {
            {
              ScopedLocker l(_serial_lock);

              if (_serial) {
                if (_serial->isOpen()) {
                  _serial->closePort();
                  delete _serial;
                  _serial = NULL;
                  m_isConnected = false;
                }
              }
            }

            while (isAutoReconnect &&
                   connect(serial_port.c_str(), baudrate_) != RESULT_OK) {
              printf("Waiting for the Lidar serial port[%s] to be available in [%d]\n",
                     serial_port.c_str(), baudrate_);
              fflush(stdout);
              delay(1000);
              this->setDriverError(DeviceNotFoundError);
            }

            if (!isAutoReconnect) {
              m_isScanning = false;
              return RESULT_FAIL;
            }

            if (isConnected()) {
              {
                ScopedLocker l(_serial_lock);
                ans = startAutoScan();
              }

              if (IS_OK(ans)) {
                timeout_count = 0;
                package_error = 0;
                local_scan.sync_flag =  Node_NotSync;

                if (getDriverError() == LidarNotFoundError ||
                    getDriverError() == DeviceNotFoundError) {
                  setDriverError(NoError);
                }

                isAutoconnting = false;
                printf("automatic connection succeeded\n");
                fflush(stdout);
              } else {
                setDriverError(LidarNotFoundError);
              }

            }
          }
        }

      } else {
        if (m_error_info == NoError) {
          setDriverError(TimeoutError);
        }

        if (m_error_info != ReadError) {
          package_error++;

          if (package_error % 2 == 0) {
            timeout_count++;
          }
        } else {
          timeout_count++;
        }

        printf("timeout[%d]: %s\n", timeout_count,
               ydlidar::protocol::DescribeError(m_error_info));
        fflush(stdout);
        local_scan.sync_flag =  Node_NotSync;
      }
    } else {
      timeout_count = 0;
      package_error = 0;
    }

    if (m_error_info != m_last_error) {
      if (m_error_info == NoError) {
        local_scan.sync_flag =  Node_NotSync;
      }
    }

    m_last_error = m_error_info;

    if (local_fan.sync_flag) {
      if ((local_scan.sync_flag)) {
        _lock.lock();//timeout lock, wait resource copy
        memcpy(&m_global_fan.info, &local_fan.info, sizeof(ct_packet_t));
        m_global_fan.sync_flag = local_fan.sync_flag;
        m_global_fan.points = local_scan.points;
        _dataEvent.set();
        _lock.unlock();

        if (valid_data_count < 5) {
          setDriverError(DataError);
        }

      }

      local_scan = local_fan;
      local_scan.points.clear();
      valid_data_count = 0;
    }

    if (local_fan.points.size()) {
      std::copy(local_fan.points.begin(), local_fan.points.end(),
                std::back_inserter(local_scan.points));
      local_fan.points.clear();
    }
  }


  m_isScanning = false;
  return RESULT_OK;
}



result_t YDlidarDriver::waitPackage(LaserFan &package, uint32_t timeout) {
  if (!m_isConnected) {
    package.points.clear();
    return RESULT_FAIL;
  }

  lidar_error_t error = NoError;
  //Si 4Byte
  tsa_scan_intensity_packet_t tscan;
  tscan.header.packageSync = Node_NotSync;


  memset(&tscan, 0, sizeof(tsa_scan_intensity_packet_t));
  result_t ans = ydlidar::protocol::read_response_tsa_scan_intensity_t(_serial,
                 tscan,
                 m_global_ct, error, timeout);

  memcpy(&package.info, &m_global_ct, sizeof(ct_packet_t));
  package.points.clear();

  if (IS_OK(ans)) {
    package.sync_flag = tscan.header.packageSync;
    result_t ret = ydlidar::protocol::parse_tsa_intensity_payload(tscan, package,
                   valid_data_count);

    if (IS_OK(ret)) {
      if (IS_OK(ydlidar::protocol::check_ct_packet_t(m_global_ct))) {
        error = ydlidar::protocol::convert_ct_packet_to_error(m_global_ct);
        setDriverError(error);
        m_new_protocol = true;
      }
    }

    if (!m_new_protocol) {
      setDriverError(error);
    }

  } else {
    setDriverError(error);
  }

  return ans;
}

result_t YDlidarDriver::waitScanData(LaserFan &package, uint32_t timeout) {
  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  return waitPackage(package, timeout);
}


result_t YDlidarDriver::grabScanData(LaserFan *fan, uint32_t timeout) {
  switch (_dataEvent.wait(timeout)) {
    case Event::EVENT_TIMEOUT:
      return RESULT_TIMEOUT;

    case Event::EVENT_OK: {
      if (m_global_fan.points.size() == 0) {
        return RESULT_FAIL;
      }

      ScopedLocker l(_lock);
      *fan = m_global_fan;
      m_global_fan.points.clear();
    }

    return RESULT_OK;

    default:
      return RESULT_FAIL;
  }

}


/**
* @brief 设置雷达异常自动重新连接 \n
* @param[in] enable    是否开启自动重连:
*     true	开启
*	  false 关闭
*/
void YDlidarDriver::setAutoReconnect(const bool &enable) {
  isAutoReconnect = enable;
}

void YDlidarDriver::setSingleChannel(bool enable) {
  single_channel = enable;
}

/************************************************************************/
/*  start to scan                                                       */
/************************************************************************/
result_t YDlidarDriver::startScan(bool force, uint32_t timeout) {
  result_t ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  if (m_isScanning) {
    return RESULT_OK;
  }

  if (!single_channel) {
    stop();
  }

  delay(10);
  startMotor();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) !=
        RESULT_OK) {
      return ans;
    }

    if (!single_channel) {
      lidar_error_t error;
      lidar_ans_header_t header;
      ans = ydlidar::protocol::read_response_header_t(_serial, header, error,
            timeout);
      setDriverError(error);

      if (!IS_OK(ans)) {
        return ans;
      }
    }

    ans = this->createThread();
    return ans;
  }
  return RESULT_OK;
}

result_t YDlidarDriver::stopScan(uint32_t timeout) {
  UNUSED(timeout);

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  ScopedLocker l(_lock);
  sendCommand(LIDAR_CMD_FORCE_STOP);
  delay(2);
  sendCommand(LIDAR_CMD_STOP);
  delay(2);
  return RESULT_OK;
}

result_t YDlidarDriver::createThread() {
  _thread = CLASS_THREAD(YDlidarDriver, cacheScanData);

  if (_thread.getHandle() == 0) {
    m_isScanning = false;
    return RESULT_FAIL;
  }

  m_isScanning = true;
  return RESULT_OK;
}


result_t YDlidarDriver::startAutoScan(bool force, uint32_t timeout) {
  result_t ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  startMotor();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) !=
        RESULT_OK) {
      return ans;
    }

    if (!single_channel) {
      lidar_error_t error;
      lidar_ans_header_t header;
      ans = ydlidar::protocol::read_response_header_t(_serial, header, error,
            timeout);
      setDriverError(error);
    }
  }
  return ans;
}

/************************************************************************/
/*   stop scan                                                   */
/************************************************************************/
result_t YDlidarDriver::stop() {
  if (isAutoconnting) {
    isAutoReconnect = false;
    m_isScanning = false;
  }

  disableDataGrabbing();
  stopScan();
  stopMotor();
  return RESULT_OK;
}

std::string YDlidarDriver::getSDKVersion() {
  return SDKVerision;
}

std::map<std::string, std::string> YDlidarDriver::lidarPortList() {
  std::vector<PortInfo> lst = list_ports();
  std::map<std::string, std::string> ports;

  for (std::vector<PortInfo>::iterator it = lst.begin(); it != lst.end(); ++it) {
    std::string port = "ydlidar" + (*it).device_id;
    ports[port] = (*it).port;
  }

  return ports;
}




}
