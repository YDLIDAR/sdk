/*
*  YDLIDAR SYSTEM
*  YDLIDAR DRIVER
*
*  Copyright 2015 - 2018 EAI TEAM
*  http://www.eaibot.com
*
*/
#include "ydlidar_driver.h"
#include "common.h"
#include <math.h>
#include "angles.h"
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


const char *YDlidarDriver::DescribeError(DriverError err) {
  char const *errorString = "Unknown error";

  switch (err) {
    case YDlidarDriver::NoError:
      errorString = ("No error");
      break;

    case YDlidarDriver::DeviceNotFoundError:
      errorString = ("Device is not found");
      break;

    case YDlidarDriver::PermissionError:
      errorString = ("Device is not permission");
      break;


    case YDlidarDriver::UnsupportedOperationError:
      errorString = ("unsupported operation");
      break;

    case YDlidarDriver::NotOpenError:
      errorString = ("Device is not open");
      break;

    case YDlidarDriver::TimeoutError:
      errorString = ("Operation timed out");
      break;

    case YDlidarDriver::BlockError:
      errorString = ("Device Block");
      break;

    case YDlidarDriver::NotBufferError:
      errorString = ("Device Failed");
      break;

    case YDlidarDriver::TrembleError:
      errorString = ("Device Tremble");
      break;

    case YDlidarDriver::LaserFailureError:
      errorString = ("Laser Failure");
      break;

    default:
      // an empty string will be interpreted as "Unknown error"
      break;
  }

  return errorString;
}


YDlidarDriver::YDlidarDriver():
  _serial(NULL) {
  isConnected         = false;
  isScanning          = false;
  //serial config
  m_intensities       = false;
  isAutoReconnect     = true;
  isAutoconnting      = false;
  m_baudrate          = 230400;
  isSupportMotorCtrl  = true;
  scan_node_count     = 0;

  m_pointTime         = 1e9 / 5000;
  trans_delay         = 0;
  m_node_time_ns      = getTime();
  m_node_last_time_ns = getTime();
  scan_frequence      = 0;
  m_sampling_rate     = -1;
  model               = -1;
  m_Maxjor            = 0;
  m_Minjor            = 0;

  //parse config
  PackageSampleBytes  = 2;
  IntervalSampleAngle = 0.0;
  FirstSampleAngle    = 0;
  LastSampleAngle     = 0;
  CheckSum            = 0;
  CheckSumCal         = 0;
  SampleNumlAndCTCal  = 0;
  LastSampleAngleCal  = 0;
  CheckSumResult      = true;
  LastCheckSumResult  = true;
  Valu8Tou16          = 0;

  package_Sample_Index = 0;
  IntervalSampleAngle_LastPackage = 0.0;
  m_IgnoreArray.clear();
  uint32_t size       = (m_intensities) ? sizeof(node_package) : sizeof(
                          node_packages);
  recvBuffer = new uint8_t[size];
  m_SingleChannel = false;
  package_index         = 0;
  data_header_error     = false;
  m_SupportMotorDtrCtrl = true;
  m_reconnectCount      = 0;
  buffer_size           = 0;
  m_driverErrno         = NoError;
  m_NoZeroNodeCount     = 0;
  m_autoTime            = getms();

  sequence = 0;
  scan_node_buf = new node_info[MAX_SCAN_NODES];
  local_scan = new node_info[MAX_SCAN_NODES];
}

YDlidarDriver::~YDlidarDriver() {
  ScopedLocker lock(_serial_lock);
  {
    isScanning = false;
  }
  isAutoReconnect = false;
  _thread.join();
  int delay_count = 0;

  while (!_thread.isThreadFinshed() && delay_count < 5) {
    delay(100);
    delay_count++;
  }

  {
    ScopedLocker l(_lock);

    if (_serial) {
      isConnected = false;

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

  if (recvBuffer) {
    delete[] recvBuffer;
    recvBuffer = NULL;
  }

  if (scan_node_buf) {
    delete[] scan_node_buf;
    scan_node_buf = NULL;
  }

  if (local_scan) {
    delete[] local_scan;
    local_scan = NULL;
  }
}

result_t YDlidarDriver::connect(const char *port_path, uint32_t baudrate) {
  ScopedLocker lock(_serial_lock);
  m_baudrate = baudrate;
  serial_port = string(port_path);
  {
    ScopedLocker l(_lock);

    if (!_serial) {
      _serial = new serial::Serial(port_path, m_baudrate,
                                   serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
    }

    if (!_serial->open()) {
      isConnected = false;
      return RESULT_FAIL;
    }

    isConnected = _serial->isOpen();

  }

  stopScan();
//  delay(50);
//  clearDTR();

  return RESULT_OK;
}


void YDlidarDriver::setDTR() {
  if (!isConnected) {
    return ;
  }

  if (_serial) {
    _serial->setDTR(1);
  }

}

void YDlidarDriver::clearDTR() {
  if (!isConnected) {
    return ;
  }

  if (_serial) {
    _serial->setDTR(0);
  }
}
void YDlidarDriver::flushSerial() {
  if (!isConnected) {
    return;
  }

  ScopedLocker l(_lock);

  if (_serial) {
    size_t len = _serial->available();

    if (len) {
      _serial->read(len);
    }

    _serial->flushInput();
    delay(20);
  }

}


void YDlidarDriver::disconnect() {
  isAutoReconnect = false;

  if (!isConnected) {
    return ;
  }

  ScopedLocker l(_serial_lock);
  stop();
  delay(20);
  {
    ScopedLocker l(_lock);
    isConnected = false;

    if (_serial) {
      if (_serial->isOpen()) {
        _serial->closePort();
      }
    }
  }
}


void YDlidarDriver::disableDataGrabbing() {
  {
    if (isScanning) {
      isScanning = false;
      _dataEvent.set();
    }
  }
  _thread.join();
}

bool YDlidarDriver::isscanning() const {
  return isScanning;
}
bool YDlidarDriver::isconnected() const {
  return isConnected;
}

result_t YDlidarDriver::sendCommand(uint8_t cmd, const void *payload,
                                    size_t payloadsize) {
  uint8_t pkt_header[10];
  cmd_packet *header = reinterpret_cast<cmd_packet * >(pkt_header);
  uint8_t checksum = 0;

  if (!isConnected) {
    return RESULT_FAIL;
  }

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
  if (!isConnected) {
    return RESULT_FAIL;
  }

  if (data == NULL || size == 0) {
    return RESULT_FAIL;
  }

  size_t r;

  while (size) {
    r = _serial->writeData(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::getData(uint8_t *data, size_t size) {
  if (!isConnected) {
    return RESULT_FAIL;
  }

  size_t r;

  while (size) {
    r = _serial->readData(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::waitResponseHeader(lidar_ans_header *header,
    uint32_t timeout) {
  int  recvPos     = 0;
  uint32_t startTs = getms();
  uint8_t  recvBuffer[sizeof(lidar_ans_header)];
  uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
  uint32_t waitTime = 0;

  while ((waitTime = getms() - startTs) <= timeout) {
    size_t remainSize = sizeof(lidar_ans_header) - recvPos;
    size_t recvSize = 0;
    result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

    if (!IS_OK(ans)) {
      return ans;
    }

    if (recvSize > remainSize) {
      recvSize = remainSize;
    }

    ans = getData(recvBuffer, recvSize);

    if (IS_FAIL(ans)) {
      return RESULT_FAIL;
    }

    for (size_t pos = 0; pos < recvSize; ++pos) {
      uint8_t currentByte = recvBuffer[pos];

      switch (recvPos) {
        case 0:
          if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
            continue;
          }

          break;

        case 1:
          if (currentByte != LIDAR_ANS_SYNC_BYTE2) {
            recvPos = 0;
            continue;
          }

          break;
      }

      headerBuffer[recvPos++] = currentByte;

      if (recvPos == sizeof(lidar_ans_header)) {
        return RESULT_OK;
      }
    }
  }

  return RESULT_FAIL;
}

result_t YDlidarDriver::waitForData(size_t data_count, uint32_t timeout,
                                    size_t *returned_size) {
  if (!_serial) {
    return RESULT_FAIL;
  }

  size_t length = 0;

  if (returned_size == NULL) {
    returned_size = (size_t *)&length;
  }

  return (result_t)_serial->waitfordata(data_count, timeout, returned_size);
}

result_t YDlidarDriver::checkAutoConnecting(bool error) {
  result_t ans = RESULT_FAIL;
  isAutoconnting = isScanning.load();
  m_NoZeroNodeCount = 0;

  if (m_driverErrno != BlockError) {
    UpdateDriverError(TimeoutError);
  }

  int buf_size = 0;
  bool close_serial_event = false;

  while (isAutoReconnect && isAutoconnting && isScanning) {
    {
      ScopedLocker l(_serial_lock);

      if (_serial && _serial->isOpen()) {
        buf_size = _serial->available();
      }

      if (_serial && ((m_reconnectCount >= 7) || error)) {
        if (_serial->isOpen() || isConnected) {
          if (buf_size > 0) {
            fprintf(stderr, "[YDLIDAR][%fs]: available buffer size: %d\n",
                    (getms() - m_autoTime) / 1000.0, buf_size);
            fflush(stderr);
          }

          if (buffer_size && (buffer_size % 7) == 0) {
            UpdateDriverError(BlockError);
          } else {
            if (buf_size > 0 || buffer_size > 0) {
              if (getSystemError() != BlockError) {
                UpdateDriverError(TrembleError);
              }

              buffer_size += buf_size;
            } else {
              if (getSystemError() != BlockError) {
                UpdateDriverError(NotBufferError);
              }
            }
          }

          close_serial_event = true;
          isConnected = false;
          _serial->closePort();
          delete _serial;
          _serial = NULL;
        }
      }
    }

    if (!isConnected && ((m_reconnectCount >= 7) || error) && isscanning() &&
        close_serial_event) {
      if (!IS_OK(connect(serial_port.c_str(), m_baudrate))) {
        UpdateDriverError(NotOpenError);
      }
    }

    if (m_reconnectCount > 25) {
      m_reconnectCount = 25;
    }

    int reconnect_count = 0;

    while (isAutoReconnect && isScanning && reconnect_count < m_reconnectCount) {
      delay(250);
      reconnect_count++;
    }

    int connectCount = 0;

    while (isAutoReconnect && isscanning()) {
      if (isscanning() && isAutoReconnect) {
        result_t ans = connect(serial_port.c_str(), m_baudrate);

        if (IS_OK(ans)) {
          break;
        }

        UpdateDriverError(NotOpenError);
      }

      connectCount++;

      if (connectCount > 10) {
        connectCount = 10;
      }

      int timeout_count = 0;

      while (isAutoReconnect && timeout_count < 2 * connectCount) {
        delay(250);
        timeout_count++;
      }

      if (!isAutoReconnect || !isscanning()) {
        isAutoconnting = false;
        isScanning = false;
        return RESULT_FAIL;
      }
    }

    if (!isAutoReconnect) {
      isAutoconnting = false;
      isScanning = false;
      return RESULT_FAIL;
    }

    if (isconnected()) {
      {
        if (!m_SingleChannel && close_serial_event) {
          device_info info;
          ans = getDeviceInfo(info, 500);

          if (!IS_OK(ans)) {
            stopScan();
            ans = getDeviceInfo(info, 500);
          }

          if (!IS_OK(ans)) {
            UpdateDriverError(DeviceNotFoundError);
            continue;
          }
        }
      }
      delay(10);

      if (!isAutoReconnect) {
        isAutoconnting = false;
        isScanning = false;
        return RESULT_FAIL;
      }

      {
        ScopedLocker l(_serial_lock);
        ans = startAutoScan();

        if (!IS_OK(ans)) {
          ans = startAutoScan();
        }
      }

      if (IS_OK(ans)) {
        isAutoconnting = false;

        if (getSystemError() == DeviceNotFoundError) {
          UpdateDriverError(NoError);
        }

        if (!isAutoReconnect && !m_SingleChannel) {
          stopScan();
        }

        return ans;
      } else {
        if (!isAutoReconnect && !m_SingleChannel) {
          stopScan();
        }

        UpdateDriverError(DeviceNotFoundError);
      }
    }
  }

  isAutoconnting = false;
  return RESULT_FAIL;

}

void YDlidarDriver::checkLaserFailure() {
  if (m_NoZeroNodeCount < 2) {
    if (m_driverErrno == NoError) {
      UpdateDriverError(LaserFailureError);
    }
  } else {
    if (m_driverErrno == LaserFailureError) {
      UpdateDriverError(NoError);
    }
  }

  m_NoZeroNodeCount = 0;
}

int YDlidarDriver::cacheScanData() {
//  node_info      local_buf[128];
  size_t         count = LOCAL_MAX_SCAN_NODES;
//  node_info      *local_scan = new node_info[MAX_SCAN_NODES];
  size_t         scan_count = 0;
  result_t       ans = RESULT_FAIL;
  memset(local_scan, 0, sizeof(node_info)*MAX_SCAN_NODES);
  memset(local_buf, 0, sizeof(local_buf));
  m_node_time_ns  = getTime();
  m_node_last_time_ns = getTime();
  UpdateDriverError(NoError);
  uint32_t thread_start_time = getms();
  flushSerial();
  waitScanData(local_buf, count);
  int timeout_count   = 0;
  m_reconnectCount = 0;
  buffer_size      = 0;
  _thread.updateThreadState(false);
  m_NoZeroNodeCount = 0;
  m_autoTime = getms();
  bool lastGood = false;
  int block_timeout_count = 0;

  while (isScanning) {
    count = LOCAL_MAX_SCAN_NODES;
    ans = waitScanData(local_buf, count, DEFAULT_TIMEOUT / 2);

    if (!IS_OK(ans)) {
      if (IS_FAIL(ans) ||
          timeout_count > DEFAULT_TIMEOUT_COUNT * (isAutoReconnect ? 1 : 2)) {
        if (!isAutoReconnect) {
          fprintf(stderr, "[YDLIDAR][ERROR][%fs] Exit scan thread completed!!!\n",
                  (getms() - thread_start_time) / 1000.f);
          fflush(stderr);
          {
            isScanning = false;
          }
          _thread.updateThreadState(true);
          return RESULT_FAIL;
        } else {
          if (lastGood) {
            buffer_size = 0;
            lastGood = false;
            m_autoTime = getms();
          }

          m_reconnectCount++;
          fprintf(stderr,
                  "[YDLIDAR][ERROR][%fs]: timout count: %d, Reconnecting[%d][%d][%d].....\n",
                  (getms() - m_autoTime) / 1000.0, timeout_count,
                  m_reconnectCount, ans, buffer_size);
          fflush(stderr);
          ans = checkAutoConnecting(IS_FAIL(ans));

          if (IS_OK(ans)) {
            timeout_count = 0;
            m_node_last_time_ns = getTime();
            local_scan[0].sync_flag = Node_NotSync;
          } else {
            isScanning = false;
            _thread.updateThreadState(true);
            fprintf(stdout, "[YDLIDAR][TIMEOUT][%fs] Exit scan thread completed.\n",
                    (getms() - thread_start_time) / 1000.f);
            fflush(stdout);
            return RESULT_FAIL;
          }
        }
      } else {
        local_scan[0].sync_flag = Node_NotSync;

        if (getSystemError() == BlockError) {
          block_timeout_count++;
        } else {
          block_timeout_count = 0;
        }

        if (block_timeout_count % 3 == 0) {
          timeout_count++;
        }
      }
    } else {
      timeout_count = 0;
      buffer_size = 0;

      if (m_reconnectCount != 0 ||
          getSystemError() == TimeoutError) {
        UpdateDriverError(NoError);
      }

      lastGood = true;
      buffer_size = 0;
      m_reconnectCount = 0;
      block_timeout_count = 0;
    }


    for (size_t pos = 0; pos < count; ++pos) {
      if (local_buf[pos].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
        if ((local_scan[0].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)) {
          _lock.lock();//timeout lock, wait resource copy
          memcpy(scan_node_buf, local_scan, scan_count * sizeof(node_info));
          scan_node_count = scan_count;
          _dataEvent.set();
          _lock.unlock();
        }

        scan_count = 0;
      }

      local_scan[scan_count++] = local_buf[pos];

      if (scan_count == MAX_SCAN_NODES) {
        scan_count -= 1;
      }
    }
  }

  isScanning = false;
  _thread.updateThreadState(true);
  fprintf(stdout, "[YDLIDAR][END][%fs] Exit scan thread completed.\n",
          (getms() - thread_start_time) / 1000.f);
  fflush(stdout);
  return RESULT_OK;
}

result_t YDlidarDriver::waitPackage(node_info *node, uint32_t timeout) {
  int recvPos         = 0;
  uint32_t startTs    = getms();

  uint32_t waitTime   = 0;
  uint8_t  *packageBuffer = (m_intensities) ? (uint8_t *)&package.package_Head :
                            (uint8_t *)&packages.package_Head;
  uint8_t  package_Sample_Num         = 0;
  int32_t  AngleCorrectForDistance    = 0;
  int  package_recvPos    = 0;
  uint8_t package_type    = 0;
  (*node).index = 255;
  (*node).scan_frequence  = 0;
  (*node).error_package = 0;
  (*node).debugInfo = 0xff;
  int blockRecvPos  = 0;


  if (package_Sample_Index == 0) {
    recvPos = 0;

    while ((waitTime = getms() - startTs) <= timeout) {
      size_t remainSize   = PackagePaidBytes - recvPos;
      size_t recvSize     = 0;
      result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

      if (!IS_OK(ans)) {
        return ans;
      }

      if (recvSize > remainSize) {
        recvSize = remainSize;
      }

      buffer_size += recvSize;
      getData(recvBuffer, recvSize);

      for (size_t pos = 0; pos < recvSize; ++pos) {
        uint8_t currentByte = recvBuffer[pos];

        switch (recvPos) {
          case 0:
            if (currentByte == (PH & 0xFF)) {

            } else {
              switch (blockRecvPos) {
                case 0:
                  if (currentByte == LIDAR_ANS_SYNC_BYTE1) {
                    blockRecvPos++;
                  }

                  break;

                case 1:
                  if (currentByte == LIDAR_ANS_SYNC_BYTE2) {
                    UpdateDriverError(BlockError);
                  }

                  blockRecvPos = 0;
                  break;
                  //default:
                  // break;
              }

              data_header_error = true;
              continue;
            }

            break;

          case 1:
            CheckSumCal = PH;

            if (currentByte == (PH >> 8)) {
              if (m_driverErrno == BlockError) {
                UpdateDriverError(NoError);
              }
            } else {
              data_header_error = true;
              recvPos = 0;
              continue;
            }

            break;

          case 2:
            SampleNumlAndCTCal = currentByte;
            package_type = currentByte & 0x01;

            if ((package_type == CT_Normal) || (package_type == CT_RingStart)) {
              if (package_type == CT_RingStart) {
                scan_frequence = (currentByte & 0xFE) >> 1;
              }
            } else {
              data_header_error = true;
              recvPos = 0;
              continue;
            }

            break;

          case 3:
            SampleNumlAndCTCal += (currentByte * 0x100);
            package_Sample_Num = currentByte;
            break;

          case 4:
            if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
              FirstSampleAngle = currentByte;
            } else {
              data_header_error = true;
              recvPos = 0;
              continue;
            }

            break;

          case 5:
            FirstSampleAngle += currentByte * 0x100;
            CheckSumCal ^= FirstSampleAngle;
            FirstSampleAngle = FirstSampleAngle >> 1;
            break;

          case 6:
            if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
              LastSampleAngle = currentByte;
            } else {
              data_header_error = true;
              recvPos = 0;
              continue;
            }

            break;

          case 7:
            LastSampleAngle = currentByte * 0x100 + LastSampleAngle;
            LastSampleAngleCal = LastSampleAngle;
            LastSampleAngle = LastSampleAngle >> 1;

            if (package_Sample_Num == 1) {
              IntervalSampleAngle = 0;
            } else {
              if (LastSampleAngle < FirstSampleAngle) {
                if ((FirstSampleAngle > 270 * 64) && (LastSampleAngle < 90 * 64)) {
                  IntervalSampleAngle = (float)((360 * 64 + LastSampleAngle -
                                                 FirstSampleAngle) / ((
                                                       package_Sample_Num - 1) * 1.0));
                  IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                } else {
                  IntervalSampleAngle = IntervalSampleAngle_LastPackage;
                }
              } else {
                IntervalSampleAngle = (float)((LastSampleAngle - FirstSampleAngle) / ((
                                                package_Sample_Num - 1) * 1.0));
                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
              }
            }

            break;

          case 8:
            CheckSum = currentByte;
            break;

          case 9:
            CheckSum += (currentByte * 0x100);
            break;
        }

        packageBuffer[recvPos++] = currentByte;
      }

      if (recvPos  == PackagePaidBytes) {
        package_recvPos = recvPos;
        break;
      }
    }

    if (PackagePaidBytes == recvPos) {
      startTs = getms();
      recvPos = 0;

      while ((waitTime = getms() - startTs) <= timeout) {
        size_t remainSize = package_Sample_Num * PackageSampleBytes - recvPos;
        size_t recvSize = 0;
        result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

        if (!IS_OK(ans)) {
          return ans;
        }

        if (recvSize > remainSize) {
          recvSize = remainSize;
        }

        buffer_size += recvSize;
        getData(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
          if (m_intensities) {
            if (recvPos % 3 == 2) {
              Valu8Tou16 += recvBuffer[pos] * 0x100;
              CheckSumCal ^= Valu8Tou16;
            } else if (recvPos % 3 == 1) {
              Valu8Tou16 = recvBuffer[pos];
            } else {
              CheckSumCal ^= recvBuffer[pos];
            }
          } else {
            if (recvPos % 2 == 1) {
              Valu8Tou16 += recvBuffer[pos] * 0x100;
              CheckSumCal ^= Valu8Tou16;
            } else {
              Valu8Tou16 = recvBuffer[pos];
            }
          }

          packageBuffer[package_recvPos + recvPos] = recvBuffer[pos];
          recvPos++;
        }

        if (package_Sample_Num * PackageSampleBytes == recvPos) {
          package_recvPos += recvPos;
          break;
        }
      }

      if (package_Sample_Num * PackageSampleBytes != recvPos) {
        return RESULT_FAIL;
      }
    } else {
      return RESULT_FAIL;
    }

    CheckSumCal ^= SampleNumlAndCTCal;
    CheckSumCal ^= LastSampleAngleCal;

    if (CheckSumCal != CheckSum) {
      CheckSumResult = false;
      data_header_error = true;
      (*node).error_package = 1;
    } else {
      CheckSumResult = true;
    }
  }

  uint8_t package_CT;

  if (m_intensities) {
    package_CT = package.package_CT;
  } else {
    package_CT = packages.package_CT;
  }

  if ((package_CT & 0x01) == CT_Normal) {
    (*node).sync_flag = Node_NotSync;
    (*node).debugInfo = 0xff;

    if (!data_header_error) {
      if (package_Sample_Index == 0) {
        package_index++;
        (*node).debugInfo = (package_CT >> 1);
        (*node).index = package_index;
      }
    } else {
      (*node).error_package = 1;
      (*node).index = 255;
      package_index = 0xff;
    }
  } else {
    (*node).sync_flag = Node_Sync;
    package_index = 0;

    if (CheckSumResult) {
      data_header_error = false;
      (*node).index = package_index;
      (*node).debugInfo = (package_CT >> 1);
      (*node).scan_frequence  = scan_frequence;
    }
  }

  (*node).sync_quality = Node_Default_Quality;

  if (CheckSumResult) {
    if (m_intensities) {
      (*node).sync_quality = ((uint16_t)((
                                           package.packageSample[package_Sample_Index].PakageSampleDistance
                                           & 0x03) << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT) |
                              (package.packageSample[package_Sample_Index].PakageSampleQuality));
      (*node).distance_q =
        package.packageSample[package_Sample_Index].PakageSampleDistance >>
        LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT;
    } else {
      (*node).distance_q = packages.packageSampleDistance[package_Sample_Index] >>
                           LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT;
      (*node).sync_quality = ((uint16_t)(0xfc |
                                         packages.packageSampleDistance[package_Sample_Index] &
                                         0x0003)) << LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

    }

    if ((*node).distance_q != 0) {
      AngleCorrectForDistance = (int32_t)(((atan(((21.8 * (155.3 - ((
                                              *node).distance_q))) / 155.3) / ((
                                                  *node).distance_q))) * 180.0 / 3.1415) * 64.0);
    } else {
      AngleCorrectForDistance = 0;
      (*node).sync_quality = 0;
    }

    if ((*node).distance_q > 10) {
      m_NoZeroNodeCount++;
    }

    if (m_IgnoreArray.size() != 0) {//eliminate the specified range angle.
      double angle = (FirstSampleAngle + IntervalSampleAngle * package_Sample_Index) /
                     64.0;
      angle = angles::normalize_angle_positive_from_degree(angle);

      for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2) {
        if ((m_IgnoreArray[j] <= angle) && (angle <= m_IgnoreArray[j + 1])) {
          (*node).distance_q = 0;
          (*node).sync_quality = 0;
          break;
        }
      }
    }

    if ((FirstSampleAngle + IntervalSampleAngle * package_Sample_Index +
         AngleCorrectForDistance) < 0) {
      (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle
                                    *
                                    package_Sample_Index + AngleCorrectForDistance + 360 * 64)) << 1) +
                                  LIDAR_RESP_MEASUREMENT_CHECKBIT;
    } else {
      if ((FirstSampleAngle + IntervalSampleAngle * package_Sample_Index +
           AngleCorrectForDistance) > 360
          * 64) {
        (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle
                                      *
                                      package_Sample_Index + AngleCorrectForDistance - 360 * 64)) << 1) +
                                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
      } else {
        (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle
                                      *
                                      package_Sample_Index + AngleCorrectForDistance)) << 1) +
                                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  } else {
    (*node).sync_flag       = Node_NotSync;
    (*node).sync_quality    = 0;
    (*node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    (*node).distance_q      = 0;
    (*node).scan_frequence  = 0;
  }



  uint8_t nowPackageNum;

  if (m_intensities) {
    nowPackageNum = package.nowPackageNum;
  } else {
    nowPackageNum = packages.nowPackageNum;
  }

  if ((*node).sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
    sequence++;
    m_node_last_time_ns = m_node_time_ns;
    uint64_t current_time_ns = getTime();
    uint64_t delay_time_ns = (nowPackageNum * PackageSampleBytes + PackagePaidBytes)
                             * trans_delay +
                             (nowPackageNum - 1) * m_pointTime;
    m_node_time_ns = current_time_ns - delay_time_ns;

    if (current_time_ns <= delay_time_ns) {
      m_node_time_ns = current_time_ns;
    }

    /*  Test No Timestamp Correction
    if (m_node_time_ns < m_node_last_time_ns) {
      if ((m_node_last_time_ns - m_node_time_ns) < 1e9 / 50) {
        m_node_time_ns = m_node_last_time_ns;
      } else {
          std::cerr << "Cannot handle time reversal due to too large time drift: "
                    << (m_node_last_time_ns - current_time_ns + delay_time_ns) * 1e-6
                    << " ms larger than 20 ms\n";
      }
    } else {//Optimize starting point timestamp
      if ((m_node_time_ns - m_node_last_time_ns) < 8 * 1e6 && !data_header_error
          && CheckSumResult && LastCheckSumResult) {
        m_node_time_ns = m_node_last_time_ns;
      }
    }
    */

    LastCheckSumResult = CheckSumResult;
    checkLaserFailure();
  }

  (*node).scan_frequence  = scan_frequence;
  (*node).stamp = m_node_time_ns + package_Sample_Index * m_pointTime;
  package_Sample_Index++;

  if (package_Sample_Index >= nowPackageNum) {
    package_Sample_Index = 0;
    m_node_time_ns = (*node).stamp + m_pointTime;
    CheckSumResult = false;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::waitScanData(node_info *nodebuffer, size_t &count,
                                     uint32_t timeout) {
  if (!isConnected) {
    count = 0;
    return RESULT_FAIL;
  }

  size_t     recvNodeCount    =  0;
  uint32_t   startTs          = getms();
  uint32_t   waitTime         = 0;
  result_t   ans              = RESULT_FAIL;

  while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
    node_info node;
    ans = waitPackage(&node, timeout - waitTime);

    if (!IS_OK(ans)) {
      count = recvNodeCount;
      return ans;
    }

    nodebuffer[recvNodeCount++] = node;

    if (recvNodeCount == count) {
      return RESULT_OK;
    }
  }

  count = recvNodeCount;
  return RESULT_FAIL;
}


result_t YDlidarDriver::grabScanData(node_info *nodebuffer, size_t &count,
                                     int *const seq,
                                     uint32_t timeout) {
  switch (_dataEvent.wait(timeout)) {
    case Event::EVENT_TIMEOUT:
      count = 0;
      return RESULT_TIMEOUT;

    case Event::EVENT_OK: {
      if (scan_node_count == 0) {
        return RESULT_FAIL;
      }

      ScopedLocker l(_lock);
      size_t size_to_copy = min(count, scan_node_count);
      memcpy(nodebuffer, scan_node_buf, size_to_copy * sizeof(node_info));
      *seq = sequence;
      count = size_to_copy;
      scan_node_count = 0;
    }

    return RESULT_OK;

    default:
      count = 0;
      return RESULT_FAIL;
  }

}


result_t YDlidarDriver::ascendScanData(node_info *nodebuffer, size_t count) {
  float inc_origin_angle = (float)360.0 / count;
  int i = 0;

  for (i = 0; i < (int)count; i++) {
    if (nodebuffer[i].distance_q == 0) {
      continue;
    } else {
      while (i != 0) {
        i--;
        float expect_angle = (nodebuffer[i + 1].angle_q6_checkbit >>
                              LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /
                             64.0f - inc_origin_angle;

        if (expect_angle < 0.0f) {
          expect_angle = 0.0f;
        }

        uint16_t checkbit = nodebuffer[i].angle_q6_checkbit &
                            LIDAR_RESP_MEASUREMENT_CHECKBIT;
        nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) <<
                                           LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
      }

      break;
    }
  }

  if (i == (int)count) {
    return RESULT_FAIL;
  }

  for (i = (int)count - 1; i >= 0; i--) {
    if (nodebuffer[i].distance_q == 0) {
      continue;
    } else {
      while (i != ((int)count - 1)) {
        i++;
        float expect_angle = (nodebuffer[i - 1].angle_q6_checkbit >>
                              LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /
                             64.0f + inc_origin_angle;

        if (expect_angle > 360.0f) {
          expect_angle -= 360.0f;
        }

        uint16_t checkbit = nodebuffer[i].angle_q6_checkbit &
                            LIDAR_RESP_MEASUREMENT_CHECKBIT;
        nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) <<
                                           LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
      }

      break;
    }
  }

  float frontAngle = (nodebuffer[0].angle_q6_checkbit >>
                      LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

  for (i = 1; i < (int)count; i++) {
    if (nodebuffer[i].distance_q == 0) {
      float expect_angle =  frontAngle + i * inc_origin_angle;

      if (expect_angle > 360.0f) {
        expect_angle -= 360.0f;
      }

      uint16_t checkbit = nodebuffer[i].angle_q6_checkbit &
                          LIDAR_RESP_MEASUREMENT_CHECKBIT;
      nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) <<
                                         LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
    }
  }

  size_t zero_pos = 0;
  float pre_degree = (nodebuffer[0].angle_q6_checkbit >>
                      LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

  for (i = 1; i < (int)count ; ++i) {
    float degree = (nodebuffer[i].angle_q6_checkbit >>
                    LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

    if (zero_pos == 0 && (pre_degree - degree > 180)) {
      zero_pos = i;
      break;
    }

    pre_degree = degree;
  }

  node_info *tmpbuffer = new node_info[count];

  for (i = (int)zero_pos; i < (int)count; i++) {
    tmpbuffer[i - zero_pos] = nodebuffer[i];
    tmpbuffer[i - zero_pos].stamp = nodebuffer[i - zero_pos].stamp;
  }

  for (i = 0; i < (int)zero_pos; i++) {
    tmpbuffer[i + (int)count - zero_pos] = nodebuffer[i];
    tmpbuffer[i + (int)count - zero_pos].stamp = nodebuffer[i +
        (int)count - zero_pos].stamp;
  }

  memcpy(nodebuffer, tmpbuffer, count * sizeof(node_info));
  delete[] tmpbuffer;

  return RESULT_OK;
}

/************************************************************************/
/* get health state of lidar                                            */
/************************************************************************/
result_t YDlidarDriver::getHealth(device_health &health, uint32_t timeout) {
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();

  if (m_SingleChannel) {
    health.error_code = 0;
    health.status = 0;
    return RESULT_OK;
  }

  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(device_health)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&health), sizeof(health));
  }

  return RESULT_OK;
}

/************************************************************************/
/* get device info of lidar                                             */
/************************************************************************/
result_t YDlidarDriver::getDeviceInfo(device_info &info, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

//  disableDataGrabbing();
  flushSerial();

  if (m_SingleChannel) {
    info.model = YDLIDAR_S4;
    model = info.model;
    info.firmware_version = 0;
    info.hardware_version = 0;
    return RESULT_OK;
  }

  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(device_info)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&info), sizeof(info));
    model = info.model;
    m_Maxjor = info.firmware_version >> 8;
    m_Minjor = (info.firmware_version & 0xff);
  }

  return RESULT_OK;
}

/************************************************************************/
/* the set to signal quality                                            */
/************************************************************************/
void YDlidarDriver::setIntensities(const bool &isintensities) {
  if (isintensities != m_intensities) {
    if (recvBuffer) {
      delete[] recvBuffer;
    }

    uint32_t size       = (isintensities) ? sizeof(node_package) : sizeof(
                            node_packages);
    recvBuffer = new uint8_t[size];
  }

  m_intensities = isintensities;

  if (m_intensities) {
    PackageSampleBytes = 3;
  } else {
    PackageSampleBytes = 2;
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

/**
 * @brief setIgnoreArray
 * set the range of angles that need to be removed.
 * @param ignore_array
 * angle list
 */
void YDlidarDriver::setIgnoreArray(const std::vector<float> ignore_array) {
  m_IgnoreArray = ignore_array;
}


void YDlidarDriver::checkTransferDelay() {
  //calc stamp
  trans_delay = _serial->getByteTime();
  m_pointTime = 1e9 / 5000;

  switch (model) {
    case YDLIDAR_G4://g4
    case YDLIDAR_G4PRO://G4PRO
      if (m_sampling_rate == -1) {
        sampling_rate _rate;
        getSamplingRate(_rate);
        m_sampling_rate = _rate.rate;
      }

      switch (m_sampling_rate) {
        case 0:
          m_pointTime = 1e9 / 4000;
          break;

        case 1:
          m_pointTime = 1e9 / 8000;
          break;

        case 2:
          m_pointTime = 1e9 / 9000;
          break;
      }

      break;

    case YDLIDAR_S2:
    case YDLIDAR_S4://S2
      m_pointTime = 1e9 / 3000;
      break;

    default:
      break;
  }
}

/************************************************************************/
/*  start to scan                                                       */
/************************************************************************/
result_t YDlidarDriver::startScan(bool force, uint32_t timeout) {
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  if (isScanning) {
    return RESULT_OK;
  }

  //wait previous thread end
  int timeout_count = 0;

  while (!_thread.isThreadFinshed() && timeout_count < 3) {
    delay(100);
    timeout_count++;
  }

  stop();
  checkTransferDelay();
  flushSerial();
  delay(30);
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) !=
        RESULT_OK) {
      return ans;
    }

    if (!m_SingleChannel) {
      lidar_ans_header response_header;

      if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
        return ans;
      }

      if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
        return RESULT_FAIL;
      }

      if (response_header.size < 5) {
        return RESULT_FAIL;
      }
    }

    ans = this->createThread();
    return ans;
  }

  if (m_SingleChannel) {
    startMotor();
  }

  return RESULT_OK;
}


result_t YDlidarDriver::stopScan(uint32_t timeout) {
  UNUSED(timeout);

  if (!isConnected) {
    return RESULT_FAIL;
  }

  ScopedLocker l(_lock);
  sendCommand(LIDAR_CMD_STOP);
  delay(5);
  sendCommand(LIDAR_CMD_STOP);
  delay(10);
  return RESULT_OK;
}

result_t YDlidarDriver::createThread() {
  _thread = CLASS_THREAD(YDlidarDriver, cacheScanData);

  if (_thread.getHandle() == 0) {
    isScanning = false;
    return RESULT_FAIL;
  }

  isScanning = true;
  return RESULT_OK;
}


result_t YDlidarDriver::startAutoScan(bool force, uint32_t timeout) {
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  flushSerial();
  delay(20);
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) !=
        RESULT_OK) {
      return ans;
    }

    if (!m_SingleChannel) {
      lidar_ans_header response_header;

      if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
        return ans;
      }

      if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
        return RESULT_FAIL;
      }

      if (response_header.size < 5) {
        return RESULT_FAIL;
      }
    }

  }

  if (m_SingleChannel) {
    startMotor();
  }

  return RESULT_OK;
}

/************************************************************************/
/*   stop scan                                                   */
/************************************************************************/
result_t YDlidarDriver::stop() {
  if (isAutoconnting) {
    isAutoReconnect = false;
    disableDataGrabbing();
    isScanning = false;
    stopScan();

    if (m_SingleChannel) {
      stopMotor();
    }

    return RESULT_OK;

  }

  disableDataGrabbing();
  stopScan();

  if (m_SingleChannel) {
    stopMotor();
  }

  int timeout_count = 0;

  //wait thread finished
  while (!_thread.isThreadFinshed() && timeout_count < 3) {
    delay(100);
    timeout_count++;
  }

  return RESULT_OK;
}

/************************************************************************/
/*  reset device                                                        */
/************************************************************************/
result_t YDlidarDriver::reset(uint32_t timeout) {
  UNUSED(timeout);
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  ScopedLocker l(_lock);

  if ((ans = sendCommand(LIDAR_CMD_RESET)) != RESULT_OK) {
    return ans;
  }

  return RESULT_OK;
}

/************************************************************************/
/*  startMotor                                                          */
/************************************************************************/
result_t YDlidarDriver::startMotor() {
  ScopedLocker l(_lock);

  if (m_SupportMotorDtrCtrl) {
    setDTR();
    delay(500);
  } else {
    clearDTR();
    delay(500);
  }

  return RESULT_OK;
}

/************************************************************************/
/*  stopMotor                                                           */
/************************************************************************/
result_t YDlidarDriver::stopMotor() {
  ScopedLocker l(_lock);

  if (m_SupportMotorDtrCtrl) {
    clearDTR();
    delay(500);
  } else {
    setDTR();
    delay(500);
  }

  return RESULT_OK;
}

/************************************************************************/
/* get the current scan frequency of lidar                              */
/************************************************************************/
result_t YDlidarDriver::getScanFrequency(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_AIMSPEED)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* add the scan frequency by 1Hz each time                              */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyAdd(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADD)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* decrease the scan frequency by 1Hz each time                         */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyDis(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DIS)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* add the scan frequency by 0.1Hz each time                            */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyAddMic(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADDMIC)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* decrease the scan frequency by 0.1Hz each time                       */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyDisMic(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DISMIC)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  get the sampling rate of lidar                                      */
/************************************************************************/
result_t YDlidarDriver::getSamplingRate(sampling_rate &rate, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_SAMPLING_RATE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
    m_sampling_rate = rate.rate;
  }
  return RESULT_OK;
}

/************************************************************************/
/*  the set to sampling rate                                            */
/************************************************************************/
result_t YDlidarDriver::setSamplingRate(sampling_rate &rate, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_SAMPLING_RATE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  the get to installtion offset angle                                        */
/************************************************************************/
result_t YDlidarDriver::getZeroOffsetAngle(offset_angle &angle,
    uint32_t timeout) {
  result_t  ans = RESULT_FAIL;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  int timeoutCount = 0;
  int maxTimeout = timeout / DEFAULT_TIMEOUT;

  if (maxTimeout < 1) {
    maxTimeout = 1;
  }

  if (!isNoRibOffsetAngleLidar(model, m_Maxjor, m_Minjor)) {
    while (timeoutCount < maxTimeout) {
      ans = getRobotOffsetZone(angle);

      if (IS_OK(ans)) {
        break;
      }

      timeoutCount++;
    }
  }

  return ans;
}

/************************************************************************/
/*  the get to lidar zero offset angle                                        */
/************************************************************************/
result_t YDlidarDriver::getLidarZeroOffsetAngle(offset_angle &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  ans = getZeroOffsetZone(angle, timeout);
  return ans;
}

result_t YDlidarDriver::saveRobotOffsetAngle(offset_angle &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  int timeoutCount = 0;
  int maxTimeout = timeout / DEFAULT_TIMEOUT;

  if (maxTimeout < 1) {
    maxTimeout = 1;
  }

  while (timeoutCount < maxTimeout) {
    ans = saveRobotOffsetZone(angle);

    if (IS_OK(ans)) {
      break;
    }

    timeoutCount++;
  }

  return ans;
}

/************************************************************************/
/*  the get to zero offset angle                                        */
/************************************************************************/
result_t YDlidarDriver::getZeroOffsetZone(offset_angle &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_OFFSET_ANGLE)) != RESULT_OK) {
      return RESULT_TIMEOUT;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return RESULT_TIMEOUT;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_TIMEOUT;
    }

    if (response_header.size < sizeof(offset_angle)) {
      return RESULT_TIMEOUT;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_TIMEOUT;
    }

    getData(reinterpret_cast<uint8_t *>(&angle), sizeof(angle));
  }

  return RESULT_OK;
}

result_t YDlidarDriver::saveZeroOffsetZone(offset_angle &angle,
    uint32_t timeout) {
  result_t  ans = RESULT_TIMEOUT;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  disableDataGrabbing();
  flushSerial();

  if (!isNoRibOffsetAngleLidar(model, m_Maxjor, m_Minjor)) {
    std::string buf;
    size_t size;
    bool ret = false;
    ans = setScanSwitchModel(buf, size, 30);

    if (!IS_OK(ans)) {
      ans = setScanSwitchModel(buf, size, 30);
    }

    if (IS_OK(ans)) {
      std::string send = format("E %d\r\n", angle.angle);
      buf.clear();
      ans = setScanSendText(send, buf, size, 40);

      if (!IS_OK(ans)) {
        ans = setScanSendText(send, buf, size, 40);
      }

      ret = IS_OK(ans);
    }

    buf.clear();
    ans = setScanExitModel(buf, size, 25);
    ans = setLidarReboot();

    if (!ret) {
      ans = RESULT_FAIL;
    }
  }

  return ans;
}

/************************************************************************/
/*  the get to rib offset angle                                        */
/************************************************************************/
result_t YDlidarDriver::getRibOffsetAngle(std::vector<offset_angle> &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  int timeoutCount = 0;
  rib_offset_angle rib_angle;
  memset(&rib_angle, 0, sizeof(rib_offset_angle));
  int maxTimeout = timeout / DEFAULT_TIMEOUT;

  if (maxTimeout < 1) {
    maxTimeout = 1;
  }

  while (timeoutCount < maxTimeout) {
    ans = getRibOffsetZone(rib_angle);

    if (IS_OK(ans)) {
      uint8_t size = uint8_t(rib_angle.angle[0].angle & 0xff);

      if (rib_angle.angle[0].angle < 1 || size >= 15) {
        return RESULT_FAIL;
      }

      for (int i = 1; i <= size; i++) {
        angle.push_back(rib_angle.angle[i]);
      }

      return ans;
    }

    timeoutCount++;
  }

  return ans;
}

result_t YDlidarDriver::saveRibOffsetAngle(std::vector<offset_angle> &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  if (angle.size() < 1 ||  angle.size() > 15) {
    return RESULT_FAIL;
  }

  int maxTimeout = timeout / DEFAULT_TIMEOUT;
  rib_offset_angle rib_angle;
  memset(&rib_angle, 0, sizeof(rib_offset_angle));

  if (maxTimeout < 1) {
    maxTimeout = 1;
  }

  rib_angle.angle[0].angle = angle.size();

  for (int i = 1; i <= angle.size() && i < 15; i++) {
    rib_angle.angle[i] = angle[i - 1];
  }

  int timeoutCount = 0;

  while (timeoutCount < maxTimeout) {
    ans = saveRibOffsetZone(rib_angle);

    if (IS_OK(ans)) {
      return ans;
    }

    timeoutCount++;
  }

  return ans;

}

result_t YDlidarDriver::saveRibOffsetZone(rib_offset_angle &angle,
    uint32_t timeout) {
  result_t  ans = RESULT_TIMEOUT;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  disableDataGrabbing();
  flushSerial();

  if (!isNoRibOffsetAngleLidar(model, m_Maxjor, m_Minjor)) {
    std::string buf;
    size_t size;
    bool ret = false;
    ans = setScanSwitchModel(buf, size, 30);

    if (!IS_OK(ans)) {
      ans = setScanSwitchModel(buf, size, 30);
    }

    if (IS_OK(ans)) {
      ret = true;
      uint8_t len = uint8_t(angle.angle[0].angle & 0xff);

      for (int i = 0; i <= len; i++) {
        std::string send = format("T %d %d\r\n", i, angle.angle[i].angle);
        buf.clear();
        ans = setScanSendText(send, buf, size, 15);

        if (!IS_OK(ans)) {
          ans = setScanSendText(send, buf, size, 15);
        }

        ret &= IS_OK(ans);
      }

      if (IS_OK(ans) && ret) {
        std::string save = format("T 30\r\n");
        ans = setScanSendText(save, buf, size, 20);

        if (!IS_OK(ans)) {
          ans = setScanSendText(save, buf, size, 20);
        }
      }
    }

    buf.clear();
    ans = setScanExitModel(buf, size, 25);
    ans = setLidarReboot();

    if (!ret) {
      ans = RESULT_FAIL;
    }
  }

  return ans;
}

result_t YDlidarDriver::saveRobotOffsetZone(offset_angle &angle,
    uint32_t timeout) {
  result_t  ans = RESULT_TIMEOUT;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  disableDataGrabbing();
  flushSerial();

  if (!isNoRibOffsetAngleLidar(model, m_Maxjor, m_Minjor)) {
    std::string buf;
    size_t size;
    bool ret = false;
    ans = setScanSwitchModel(buf, size, 30);

    if (!IS_OK(ans)) {
      ans = setScanSwitchModel(buf, size, 30);
    }

    if (IS_OK(ans)) {
      ret = true;
      std::string send = format("T 15 %d\r\n", angle.angle);
      buf.clear();
      ans = setScanSendText(send, buf, size, 15);

      if (!IS_OK(ans)) {
        ans = setScanSendText(send, buf, size, 15);
      }

      if (IS_OK(ans)) {
        std::string save = format("T 30\r\n");
        ans = setScanSendText(save, buf, size, 20);

        if (!IS_OK(ans)) {
          ans = setScanSendText(save, buf, size, 20);
        }
      }

      ret = IS_OK(ans);

    }

    buf.clear();
    ans = setScanExitModel(buf, size, 25);
    ans = setLidarReboot();

    if (!ret) {
      ans = RESULT_FAIL;
    }
  }

  return ans;
}
/************************************************************************/
/*  the get to rib offset zone                                        */
/************************************************************************/
result_t YDlidarDriver::getRibOffsetZone(rib_offset_angle &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  disableDataGrabbing();
  flushSerial();
  memset(&angle, 0, sizeof(rib_offset_angle));
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_RIB_OFFSET_ANGLE)) != RESULT_OK) {
      return RESULT_TIMEOUT;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_TIMEOUT;
    }

    if (response_header.size < sizeof(rib_offset_angle)) {
      return RESULT_TIMEOUT;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_TIMEOUT;
    }

    getData(reinterpret_cast<uint8_t *>(&angle), sizeof(angle));

  }
  return RESULT_OK;
}

/************************************************************************/
/*  the get to rib offset zone                                        */
/************************************************************************/
result_t YDlidarDriver::getRobotOffsetZone(offset_angle &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_TIMEOUT;
  }

  rib_offset_angle rov_angle;
  memset(&angle, 0, sizeof(offset_angle));
  ans = getRibOffsetZone(rov_angle, timeout);

  if (IS_OK(ans)) {
    angle.angle = rov_angle.angle[15].angle;
  }

  return ans;
}

result_t YDlidarDriver::setLidarReboot(uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  flushSerial();
  delay(20);
  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if (!IS_OK(ans = sendCommand(LIDAR_CMD_EXIT))) {
      return ans;

    }

    waitForData(1, timeout);
    flushSerial();
  }
  return RESULT_OK;
}

result_t YDlidarDriver::setScanSwitchModel(string &buffer, size_t &size,
    uint32_t count, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  flushSerial();
  delay(50);
  disableDataGrabbing();
  size = 0;
  {
    ScopedLocker l(_lock);

    if (!IS_OK(ans = sendCommand(LIDAR_CMD_ENTER_TEXT))) {
      return ans;

    }

    delay(20);
    {
      if (!IS_OK(ans = waitForData(count, timeout, &size))) {
      }
    }
    delay(1);
    {
      if (_serial->available() > size) {
        size = _serial->available();
      }

      if (size > 0) {
        uint8_t *buf = static_cast<uint8_t *>(alloca(size * sizeof(uint8_t)));
        getData(buf, size);
        buffer.append(reinterpret_cast<const char *>(buf), size);
      }

    }

  }
  return ans;
}

result_t YDlidarDriver::setScanExitModel(string &buffer, size_t &size,
    uint32_t count, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  string sendBuf = "G\r\n";
  ans = setScanSendText(sendBuf, buffer, size, count, timeout);
  return ans;
}

result_t YDlidarDriver::setScanSendText(const string &data, string &buffer,
                                        size_t &size, uint32_t count, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  flushSerial();
  delay(20);
  disableDataGrabbing();
  size = 0;
  {
    ScopedLocker l(_lock);

    if (_serial) {
      _serial->write(reinterpret_cast<const uint8_t *>(data.c_str()),
                     data.length());
    }

    delay(20);
    {
      if (!IS_OK(ans = waitForData(count, timeout, &size))) {
      }
    }
    delay(1);
    {
      if (_serial->available() > size) {
        size = _serial->available();
      }

      if (size > 0) {
        uint8_t *buf = static_cast<uint8_t *>(alloca(size * sizeof(uint8_t)));
        getData(buf, size);
        buffer.append(reinterpret_cast<const char *>(buf), size);
      }
    }

  }
  return ans;

}
void YDlidarDriver::UpdateDriverError(const YDlidarDriver::DriverError
                                      &error) {
  ScopedLocker l(_error_lock);
  m_driverErrno = error;
}

YDlidarDriver::DriverError YDlidarDriver::getSystemError() {
  ScopedLocker l(_error_lock);
  return m_driverErrno;
}

std::string YDlidarDriver::getSDKVersion() {
  return SDKVerision;
}
std::map<std::string, std::string>  YDlidarDriver::lidarPortList() {
  std::vector<PortInfo> lst = list_ports();
  std::map<std::string, std::string> ports;

  for (std::vector<PortInfo>::iterator it = lst.begin(); it != lst.end(); it++) {
    std::string port = "ydlidar" + (*it).device_id;
    ports[port] = (*it).port;
  }

  return ports;
}

}
