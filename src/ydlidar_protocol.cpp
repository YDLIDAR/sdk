
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "ydlidar_protocol.h"
#include "timer.h"

namespace ydlidar {
namespace protocol {

const char *DescribeError(const lidar_error_t &error) {
  char const *errorString = "Unknown error";

  switch (error) {
    case NoError:
      errorString = "";
      break;

    case DeviceNotFoundError:
      errorString = ("Device is not found");
      break;

    case PermissionError:
      errorString = ("Device is not permission");
      break;

    case OpenError:
      errorString = ("Open error");

      break;

    case ParityError:
      errorString = ("Parity error");

      break;

    case FramingError:
      errorString = ("Framing error");

      break;

    case BreakConditionError:
      errorString = ("Break Condition error");

      break;

    case WriteError:
      errorString = ("Write Data error");

      break;

    case ReadError:
      errorString = ("Read Data Error");

      break;

    case ResourceError:
      errorString = ("Resource temporarily unavailab");

      break;

    case UnsupportedOperationError:
      errorString = ("unsupported operation");
      break;

    case NotOpenError:
      errorString = ("Device is not open");
      break;

    case TimeoutError:
      errorString = ("Operation timed out");
      break;

    case HeaderError:
      errorString = ("Package Header Error");
      break;

    case FirstSampleAngleError:
      errorString = ("First Sample Angle Error");

      break;

    case LastSampleAngleError:
      errorString = ("Last Sample Angle Error");

      break;

    case PackageNumberError:
      errorString = ("Package Number Error");

      break;

    case CheckSumError:
      errorString = ("Check Sum Error");

      break;

    case SensorError:
      errorString = ("Sensor Error");

      break;

    case EncodeError:
      errorString = ("Encode Error");

      break;

    case PWRError:
      errorString = ("Power Error");

      break;

    case PDError:
      errorString = ("PD Error");

      break;

    case LDError:
      errorString = ("LD Error");

      break;

    case DataError:
      errorString = ("Data Error");
      break;

    case TrembleError:
      errorString = ("Lidar Tremble");
      break;

    case LidarNotFoundError:
      errorString = ("Lidar is not found");
      break;

    case UnknownError:

      break;

    default:
      // an empty string will be interpreted as "Unknown error"
      break;
  }

  return errorString;

}

lidar_error_t convert_ct_packet_to_error(const ct_packet_t &ct) {
  lidar_error_t err = NoError;

  if (ct.size > HEALTHINDEX) {
    if (ct.info[HEALTHINDEX] & response_health_error::DataError) {
      err = DataError;
    }

    if (ct.info[HEALTHINDEX] & response_health_error::PWRError) {
      err = PWRError;
    }

    if (ct.info[HEALTHINDEX] & response_health_error::PDError) {
      err = PDError;
    }

    if (ct.info[HEALTHINDEX] & response_health_error::LDError) {
      err = LDError;
    }

    if (ct.info[HEALTHINDEX] & response_health_error::EncodeError) {
      err = EncodeError;
    }

    if (ct.info[HEALTHINDEX] & response_health_error::SensorError) {
      err = SensorError;
    }
  }

  if (err == NoError) {

    if (ct.size > FREINDEX) {
      if (ct.info[FREINDEX] < 1) {
//        err = EncodeError;
      }
    }
  }

  return err;
}

void reset_ct_packet_t(ct_packet_t &ct) {
  ct.cs = 0x00;
  ct.crc = 0x01;
  ct.size = 0x00;
  ct.index = 0xff;
  ct.valid = 0x00;
  memset(ct.info, 0, sizeof(ct.info));
}

result_t check_ct_packet_t(const  ct_packet_t &ct) {
  result_t ans = RESULT_TIMEOUT;

  if (ct.valid == 0x01 && ct.size <= sizeof(ct.info) && ct.size > 0) {
    ans = RESULT_OK;
  } else if (ct.valid == 0x00 && ct.index == 0xff) {
    ans = RESULT_FAIL;
  }

  return ans;
}

void write_command(Serial *serial, uint8_t cmd) {
  assert(serial);
  uint8_t pkt_header[2];
  cmd_packet_t *header = reinterpret_cast<cmd_packet_t * >(pkt_header);
  header->syncByte = LIDAR_CMD_SYNC_BYTE;
  header->cmd_flag = cmd;
  serial->write(pkt_header, 2);
}

result_t wait_for_data(Serial *serial, size_t data_count, uint32_t timeout) {
  assert(serial);
  size_t recvSize = 0;
  result_t ans = serial->waitfordata(data_count, timeout, &recvSize);
  return ans;
}

result_t read_command(Serial *serial, uint8_t *buffer, size_t size,
                      lidar_error_t &error,
                      uint32_t timeout) {
  assert(serial);

  result_t ans = wait_for_data(serial, size, timeout);

  if (!IS_OK(ans)) {
    error = ReadError;
    return ans;
  }

  size_t r = serial->read(buffer, size);

  if (r < size) {
    error = ReadError;
    ans = RESULT_TIMEOUT;
  }

  return ans;
}

result_t read_response_header_t(Serial *serial, lidar_ans_header_t &header,
                                lidar_error_t &error, uint32_t timeout) {
  assert(serial);
  uint32_t startTs = getms();
  uint32_t waitTime = 0;
  size_t ans_header_size  = sizeof(lidar_ans_header_t);
  memset(&header, 0, ans_header_size);
  result_t ans = read_command(serial, (uint8_t *)&header,
                              ans_header_size,
                              error,
                              timeout);

  if (!IS_OK(ans)) {
    return ans;
  }

  unsigned short error_count = 0; // error count.
  ans = RESULT_TIMEOUT;
  uint8_t *p2header = (uint8_t *)&header;

  while ((waitTime = getms() - startTs) < timeout) {
    if (header.syncByte1 != LIDAR_ANS_SYNC_BYTE1) {
      error = HeaderError;
      error_count++;
      uint8_t *p2header_back = p2header + ans_header_size - 1;

      for (int i = 0; i < ans_header_size - 1; ++i) {
        p2header[i] = p2header[i + 1];
      }

      ans = read_command(serial, p2header_back, 1, error, timeout - waitTime);

      if (!IS_OK(ans)) {
        return ans;
      }

      ans = RESULT_TIMEOUT;
    } else {
      uint8_t *p3header_back = p2header + ans_header_size - 2;

      if (header.syncByte2 != LIDAR_ANS_SYNC_BYTE2) {
        error = HeaderError;
        error_count++;

        for (int i = 0; i < ans_header_size - 2; ++i) {
          p2header[i] = p2header[i + 2];
        }

        ans = read_command(serial, p3header_back, 2, error, timeout - waitTime);

        if (!IS_OK(ans)) {
          return ans;
        }

        ans = RESULT_TIMEOUT;
      } else {
        if (error == HeaderError) {
          error = NoError;
          error_count = 0;
        }

        ans = check_ans_header_t(header, error);

        if (IS_OK(ans)) {
          error = NoError;
          break;
        } else {
          p2header[0] = 0x00;
          ans = RESULT_TIMEOUT;
        }
      }
    }
  }

  return ans;
}

result_t check_ans_header_t(const lidar_ans_header_t &header,
                            lidar_error_t &error) {
  result_t ans = RESULT_TIMEOUT;

  if (header.type != LIDAR_ANS_TYPE_DEVINFO &&
      header.type != LIDAR_ANS_TYPE_DEVHEALTH &&
      header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
    error = HeaderError;
    return ans;
  }

  if (header.syncByte1 != LIDAR_ANS_SYNC_BYTE1 ||
      header.syncByte2 != LIDAR_ANS_SYNC_BYTE2) {
    error = HeaderError;
    return ans;
  }

  ans = RESULT_OK;
  return ans;
}

result_t read_response_health_t(Serial *serial, device_health &health,
                                lidar_error_t &error, uint32_t timeout) {
  assert(serial);
  lidar_ans_header_t header;
  result_t ans = read_response_header_t(serial, header, error, timeout);

  if (IS_OK(ans)) {
    if (header.size != sizeof(device_health) ||
        header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
      error = HeaderError;
      ans = RESULT_TIMEOUT;
      return ans;
    }

    ans = read_command(serial, (uint8_t *)&health, sizeof(device_health), error,
                       INFO_DEFAULT_TIMEOUT);
  }

  return ans;
}

result_t read_response_device_info_t(Serial *serial, device_info &info,
                                     lidar_error_t &error, uint32_t timeout) {
  assert(serial);
  lidar_ans_header_t header;
  result_t ans = read_response_header_t(serial, header, error, timeout);

  if (IS_OK(ans)) {
    if (header.size != sizeof(device_info) ||
        header.type != LIDAR_ANS_TYPE_DEVINFO) {
      error = HeaderError;
      ans = RESULT_TIMEOUT;
      return ans;
    }

    ans = read_command(serial, (uint8_t *)&info, sizeof(device_info), error,
                       INFO_DEFAULT_TIMEOUT);
  }

  return ans;

}

result_t read_response_sample_rate_t(Serial *serial, sampling_rate_t &rate,
                                     lidar_error_t &error, uint32_t timeout) {
  assert(serial);
  lidar_ans_header_t header;
  result_t ans = read_response_header_t(serial, header, error, timeout);

  if (IS_OK(ans)) {
    if (header.size != sizeof(sampling_rate_t) ||
        header.type != LIDAR_ANS_TYPE_DEVINFO) {
      error = HeaderError;
      ans = RESULT_TIMEOUT;
      return ans;
    }

    ans = read_command(serial, (uint8_t *)&rate, sizeof(sampling_rate_t), error,
                       INFO_DEFAULT_TIMEOUT);
  }

  return ans;
}

result_t read_response_scan_frequency_t(Serial *serial,
                                        scan_frequency_t &frequency,
                                        lidar_error_t &error, uint32_t timeout) {
  assert(serial);
  lidar_ans_header_t header;
  result_t ans = read_response_header_t(serial, header, error, timeout);

  if (IS_OK(ans)) {
    if (header.size != sizeof(scan_frequency_t) ||
        (header.type != LIDAR_ANS_TYPE_DEVINFO)) {
      error = HeaderError;
      ans = RESULT_TIMEOUT;
      return ans;
    }

    ans = read_command(serial, (uint8_t *)&frequency, sizeof(scan_frequency_t),
                       error, INFO_DEFAULT_TIMEOUT);
  }

  return ans;
}

result_t read_response_offset_angle_t(Serial *serial, offset_angle_t &angle,
                                      lidar_error_t &error, uint32_t timeout) {
  assert(serial);
  lidar_ans_header_t header;
  result_t ans = read_response_header_t(serial, header, error, timeout);

  if (IS_OK(ans)) {
    if (header.size != sizeof(offset_angle_t) ||
        header.type != LIDAR_ANS_TYPE_DEVINFO) {
      error = HeaderError;
      ans = RESULT_FAIL;
      return ans;
    }

    ans = read_command(serial, (uint8_t *)&angle, sizeof(offset_angle_t), error,
                       INFO_DEFAULT_TIMEOUT);
  }

  return ans;
}

result_t parse_payload(const scan_packet_t &scan, LaserFan &data) {
  result_t ans = RESULT_TIMEOUT;
  float AngleInc = (scan.header.packageLastSampleAngle -
                    scan.header.packageFirstSampleAngle);

  if (AngleInc < 0) {
    AngleInc += 360 * 64.f;
  }

  float AngleCorrect = 0;

  for (int i = 0; i < scan.header.nowPackageNum; ++i) {
    LaserPoint point;

    if (scan.header.nowPackageNum > 1) {
      point.angle = AngleInc * i / (scan.header.nowPackageNum - 1) +
                    scan.header.packageFirstSampleAngle;
    } else {
      point.angle = scan.header.packageFirstSampleAngle;
    }

    point.range = scan.payload[i].PackageSampleDistance;
    point.interference_sign = scan.payload[i].PackageSampleSi;
    point.intensity = 0;

    if (point.range > 0) {
      //180/M_PI=57.29578
      AngleCorrect = (atan(21.8 * (155.3 - point.range) / (155.3 * point.range)) *
                      57.29578);
    } else {
      AngleCorrect = 0;
    }

    point.angle = point.angle / 64.0 + AngleCorrect;

    if (point.angle < 0) {
      point.angle += 360.f;
    }

    if (point.angle > 360.f) {
      point.angle -= 360.f;
    }

    data.points.push_back(point);
  }

  if (scan.header.packageSync) {//sync package flag
    ans = RESULT_OK;
  }

  return ans;

}

result_t parse_intensity_payload(const scan_intensity_packet_t &scan,
                                 LaserFan &data) {
  result_t ans = RESULT_TIMEOUT;
  float AngleInc = (scan.header.packageLastSampleAngle -
                    scan.header.packageFirstSampleAngle);

  if (AngleInc < 0) {
    AngleInc += 360 * 64.f;
  }

  float AngleCorrect = 0;

  for (int i = 0; i < scan.header.nowPackageNum; ++i) {
    LaserPoint point;

    if (scan.header.nowPackageNum > 1) {
      point.angle = AngleInc * i / (scan.header.nowPackageNum - 1) +
                    scan.header.packageFirstSampleAngle;
    } else {
      point.angle = scan.header.packageFirstSampleAngle;
    }

    point.range = scan.payload[i].PackageSample.PackageSampleDistance;
    point.interference_sign = scan.payload[i].PackageSample.PackageSampleSi;
    point.intensity = scan.payload[i].PackageSampleIntensity;

    if (point.range > 0) {
      //180/M_PI=57.29578
      AngleCorrect = (atan(21.8 * (155.3 - point.range) / (155.3 * point.range)) *
                      57.29578);
    } else {
      AngleCorrect = 0;
    }

    point.angle = point.angle / 64.0 + AngleCorrect;

    if (point.angle < 0) {
      point.angle += 360.f;
    }

    if (point.angle > 360.f) {
      point.angle -= 360.f;
    }

    data.points.push_back(point);
  }

  if (scan.header.packageSync) {//sync package flag
    ans = RESULT_OK;
  }

  return ans;

}

result_t check_package_header_t(const node_package_header_t &header,
                                lidar_error_t &error) {
  result_t ans = RESULT_FAIL;

  if (!(header.packageFirstSampleAngleSync & response_scan_packet_sync::sync)) {
    error = FirstSampleAngleError;
    return ans;
  }

  if (!(header.packageLastSampleAngleSync & response_scan_packet_sync::sync)) {
    error = LastSampleAngleError;
    return ans;
  }

  if (header.nowPackageNum > 40 ||
      header.nowPackageNum < 1) {
    error = PackageNumberError;
    return ans;
  }

  if (header.packageHeaderMSB != HEADER_MSB ||
      header.packageHeaderLSB != HEADER_LSB) {
    error = HeaderError;
    return ans;
  }

  ans = RESULT_OK;
  return ans;
}

uint8_t crc8_t(uint8_t *ptr, uint16_t len, uint8_t default_crc, uint8_t poly,
               uint8_t inverted) {
  uint8_t crc = default_crc;
  uint8_t i = 0;

  while (len--) {
    crc ^= *ptr++;

    for (i = 0; i < 8; ++i) {//0x8c
      if (inverted == 1) {
        if (crc & 0x01) { /* inverted sequence 1 */
          crc = (crc >> 1) ^ poly;
        } else {
          crc >>= 1;
        }
      } else {
        if (crc & 0x80) {
          crc = (crc << 1) ^ poly;
        } else {
          crc = (crc << 1);
        }
      }
    }
  }

  return crc;
}

uint16_t checksum_response_scan_packet_t(const scan_packet_t &scan) {
  uint16_t checksum = 0;
  uint16_t *data_ptr = (uint16_t *)(&scan);
  uint16_t i = 0;

  for (i = 0; i < (sizeof(scan.header) - sizeof(scan.header.checkSum)) / 2; ++i) {
    checksum ^= *data_ptr++;
  }

  data_ptr++;

  for (i = 0; i < scan.header.nowPackageNum; ++i) {
    if (sizeof(node_package_payload_t) == 3) {//intensity protocol
      uint8_t *p1data_back = (uint8_t *)data_ptr;
      checksum ^= *p1data_back++;
      data_ptr = (uint16_t *)(p1data_back);
    }

    checksum ^= *data_ptr++;
  }

  return checksum;
}

uint16_t checksum_response_scan_intensity_packet_t(
  const scan_intensity_packet_t &scan) {
  uint16_t checksum = 0;
  uint16_t *data_ptr = (uint16_t *)(&scan);
  uint16_t i = 0;

  for (i = 0; i < (sizeof(scan.header) - sizeof(scan.header.checkSum)) / 2; ++i) {
    checksum ^= *data_ptr++;
  }

  data_ptr++;

  for (i = 0; i < scan.header.nowPackageNum; ++i) {
    if (sizeof(node_package_intensity_payload_t) == 3) {//intensity protocol
      uint8_t *p1data_back = (uint8_t *)data_ptr;
      checksum ^= *p1data_back++;
      data_ptr = (uint16_t *)(p1data_back);
    }

    checksum ^= *data_ptr++;
  }

  return checksum;
}

result_t parse_ct_packet_t(const node_package_header_t &header,
                           unsigned short error_count, ct_packet_t &ct) {

  result_t ans = RESULT_TIMEOUT;
  uint8_t *p1header = (uint8_t *)&header;

  if (error_count == 1) {
       //printf("ct.cs:%02x,crc:%02x,index:%x\n",ct.cs,ct.crc,ct.index);
    if (header.packageSync) {
      if (ct.cs == ct.crc && ct.index < sizeof(ct.info)) {
        ct.valid = 0x01;
        ct.size = ct.index;
      }

      ans = RESULT_OK;
      ct.cs = 0x00;
      ct.index = 0;
    }
  } else {
    if (error_count > 1) {
      reset_ct_packet_t(ct);
    } else {
      ct.valid = 0x00;
      ct.size = 0;
    }

    if (header.packageSync) {
      ct.cs = 0x00;
      ct.index = 0;
      ans = RESULT_FAIL;
    }
  }

  ct.cs = crc8_t(p1header + 2, sizeof(ct.cs), ct.cs);
//  printf("packageSync:%d,valid:%d\n",header.packageSync,ct.valid);
//  printf("err_count:%d,index:%d,ct.info:%x\n",error_count,ct.index,((header.packageCTInfo<<1) + header.packageSync));
  fflush(stdout);
  if (ct.index < sizeof(ct.info)) {
    ct.info[ct.index] = header.packageCTInfo;//(header.packageCTInfo<<1) + header.packageSync;
    ct.index++;
  }

  return ans;
}

result_t read_response_scan_header_t(Serial *serial,
                                     node_package_header_t &header, ct_packet_t &ct, lidar_error_t &error,
                                     uint32_t timeout) {
  assert(serial);
  uint32_t startTs = getms();
  uint32_t waitTime = 0;
  size_t node_header_size = sizeof(node_package_header_t);
  memset(&header, 0, node_header_size);
  result_t ans = read_command(serial, (uint8_t *)&header,
                              node_header_size,
                              error,
                              timeout);

  if (!IS_OK(ans)) {
    return ans;
  }

  unsigned short error_count = 0; // error count.
  ans = RESULT_TIMEOUT;
  uint8_t *p2header = (uint8_t *)&header;

  while ((waitTime = (getms() - startTs)) < timeout) {
    if (header.packageHeaderMSB != HEADER_MSB) {
      error = HeaderError;
      error_count++;

      if (error_count == 1) {
        ct.crc = *p2header;
      }

      uint8_t *p2header_back = p2header + node_header_size - 1;

      for (int i = 0; i < node_header_size - 1; ++i) {
        p2header[i] = p2header[i + 1];
      }

      ans = read_command(serial, p2header_back, 1, error, timeout - waitTime);

      if (!IS_OK(ans)) {
        return ans;
      }

      ans = RESULT_TIMEOUT;
    } else {
      uint8_t *p3header_back = p2header + node_header_size - 2;

      if (header.packageHeaderLSB != HEADER_LSB) {
        error = HeaderError;
        error_count++;

        for (int i = 0; i < node_header_size - 2; ++i) {
          p2header[i] = p2header[i + 2];
        }

        ans = read_command(serial, p3header_back, 2, error, timeout - waitTime);

        if (!IS_OK(ans)) {
          return ans;
        }

        ans = RESULT_TIMEOUT;
      } else {
        if (error == HeaderError) {
          error = NoError;
        }
//        printf("header:");
//        for(int i=0;i<10;i++)
//          printf(" %x",*(p2header + i));
//        printf("\n");
        if (IS_OK(check_package_header_t(header, error))) {
          parse_ct_packet_t(header, error_count, ct);
          ans = RESULT_OK;
          error = NoError;
          break;
        } else {
//            printf("error:%d\n",error);
//            fflush(stdout);
          p2header[0] = 0x00;
          ans = RESULT_TIMEOUT;
        }

        error_count = 0;
      }
    }
  }

  return ans;
}

bool is_valid_data(uint8_t *data, size_t size) {
  if (data == nullptr || size == 0) {
    return false;
  }

  bool ret = false;

  for (int i = 0; i < size; ++i) {
    if (data[i] != 0x00) {
      ret = true;
      break;
    }
  }

  return ret;
}

result_t check_scan_protocol(Serial *serial, int8_t &protocol,
                             uint32_t timeout) {
  assert(serial);

  if (protocol > -1) {
    return RESULT_OK;
  }

  uint32_t startTs = getms();
  uint32_t waitTime = 0;
  result_t ans = RESULT_TIMEOUT;
  uint8_t scan_double_check = 0;
  uint8_t invalid_scan_count = 0;
  uint8_t iscan_double_check = 0;
  uint8_t invalid_iscan_count = 0;

  if (serial->available() > sizeof(scan_intensity_packet_t)) {
    serial->flushInput();
  }

  while ((waitTime = (getms() - startTs)) < timeout * 2) {
    scan_packet_t scan;
    ct_packet_t ct;
    lidar_error_t error;
    ans = read_response_scan_header_t(serial, scan.header, ct, error,
                                      timeout);


    if (IS_OK(ans)) {
      size_t size =  sizeof(node_package_payload_t) * scan.header.nowPackageNum;
      ans = read_command(serial, (uint8_t *)&scan.payload, size, error,
                         SCAN_DEFAULT_TIMEOUT);

      size_t isize =  sizeof(node_package_intensity_payload_t) *
                      scan.header.nowPackageNum;
      scan_intensity_packet_t iscan;
      iscan.header = scan.header;
      memcpy(&iscan.payload, &scan.payload, size);
      uint8_t *scan_buffer = reinterpret_cast<uint8_t *>(&iscan.payload);
      size_t offset_size = size;
      uint16_t buffer = 0;

      if (IS_OK(ans)) {
        if (checksum_response_scan_packet_t(scan) == scan.header.checkSum) {
          ans = read_command(serial, (uint8_t *)&buffer, 2, error, SCAN_DEFAULT_TIMEOUT);

          if (IS_OK(ans)) {
            if (buffer == 0x55aa) {
              if (is_valid_data((uint8_t *)&scan.payload, size)) {
                scan_double_check++;
                iscan_double_check = 0;
              } else {
                invalid_scan_count++;
              }

              invalid_iscan_count = 0;

              if (scan_double_check > 2 || invalid_scan_count > 5) {
                protocol = 0;
                printf("[YDLIDAR INFO]: intensity: false\n");
                fflush(stdout);
                return ans;
              } else {
                continue;
              }
            } else {
              scan_double_check = 0;
              invalid_scan_count = 0;

              if (scan.header.nowPackageNum == 1) {
                memcpy(scan_buffer + offset_size, &buffer, 1);
                offset_size += 1;
              } else {
                memcpy(scan_buffer + offset_size, &buffer, 2);
                offset_size += 2;
              }
            }
          }
        } else {
          scan_double_check = 0;
          invalid_scan_count = 0;
        }

        if (protocol < 0) {
          if (isize - offset_size > 0) {
            ans = read_command(serial, scan_buffer + offset_size, isize - offset_size,
                               error, SCAN_DEFAULT_TIMEOUT);
          }

          if (IS_OK(ans)) {
            if (checksum_response_scan_intensity_packet_t(iscan) == iscan.header.checkSum) {
              if (iscan.header.nowPackageNum == 1 && offset_size > size) {
                uint8_t buf = 0;
                ans = read_command(serial, (uint8_t *)&buf, 1, error, SCAN_DEFAULT_TIMEOUT);

                if (IS_OK(ans)) {
                  buffer = (buffer << 8 | buf);
                }
              } else {
                ans = read_command(serial, (uint8_t *)&buffer, 2, error, SCAN_DEFAULT_TIMEOUT);
              }

              if (IS_OK(ans)) {
                if (buffer == 0x55aa) {
                  if (is_valid_data((uint8_t *)&iscan.payload, isize)) {
                    iscan_double_check++;
                    scan_double_check = 0;
                  } else {
                    invalid_iscan_count++;
                  }

                  invalid_scan_count = 0;

                  if (iscan_double_check > 2 || invalid_iscan_count > 5) {
                    protocol = 1;
                    printf("[YDLIDAR INFO]: intensity: true\n");
                    fflush(stdout);
                    return ans;
                  } else {
                    continue;
                  }

                } else {
                  iscan_double_check = 0;
                  invalid_iscan_count = 0;
                }
              }
            } else {
              iscan_double_check = 0;
              invalid_iscan_count = 0;
            }
          }
        }

      }
    }
  }

  ans = RESULT_TIMEOUT;
  return ans;
}

result_t read_response_scan_t(Serial *serial, scan_packet_t &scan,
                              ct_packet_t &ct,
                              lidar_error_t &error, uint32_t timeout) {
  assert(serial);
  result_t ans = read_response_scan_header_t(serial, scan.header, ct, error,
                 timeout);

  if (IS_OK(ans)) {
    size_t size =  sizeof(node_package_payload_t) * scan.header.nowPackageNum;
    ans = read_command(serial, (uint8_t *)&scan.payload, size, error,
                       SCAN_DEFAULT_TIMEOUT);

    if (IS_OK(ans)) {
      if (checksum_response_scan_packet_t(scan) != scan.header.checkSum) {
        ans = RESULT_TIMEOUT;
        error = CheckSumError;
      }
    }
  }

  if (!IS_OK(ans)) {
    reset_ct_packet_t(ct);
  }

  return ans;
}

result_t read_response_scan_intensity_t(Serial *serial,
                                        scan_intensity_packet_t &scan, ct_packet_t &ct,
                                        lidar_error_t &error, uint32_t timeout) {
  assert(serial);
  result_t ans = read_response_scan_header_t(serial, scan.header, ct, error,
                 timeout);

  if (IS_OK(ans)) {
    size_t size =  sizeof(node_package_intensity_payload_t) *
                   scan.header.nowPackageNum;
    ans = read_command(serial, (uint8_t *)&scan.payload, size, error,
                       SCAN_DEFAULT_TIMEOUT);

    if (IS_OK(ans)) {
      if (checksum_response_scan_intensity_packet_t(scan) != scan.header.checkSum) {
        ans = RESULT_TIMEOUT;
        error = CheckSumError;
      }
    }
  }

  if (!IS_OK(ans)) {
    reset_ct_packet_t(ct);
  }

  return ans;
}


}  // namespace protocol
}  // namespace ydlidar
