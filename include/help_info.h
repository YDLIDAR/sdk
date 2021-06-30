/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, EAIBOT, Inc.
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
#pragma once

#include <ydlidar_protocol.h>

namespace ydlidar {

inline bool isValidValue(uint8_t value) {
  if (value & 0x80) {
    return false;
  }

  return true;
}
inline bool isValidModel(uint8_t value) {
  if (value == 4 || value == 5 ||
      value == 7 || value == 9 ||
      value == 12) {
    return true;
  }

  return false;
}
inline bool isValidYear(uint8_t value) {
  if (value > 2 && value < 12) {
    return true;
  }

  return false;
}

inline bool isValidMoth(uint8_t value) {
  if (value >= 0 && value <= 12) {
    return true;
  }

  return false;
}

inline bool isValidDate(uint8_t value) {
  if (value >= 0 && value <= 31) {
    return true;
  }

  return false;
}

//inline bool isVersionValid(const LaserDebug &info) {
//  bool ret = false;

//  if (isValidValue(info.W3F4CusMajor_W4F0CusMinor) &&
//      isValidValue(info.W4F3Model_W3F0DebugInfTranVer) &&
//      isValidValue(info.W3F4HardwareVer_W4F0FirewareMajor) &&
//      isValidValue(info.W3F4BoradHardVer_W4F0Moth)) {
//    ret = true;
//  }

//  return ret;
//}

//inline bool isSerialNumbValid(const LaserDebug &info) {
//  bool ret = false;

//  if (isValidValue(info.W2F5Output2K4K5K_W5F0Date) &&
//      isValidValue(info.W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear) &&
//      isValidValue(info.W7F0SnNumH) &&
//      isValidValue(info.W7F0SnNumH)) {
//    ret = true;
//  }

//  return ret;
//}

inline void parsePackageNode(const node_info &node, LaserDebug &info) {
  switch (node.index) {
    case 0://scan frequency

      break;

    case 1://TranIndex_W2F5CusVerMajor_W5F0CusVerMinor;
      info.TranIndex_W2F5CusVerMajor_W5F0CusVerMinor = node.debugInfo;
      break;

    case 2://TranIndex_W7F0ProtoMode
      info.TranIndex_W7F0ProtoMode = node.debugInfo;
      break;

    case 3://TranIndex_W7F0Health
      info.TranIndex_W7F0Health = node.debugInfo;

      break;

    case 4://TranIndex_W3F4HardwareVer_W4F0FirewareMajor
      info.TranIndex_W3F4HardwareVer_W4F0FirewareMajor = node.debugInfo;

      break;

    case 5://TranIndex_W7F0FirewareMinor
      info.TranIndex_W7F0FirewareMinor = node.debugInfo;

      break;

    case 6://TranIndex_W3F4MeasureMode_W4F0WorkMode
      info.TranIndex_W3F4MeasureMode_W4F0WorkMode = node.debugInfo;
      break;

    case 7://TranIndex_W3F4MasterMode_W4F0PlatFormMode
      info.TranIndex_W3F4MasterMode_W4F0PlatFormMode =
        node.debugInfo;
      break;

    case 8://TranIndex_W3F4FactorVer_W4F0LaserMode
      info.TranIndex_W3F4FactorVer_W4F0LaserMode = node.debugInfo;
      break;

    case 9://TranIndex_W7F0PDVal
      info.TranIndex_W7F0PDVal = node.debugInfo;

      break;

    case 10://TranIndex_W7F0LDVal
      info.TranIndex_W7F0LDVal = node.debugInfo;

      break;

    case 11://TranIndex_W7F0REFVal
      info.TranIndex_W7F0REFVal = node.debugInfo;

    case 12://TranIndex_W5F2Year_W2F0SN1
      info.TranIndex_W5F2Year_W2F0SN1 = node.debugInfo;
      break;
    case 13://TranIndex_W4F3Month_W3F0SN2
      info.TranIndex_W4F3Month_W3F0SN2 = node.debugInfo;
      break;
    case 14://TranIndex_W5F2Day_W2F0SN3
      info.TranIndex_W5F2Day_W2F0SN3 = node.debugInfo;
      break;
    case 15://TranIndex_W7F0SN4
      info.TranIndex_W7F0SN4 = node.debugInfo;
      break;
    case 16://TranIndex_W7F0SN5
      info.TranIndex_W7F0SN5 = node.debugInfo;
      break;

    default:
      break;
  }

  if (info.MaxDebugIndex > node.index) {
    info.TranIndex_W2F5CusVerMajor_W5F0CusVerMinor = 0xff;
  }

  if (static_cast<int>(node.index) > info.MaxDebugIndex && node.index < 100) {
    info.MaxDebugIndex = static_cast<int>(node.index);
  }
}

inline bool ParseLaserDebugInfo(const LaserDebug &info, device_info &value) {
  bool ret = false;
  uint8_t CustomVerMajor = (static_cast<uint8_t>
                            (info.TranIndex_W2F5CusVerMajor_W5F0CusVerMinor) >> 5);
  uint8_t CustomVerMinor = static_cast<uint8_t>
                           (info.TranIndex_W2F5CusVerMajor_W5F0CusVerMinor) & 0x1F;
//  uint8_t lidarmodel = (static_cast<uint8_t>(info.W4F3Model_W3F0DebugInfTranVer)
//                        >> 3);
//  uint8_t hardwareVer = static_cast<uint8_t>
//                        (info.W3F4HardwareVer_W4F0FirewareMajor) >> 4;
//  uint8_t Moth = static_cast<uint8_t>(info.W3F4BoradHardVer_W4F0Moth) & 0x0F;

//  uint8_t Date = static_cast<uint8_t>(info.W2F5Output2K4K5K_W5F0Date) & 0x1F;
//  uint8_t Year = static_cast<uint8_t>
//                 (info.W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear) & 0x0F;
//  uint16_t Number = ((static_cast<uint8_t>(info.W7F0SnNumH) << 7) |
//                     static_cast<uint8_t>(info.W7F0SnNumL));

//  if (isVersionValid(info) && info.MaxDebugIndex > 0 && isValidYear(Year) &&
//      isValidModel(lidarmodel)) {

//    if (isSerialNumbValid(info) && info.MaxDebugIndex > 8 && isValidMoth(Moth) &&
//        isValidDate(Date)) {
//      value.firmware_version = (CustomVerMajor << 8 | CustomVerMinor);
//      value.hardware_version = hardwareVer;
//      value.model = lidarmodel;
//      uint32_t year = Year + 2015;
//      sprintf(reinterpret_cast<char *>(value.serialnum), "%04d", year);
//      sprintf(reinterpret_cast<char *>(value.serialnum + 4), "%02d", Moth);
//      sprintf(reinterpret_cast<char *>(value.serialnum + 6), "%02d", Date);
//      sprintf(reinterpret_cast<char *>(value.serialnum + 8), "%08d", Number);

//      for (int i = 0; i < 16; i++) {
//        value.serialnum[i] -= 48;
//      }

//      ret = true;
//    }
//  }

//  return ret;
}

}

