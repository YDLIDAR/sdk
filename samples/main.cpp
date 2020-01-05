/*********************************************************************
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
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <regex>
using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);
  ydlidar::init(argc, argv);
  std::string port;
  std::string baud;
  printf("Please enter the lidar serial port or IP:");
  std::cin >> port;
  printf("Please enter the lidar serial baud rate or network port:");
  std::cin >> baud;
  int baudrate = atoi(baud.c_str());
  regex reg("(\\d{1,3}).(\\d{1,3}).(\\d{1,3}).(\\d{1,3})");
  smatch m;
  uint8_t driver_type;

  if (regex_match(port, m, reg)) {
    driver_type = YDLIDAR_TYPE_TCP;
  } else {
    driver_type = YDLIDAR_TYPE_SERIAL;
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  bool isSingleChannel = false;
  bool isTOFLidar = false;
  std::string input_channel;
  std::string input_tof;
  printf("Whether the Lidar is one-way communication[yes/no]:");
  std::cin >> input_channel;
  std::transform(input_channel.begin(), input_channel.end(),
                 input_channel.begin(),
  [](unsigned char c) {
    return std::tolower(c);  // correct
  });

  if (input_channel.find("yes") != std::string::npos) {
    isSingleChannel = true;
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  printf("Whether the Lidar is a TOF Lidar [yes/no]:");
  std::cin >> input_tof;
  std::transform(input_tof.begin(), input_tof.end(),
                 input_tof.begin(),
  [](unsigned char c) {
    return std::tolower(c);  // correct
  });

  if (input_tof.find("yes") != std::string::npos) {
    isTOFLidar = true;
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  std::string input_frequency;

  float frequency = 8.0;

  while (ydlidar::ok() && !isSingleChannel) {
    printf("Please enter the lidar scan frequency[3-15.7]:");
    std::cin >> input_frequency;
    frequency = atof(input_frequency.c_str());

    if (frequency <= 15.7 && frequency >= 3.0) {
      break;
    }

    fprintf(stderr,
            "Invalid scan frequency,The scanning frequency range is 5 to 12 HZ, Please re-enter.\n");
  }

  if (!ydlidar::ok()) {
    return 0;
  }




  CYdLidar laser;
  //<! lidar port
  laser.setSerialPort(port);
  //<! lidar baudrate
  laser.setSerialBaudrate(baudrate);
  //<! lidar connect type
  laser.setDeviceType(driver_type);//network or serial

  //<! fixed angle resolution
  laser.setFixedResolution(false);
  //<! rotate 180
  laser.setReversion(false); //rotate 180
  //<! Counterclockwise
  laser.setInverted(false);//false:顺时针， true: 逆时针
  laser.setAutoReconnect(true);//hot plug
  //<! one-way communication
  laser.setSingleChannel(isSingleChannel);//true: TX8 TX20 S2

  //<! tof lidar
  laser.setLidarType(!isTOFLidar);//tof: TG15, TG30, TG50, TX8 TX20
  //unit: °
  laser.setMaxAngle(180);
  laser.setMinAngle(-180);

  //unit: m
  laser.setMinRange(0.01);
  laser.setMaxRange(64.0);

  //unit: Hz
  laser.setScanFrequency(frequency);
  std::vector<float> ignore_array;
  ignore_array.clear();
  laser.setIgnoreArray(ignore_array);

  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  }

  while (ret && ydlidar::ok()) {
    bool hardError;
    LaserScan scan;

    if (laser.doProcessSimple(scan, hardError)) {
      fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
              scan.stamp,
              (unsigned int)scan.points.size(), 1.0 / scan.config.scan_time);
      fflush(stdout);

      for (int i = 0; i < scan.points.size(); i++) {
        LaserPoint point = scan.points[i];
//        point.angle;//当前角度弧度
//        point.range;//当前距离米
      }
    } else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}
