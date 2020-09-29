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
#include <iostream>
#include <string>
#include <memory>
using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

int main(int argc, char *argv[]) {

  ydlidar::init(argc, argv);

  std::string port;
  std::string baudrate;
  int baud = 115200;
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);

  std::map<std::string, std::string> lidars = YDlidarDriver::lidarPortList();

  if (lidars.size() == 1) {
    std::map<string, string>::iterator iter = lidars.begin();
    port = iter->second;
  } else {
    printf("Please enter the lidar serial port:");
    std::cin >> port;
    printf("Please enter the lidar serial baud rate:");
    std::cin >> baudrate;
    baud = atoi(baudrate.c_str());
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  CYdLidar laser;
  laser.setSerialPort(port);
  laser.setSerialBaudrate(baud);
  laser.setScanFrequency(6.0);
  laser.setFixedResolution(false);
  laser.setReversion(false);
  laser.setAutoReconnect(true);
  laser.setGlassNoise(true);
  laser.setSunNoise(true);
  bool ret = laser.initialize();
  LaserScan scan;

  while (ret && ydlidar::ok()) {
    bool hardError;
    scan.data.clear();

    if (laser.doProcessSimple(scan, hardError)) {
      fprintf(stdout, "Scan received: %u ranges in %f HZ\n",
              (unsigned int)scan.data.size(), 1.0 / scan.config.scan_time);
      fflush(stdout);
    } else {
      printf("[YDLIDAR ERROR]: %s\n",
             ydlidar::protocol::DescribeError(laser.getDriverError()));
      fflush(stdout);
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;


}
