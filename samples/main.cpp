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
using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

#define DEFAULT_TIMEOUT 2000

result_t initRecvPort(serial::Serial **_serial,std::string port_name,int baudrate){
    if (!(*_serial)) {
        *_serial = new serial::Serial(port_name, baudrate,
                                     serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
    }

    if((*_serial)->isOpen()){
        return RESULT_OK;
    }
    if (!(*_serial)->open()) {
        return RESULT_FAIL;
    }

    return  RESULT_OK;
}

int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);
  std::string port;
  std::string recv_port;
  ydlidar::init(argc, argv);

  std::map<std::string, std::string> ports =
    ydlidar::YDlidarDriver::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  int id = 0;

  for (it = ports.begin(); it != ports.end(); it++) {
      printf("%d. %s\n", id, it->first.c_str());
      id++;
  }

  std::string number;
  bool haveRecvPort = false;
  bool isTwoPort = false;
  if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
  } else {
      while (ydlidar::ok()) {
          printf("Please select the lidar port:");

          std::cin >> number;

          if ((size_t)atoi(number.c_str()) >= ports.size()) {
              continue;
          }

          it = ports.begin();
          id = atoi(number.c_str());

          while (id) {
              id--;
              it++;
          }

          port = it->second;
          break;
      }

      if(isTwoPort){
          std::string  input_port;
          printf("The current number of serial ports is greater than 2, whether to select the receiver port[yes/no]:");
          std::cin >> input_port;
          std::transform(input_port.begin(), input_port.end(),
                         input_port.begin(),
          [](unsigned char c) {
            return std::tolower(c);  // correct
          });

          if (input_port.find("yes") != std::string::npos) {
            haveRecvPort = true;
          }

          if(haveRecvPort && ports.size() >= 2){
              while (ydlidar::ok()) {
                  printf("Please select the recv lidar port:");
                  std::string number2;
                  std::cin >> number2;

                  if ((size_t)atoi(number2.c_str()) >= ports.size()) {
                      continue;
                  }

                  it = ports.begin();
                  id = atoi(number2.c_str());

                  if(number == number2){
                      continue;
                  }

                  while (id) {
                      id--;
                      it++;
                  }

                  recv_port = it->second;
                  break;
              }
          }
      }
  }

  int baudrate = 230400;
  std::map<int, int> baudrateList;
  baudrateList[0] = 115200;
  baudrateList[1] = 128000;
  baudrateList[2] = 153600;
  baudrateList[3] = 230400;
  baudrateList[4] = 512000;
  baudrateList[5] = 921600;

  printf("Baudrate:\n");

  for (std::map<int, int>::iterator it = baudrateList.begin();
       it != baudrateList.end(); it++) {
    printf("%d. %d\n", it->first, it->second);
  }

  while (ydlidar::ok()) {
    printf("Please select the lidar baudrate:");
    std::string number;
    std::cin >> number;

    if ((size_t)atoi(number.c_str()) > baudrateList.size()) {
      continue;
    }

    baudrate = baudrateList[atoi(number.c_str())];
    break;
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  int recv_baudrate = 921600;
  while (isTwoPort && haveRecvPort && ydlidar::ok()) {
    printf("Please select the recv lidar baudrate:");
    std::string number;
    std::cin >> number;

    if ((size_t)atoi(number.c_str()) > baudrateList.size()) {
      continue;
    }

    recv_baudrate = baudrateList[atoi(number.c_str())];
    break;
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

//  printf("Whether the Lidar is a TOF Lidar [yes/no]:");
//  std::cin >> input_tof;
//  std::transform(input_tof.begin(), input_tof.end(),
//                 input_tof.begin(),
//  [](unsigned char c) {
//    return std::tolower(c);  // correct
//  });

//  if (input_tof.find("yes") != std::string::npos) {
//    isTOFLidar = true;
//  }

//  if (!ydlidar::ok()) {
//    return 0;
//  }

  std::string input_frequency;

  float frequency = 8.0;
  if (!ydlidar::ok()) {
    return 0;
  }

  serial::Serial *_serial = nullptr;
  if(haveRecvPort){
     result_t  ret = initRecvPort(&_serial,recv_port,recv_baudrate);
     if(!IS_OK(ret)){
         fprintf(stderr,"Failed to initialize the receiving port!\n");
         fflush(stderr);
         return -1;
     }
  }

  CYdLidar laser;
  //<! lidar port
  laser.setSerialPort(port);
  //<! lidar baudrate
  laser.setSerialBaudrate(baudrate);

  //<! fixed angle resolution
  laser.setFixedResolution(false);
  //<! rotate 180
  laser.setReversion(false); //rotate 180
  //<! Counterclockwise
  laser.setInverted(false);//ccw
  laser.setAutoReconnect(true);//hot plug
  //<! one-way communication
  laser.setSingleChannel(isSingleChannel);

  //<! tof lidar
  laser.setLidarType(isTOFLidar ? TYPE_TOF : TYPE_TRIANGLE);
  //unit: °
  laser.setMaxAngle(180);
  laser.setMinAngle(-180);

  //unit: m
  laser.setMinRange(30);
  laser.setMaxRange(1000);

  //unit: Hz
  laser.setScanFrequency(frequency);
  std::vector<float> ignore_array;
  ignore_array.clear();
  laser.setIgnoreArray(ignore_array);
  bool isIntensity = true;
  laser.setIntensity(isIntensity);

  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  }

  result_t  isOk = RESULT_OK;
  while (ret && ydlidar::ok()) {
    bool hardError;
    LaserScan scan;
    uint8_t *header = reinterpret_cast<uint8_t *>(&scan);
    size_t size = sizeof(scan);

    if (laser.doProcessSimple(scan, hardError)) {
      fprintf(stdout, "Scan received[%lu]: %u ranges is [%f]Hz\n",
              scan.stamp,
              (unsigned int)scan.points.size(), 1.0 / scan.config.scan_time);
      fflush(stdout);
      if(haveRecvPort){
          isOk = _serial->writeData(header,size);
          if(!IS_OK(isOk)){
              fprintf(stderr, "Failed to send data to the receiving port\n");
              fflush(stderr);
          }
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
