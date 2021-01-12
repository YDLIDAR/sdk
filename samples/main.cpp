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
#include <unistd.h>
using namespace std;
using namespace ydlidar;
#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

int main(int argc, char *argv[]) {

    ydlidar::init(argc, argv);

    std::string port;
    std::string baudrate;
    std::string serial_number;
    std::string input_frequency;
    int baud = 153600;
    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
    printf("\n");
    fflush(stdout);
    float frequency = 6.0;
    int number = 1;
//    if(argc == 3){
//        if(string(argv[1]).find("pwm")!= string::npos){
//            if(string(argv[2]).find("0") != string::npos)
//                number = 0;
//        }
//    }

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
//        printf("Please enter the pwd serial number [0 or 1]:");
//        std::cin >> serial_number;
//        number = atoi(serial_number.c_str());
      
//        while (true) {
//          printf("Please enter the lidar scan frequency[5-12]:");
//          std::cin >> input_frequency;
//          frequency = atof(input_frequency.c_str());
      
//          if (frequency <= 12 && frequency >= 5.0) {
//            break;
//          }
      
//          fprintf(stderr,
//                  "Invalid scan frequency,The scanning frequency range is 5 to 12 HZ, Please re-enter.\n");
//        }
       
        
    }

    if (!ydlidar::ok()) {
        return 0;
    }

    CYdLidar laser;
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baud);
    laser.setScanFrequency(frequency);
    laser.setFixedResolution(false);
    laser.setReversion(false);
    laser.setAutoReconnect(true);
    laser.setGlassNoise(true);
    laser.setSunNoise(true);
    laser.setAbnormalCheckCount(8);
    //不带型号强度的雷达
    //  laser.setIntensity(0);
    //  laser.setSerialBaudrate(115200);
    //带信号强度的雷达
    laser.setSerialBaudrate(153600);
    laser.setIntensity(1);
    laser.initPwdPath(number);

    CYdLidar::PIDError error = laser.initPIDParams();
    string error_str =  laser.getErrorString(error);
    if (error != NoError){
        printf("init pid config failed!,error:%s",error_str.c_str());
        fflush(stdout);
        return 1;
    }
    bool ret = laser.initialize();

    if (ret) {

        ret &= laser.turnOn();
    }

    LaserScan scan;
    bool getSN = false;
    while (ret && ydlidar::ok()) {
        bool hardError;
        scan.data.clear();

        if (laser.doProcessSimple(scan, hardError)) {

            if(!getSN){
                LidarVersion _version;
                memset(&_version, 0, sizeof(LidarVersion));
                getSN = laser.GetLidarVersion(_version);
                if(getSN){
                    printf("LiDAR HW Version: %d, Fireware Version: %u.%u,FW Version: %u.%u.%u, SN: ", _version.hardware,_version.fire_major,_version.fire_minor,
                           _version.soft_major, _version.soft_minor, _version.soft_patch);

                    for (int i = 0; i < 32; i++) {
                        printf("%01X", _version.sn[i]);
                    }

                    printf("\n");
                    fflush(stdout);
                }
            }

            if (scan.lidar_scan_frequency > 0) {
                fprintf(stdout, "Scan received: %u ranges in %f HZ, lidar frequency[%f Hz]\n",
                        (unsigned int)scan.data.size(), 1000.0 / scan.config.scan_time,
                        scan.lidar_scan_frequency);
            } else {
                fprintf(stdout, "Scan received: %u ranges in %f HZ\n",
                        (unsigned int)scan.data.size(), 1000.0 / scan.config.scan_time);
            }

            fflush(stdout);
        } else {
            if (laser.getDriverError() != NoError) {
                printf("[YDLIDAR ERROR]: %s\n",
                       ydlidar::protocol::DescribeError(laser.getDriverError()));
                fflush(stdout);
            }
        }
    }

    laser.turnOff();
    laser.disconnecting();

    return 0;


}
