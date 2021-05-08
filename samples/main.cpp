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
#include "LogModule.h"
#include "timer.h"
#include <pthread.h>
using namespace std;
using namespace ydlidar;
#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif


int main(int argc, char *argv[]) {

  //  pthread_t th1;

    ydlidar::init(argc, argv);

    std::string port;
    std::string baudrate;
    std::string serial_number;
    std::string input_frequency;
    int baud = 115200;
    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
    printf("\n");
    fflush(stdout);
    float frequency = 6.0;

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
    laser.setScanFrequency(frequency);
    laser.setFixedResolution(false);
    laser.setReversion(false);
    laser.setAutoReconnect(false);
    laser.setGlassNoise(true);
    laser.setSunNoise(true);
    laser.setAbnormalCheckCount(8);
    //不带型号强度的雷达
      laser.setIntensity(0);
    //  laser.setSerialBaudrate(115200);
    //带信号强度的雷达
//    laser.setSerialBaudrate(153600);
//    laser.setIntensity(1);


    bool ret = laser.initialize();

    if (ret) {

        ret &= laser.turnOn();
    }else {
        char error[100];
         sprintf(error,"[YDLIDAR ERROR]: %s",
               ydlidar::protocol::DescribeError(laser.getDriverError()));
         LOG_ERROR(error,"");
         printf("%s\n",error);
        fflush(stdout);
    }

    LaserScan scan;
    LaserScanMsg scan_for_calibrate;
    bool getSN = false;
    while (ret && ydlidar::ok()) {

        bool hardError;
        scan.data.clear();
        if (laser.doProcessSimple(scan_for_calibrate,scan, hardError)) {

            if(laser.getdevice_info_status()){
                LidarVersion _version;
                memset(&_version, 0, sizeof(LidarVersion));
                getSN = laser.GetLidarVersion(_version);
                if(getSN){
                    printf("LiDAR HW Version: %d, Fireware Version: %u.%u.%u,Custom Version: %u.%u", _version.hardware,_version.fire_major,_version.fire_minor,
                           _version.fire_patch,_version.soft_major, _version.soft_minor);

                    printf("\n");
                    fflush(stdout);
                }
            }

            if (scan.lidar_scan_frequency > 0) {
                fprintf(stdout, "Scan received[%llu] [%f]: %u ranges in %f HZ, lidar frequency[%f Hz]\n",
                        impl::getHDTimer(),scan.config.scan_time,
                        (unsigned int)scan.data.size(), 1000.0 / scan.config.scan_time,
                        scan.lidar_scan_frequency);
            } else {
                fprintf(stdout, "Scan received[%llu] [%f]: %u ranges in %f HZ\n",
                        impl::getHDTimer(),scan.config.scan_time,
                        (unsigned int)scan.data.size(), 1000.0 / scan.config.scan_time);
            }

            fflush(stdout);
        } else {
            if (laser.getDriverError() != NoError) {
                char error[100];
                 sprintf(error,"[YDLIDAR ERROR]: %s",
                       ydlidar::protocol::DescribeError(laser.getDriverError()));
                 LOG_ERROR(error,"");
                 printf("%s\n",error);
                fflush(stdout);
            }
        }
    }

    laser.turnOff();
    laser.disconnecting();
    return 0;
}
