
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
  laser.setFixedResolution(false);
  laser.setReversion(false);
  laser.setAutoReconnect(true);
  laser.setGlassNoise(true);
  laser.setSunNoise(true);
  laser.setRobotLidarDifference(0);//机器零度和雷达零度理论偏差
  laser.setStartAngleOffset(true);//开启整机零位修正
  bool ret = laser.initialize();

  while (ret && ydlidar::ok()) {
    bool hardError;
    LaserScan scan;

    //雷达频率等于扫描时间的倒数
    if (laser.doProcessSimple(scan, hardError)) {
      float lidar_scan_frequency = 1.0 / scan.config.scan_time;
      fprintf(stdout, "Scan received: %u ranges in %f HZ\n",
              (unsigned int)scan.data.size(), lidar_scan_frequency);
      fflush(stdout);
    }

    laser.isAngleOffetCorrected();//判断当前整机零位是否已经修正
    laser.getStartAngleOffset();//判断当前是否正在修正零位中
    laser.getAngleOffset();//获取修正零位值
  }


  laser.turnOff();
  laser.disconnecting();

  return 0;


}
