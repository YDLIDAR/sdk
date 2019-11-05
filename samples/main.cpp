
#include "CYdLidar.h"
#include <iostream>
#include <string>
using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

void LidarCtrlFreqCallback(bool isLowerFreq) {
  if (isLowerFreq) { //低频模式
    //设置雷达频率小于6.8Hz

  } else {//高频模式
    //设置雷达频率大于11.7Hz

  }
}

int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____            \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\     \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) |        \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <          \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\      \n");
  printf("\n");
  fflush(stdout);
  std::string port;
  ydlidar::init(argc, argv);
  std::map<std::string, std::string> ports =
    ydlidar::YDlidarDriver::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    it = ports.begin();
    port = it->second;
  } else {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++) {
      printf("%d. %s\n", id, it->first.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::ok()) {
        printf("Please select the lidar port:");
        std::string number;
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
    }
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  CYdLidar laser;
  laser.setSerialPort(port);
  laser.setSerialBaudrate(214285);
  laser.setIntensities(true);//intensity
  laser.setAutoReconnect(true);//hot plug

  //unit: Deg
  laser.setMaxAngle(180);
  laser.setMinAngle(-180);

  //unit: m
  laser.setMinRange(0.1);
  laser.setMaxRange(16.0);


  //set the range of angles that need to be removed.
  //usage: [0, 10, 15,25, 80, 90]
  std::vector<float> ignore_array;
  ignore_array.clear();
  laser.setIgnoreArray(ignore_array);

  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  }

  //开启修正需注册高低频回调函数, 外部调用设定雷达高低频率
  laser.RegisterCtrlFreqCallback(std::bind(LidarCtrlFreqCallback,
                                 std::placeholders::_1));

  //如果要开启进入修正模式并修正, 调用startCorrectionMod函数
  laser.startCorrectionMod();
  //修正中可以通过getCheckStateError函数获取状态信息
  //开启修正模式后, 判断是否修正完成, 调用IsCheckingFinished函数, 返回值是true, 修正完成, 否则,正在修正
  //laser.IsCheckingFinished();
  //修正完成后, 判断修正成功还是失败调用getResult函数, 返回值是true, 修正成功, 否则修正失败
  //laser.getResult();

  while (ret && ydlidar::ok()) {
    bool hardError;
    LaserScan scan;

    if (laser.doProcessSimple(scan, hardError)) {
      if (laser.getCheckFinished() && laser.IsCheckingFinished()) {
        fprintf(stdout, "Scan received[%lu]: %u ranges is [%f]Hz\n",
                scan.system_time_stamp,
                (unsigned int)scan.ranges.size(), 1.0 / scan.config.scan_time);
        fflush(stdout);
      }
    } else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }

    if (laser.IsCheckingFinished()) {//修正完成
      if (laser.getResult()) { //修正成功

      } else {//修正失败

      }
    } else {//查看修正状态
      switch (laser.getCheckStateError()) {
        case CYdLidar::NOERROR:

          break;

        case CYdLidar::FREQUENCYOUT:

          break;

        case CYdLidar::JUMPFREQUENCY:

          break;

        default:
          break;
      }
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}
