
#include "CYdLidar.h"
#include "common.h"
#include <map>
#include<numeric>
#include "angles.h"
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
//#include "impl/unix/list_ports_linux.cpp"

using namespace std;
using namespace ydlidar;
using namespace impl;

bool path_exists(const string &path);

bool
path_exists(const string &path) {
  struct stat sb;

  if (stat(path.c_str(), &sb) == 0) {
    return true;
  }

  return false;
}

int get_int_size(const int &dest);
int get_int_size(const int &dest){
    if(dest <10){
        return 1;
    }else if(dest <100 && dest >= 10) {
        return 2;
    }else if (dest < 1000 && dest >= 100){
        return 3;
    }else if(dest <10000 && dest >= 1000){
        return 4;
    }else if(dest < 100000 && dest >= 10000){
        return 5;
    }else{
        return 5;
    }
}
/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(0) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 115200;
  m_FixedResolution   = false;
  m_Reversion         = false;
  m_AutoReconnect     = false;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 8.0;
  m_MinRange          = 0.08;
  m_AbnormalCheckCount = 8;
  isScanning          = false;
  isConnected         = false;
  m_GlassNoise        = true;
  m_SunNoise          = true;
  m_OffsetTime        = 0.0;
  point_interval_time = 1e9 / 3000; //单位为纳秒
  package_transfer_time = 0;
  last_node_time      = getTime();
  fixed_size          = 500;
  sample_rate         = 3;
  m_ScanFrequency     = 6.0;
  frequency_offset    = 0.0;
  zero_offset_angle   = 0.0;
  single_channel      = false;
  m_Intensity         = -1;
  default_mode_duty   = 6000;
  node_counts = 720;
  each_angle          = 0.5;
  m_device_info_status           = false;
  m_IgnoreArray.clear();
  memset(&m_LidarVersion, 0, sizeof(LidarVersion));
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();
}

void CYdLidar::disconnecting() {
  if (lidarPtr) {
    lidarPtr->disconnect();
    printf("[YDLIDAR INFO] Now YDLIDAR disconnecting ......\n");
    fflush(stdout);
    delete lidarPtr;
    lidarPtr    = 0;
    isConnected = false;
  }
}

int CYdLidar::getFixedSize() const {
  return fixed_size;
}

bool CYdLidar::GetLidarVersion(LidarVersion &version) {
    bool sn_ok = m_device_info_status;
    if(sn_ok){
        memcpy(&version,&m_LidarVersion,sizeof(version));
    }

    return sn_ok;
}

lidar_error_t CYdLidar::getDriverError() const {
  if (lidarPtr) {
    return lidarPtr->getDriverError();
  }

  return UnknownError;
}

/*-------------------------------------------------------------
                        doProcessSimple
-------------------------------------------------------------*/

bool  CYdLidar::doProcessSimple(LaserScanMsg &outscan, LaserScan& msg, bool &hardwareError) {
    hardwareError			= false;
  if (!checkHardware()) {
    hardwareError = true;
    return false;
    }

  size_t count = 2048;
  size_t all_nodes_counts = node_counts;

  //  wait Scan data:
  uint64_t tim_scan_start = getHDTimer();
  laser_packages.points.clear();
  result_t op_result =  lidarPtr->grabScanData(&laser_packages, 500);
  count = laser_packages.points.size();
  fflush(stdout);
  if(count == 0){
      return false;
  }

  uint64_t tim_scan_end = getHDTimer();
  outscan.scan_cnts = laser_packages.points.size();

    // Fill in scan data:
  if (IS_OK(op_result))
    {
    op_result = lidarPtr->ascendScanData(&laser_packages, count);
    uint64_t max_time =laser_packages.points[0].stamp ;
    uint64_t min_time = laser_packages.points[0].stamp;
    float scan_time = (point_interval_time * (count - 1)) / 1000000.0;

    if (count == 1) {
      scan_time = point_interval_time / 1000000.0;
    }

    tim_scan_end -= package_transfer_time / 1000000.0;
    tim_scan_end += m_OffsetTime * 1e3;
    tim_scan_end -= point_interval_time / 1000000.0;
    tim_scan_start = tim_scan_end -  scan_time ;
    last_node_time = tim_scan_end;
    msg.data.clear();
    msg.config.scan_time =
      scan_time; //static_cast<float>(1.0 * scan_time / 1e9);
    msg.config.fixed_size = fixed_size;
    msg.config.min_angle = angles::from_degrees(m_MinAngle);
    msg.config.max_angle = angles::from_degrees(m_MaxAngle);

    if (count > 1) {
      msg.config.time_increment = msg.config.scan_time / (double)(
                                         count - 1);
    } else {
      msg.config.time_increment = msg.config.scan_time;
    }

    msg.system_time_stamp = tim_scan_end;
    msg.config.min_range = m_MinRange;
    msg.config.max_range = m_MaxRange;
    if (IS_OK(op_result)){
      if (!m_FixedResolution){
        all_nodes_counts = count;
      } else {
        all_nodes_counts = node_counts;
      }
    each_angle = 360.0/all_nodes_counts;
    LaserPoint *points = new LaserPoint[all_nodes_counts];
    unsigned int i = 0;
    msg.data.clear();
    for( ; i < count; i++) {
      if (laser_packages.points[i].range != 0) {
        float angle = laser_packages.points[i].angle;
        LaserPoint point;
        point.angle = angle / 180 * M_PI;
        point.range = laser_packages.points[i].range/1000.f;
        // std::cout<<"angle : "<< point.angle<<" range is: "<<point.range<<std::endl;
        msg.data.push_back(point);
        if(m_Reversion){
          std::cout<<"m_Reversion is true"<<std::endl;
          angle=angle+180;
          if(angle>=360){
            angle=angle-360;
          }
          laser_packages.points[i].angle = angle;
        }
        unsigned int inter =(unsigned int)(angle / each_angle);
        //int inter =(int)( angle / each_angle );
        float angle_pre = angle - inter * each_angle;
        float angle_next = (inter+1) * each_angle - angle;
        if (angle_pre < angle_next) {
          if(inter < all_nodes_counts)
            points[inter]=laser_packages.points[i];
        } else {
          if (inter < all_nodes_counts -1)
            points[inter + 1]=laser_packages.points[i];
        }

        if(laser_packages.points[i].stamp > max_time) {
          max_time = laser_packages.points[i].stamp;
        }

        if(laser_packages.points[i].stamp < min_time) {
          min_time = laser_packages.points[i].stamp;
        }
      }

    }

    if (m_MaxAngle< m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }


    tim_scan_start = min_time;
    double scan_time = max_time - min_time;

    int counts = all_nodes_counts*((m_MaxAngle-m_MinAngle)/360.0f);
    int angle_start = 180+m_MinAngle;
    int node_start = all_nodes_counts*(angle_start/360.0f);
    //LOG(INFO)<<"angle_start is: "<<angle_start<<"  node_start is: "<<node_start<<"  all_nodes_counts :"<<all_nodes_counts;
    scan_msg.ranges.resize(counts);
    scan_msg.intensities.resize(counts);
    scan_msg.noise_flags.resize(counts);
    scan_msg.point_time.resize(counts);
    float range = 0.0;
    float intensity = 0.0;
    bool noise_flag = false;
    int index = 0;

    for (size_t i = 0; i < all_nodes_counts; i++) {
      range = (float)points[i].range/1000.f;
      intensity = (float)(points[i].intensity);
      uint16_t disturb_value = points[i].interference_sign;
      noise_flag = (disturb_value == 2 || disturb_value == 3); //2-sunnoise 3-glassnoise
      // uint16_t disturb_value = angle_compensate_nodes[i].distance_q2 & 3;
      // if(disturb_value == 2 || disturb_value == 3) { //2-sunnoise 3-glassnoise
      //   range = 0.0;
      // }
      uint64_t stamp = points[i].stamp;
      if (i<all_nodes_counts/2) {
        index = all_nodes_counts/2-1-i;
      } else {
        index =all_nodes_counts-1-(i-all_nodes_counts/2);
      }
      if (m_IgnoreArray.size() != 0) {
        float angle = points[i].angle;
        if (angle>180) {
          angle=360-angle;
        } else {
          angle=-angle;
        }

        for (uint16_t j = 0; j < m_IgnoreArray.size();j = j+2) {
          if ((m_IgnoreArray[j] < angle) && (angle <= m_IgnoreArray[j+1])) {
            range = 0.0;
            break;
          }
        }
      }

      if (range > m_MaxRange|| range < m_MinRange) {
        range = 0.0;
      }

      int pos = index - node_start ;
        if (0<= pos && pos < counts) {
          scan_msg.ranges[pos] =  range;
          scan_msg.intensities[pos] = intensity;
          scan_msg.noise_flags[pos] = noise_flag;
          scan_msg.point_time[pos] = stamp;
        }
      }

      scan_msg.system_time_stamp = tim_scan_start;
      scan_msg.self_time_stamp = tim_scan_start;
      scan_msg.end_time_stamp = max_time;
      scan_msg.config.min_angle = angles::from_degrees(m_MinAngle);
      scan_msg.config.max_angle = angles::from_degrees(m_MaxAngle);
      scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) / (double)counts;
      scan_msg.config.time_increment = scan_time / (double)counts *1e-9f;
      scan_msg.config.min_range = m_MinRange;
      scan_msg.config.max_range = m_MaxRange;
      outscan = scan_msg;
      msg.config.max_range = m_MaxRange;
      msg.config.min_range = m_MinRange;
      delete []points;
      return true;
        }

  } else {
    if (op_result==RESULT_FAIL) {
            // Error? Retry connection
            //this->disconnect();
        }
    }
    return false;

}

bool CYdLidar::checkHealth(/*const */ct_packet_t &info) {
  bool ret = true;
  if (IS_OK(ydlidar::protocol::check_ct_packet_t(info))) {
    lidar_error_t err = ydlidar::protocol::convert_ct_packet_to_error(info);

    if (err != NoError) {
      ret = false;
    }
  }

  return ret;
}

/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  if (isScanning && lidarPtr->isScanning()) {
    return true;
  }

  uint32_t startTs = getms();
  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      printf("[CYdLidar] Failed to start scan mode: %x, %s\n", op_result,
             ydlidar::protocol::DescribeError(lidarPtr->getDriverError()));
      fflush(stdout);
      lidarPtr->stop();
      isScanning = false;
      return false;
    }
  }

  point_interval_time = lidarPtr->getPointIntervalTime();
  package_transfer_time = lidarPtr->getPackageTransferTime();

  if (checkLidarAbnormal()) {
    printf("[CYdLidar][%fs] Failed to turn on the Lidar, because %s.\n",
           (getms() - startTs) / 1000.0,
           ydlidar::protocol::DescribeError(lidarPtr->getDriverError()));
    fflush(stdout);
    lidarPtr->stop();
    isScanning = false;
    return false;
  }

  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO][%fs] Now YDLIDAR is scanning ......\n",
         (getms() - startTs) / 1000.0);
  fflush(stdout);
  return true;
}




/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff() {
  if (lidarPtr) {
    lidarPtr->stop();
    printf("[YDLIDAR INFO] Now YDLIDAR Stop Scan ......\n");
    fflush(stdout);
  }

  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
    fflush(stdout);
  }

  isScanning = false;
  m_device_info_status = false;
  return true;
}


/*-------------------------------------------------------------
            checkLidarAbnormal
-------------------------------------------------------------*/
bool CYdLidar::checkLidarAbnormal() {
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;

  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 1) {
      delay(check_abnormal_count * 50);
    }

    LaserFan packages;
    packages.points.clear();
    op_result =  lidarPtr->grabScanData(&packages, 500);

    if (IS_OK(op_result)) {
      return !IS_OK(op_result);
    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
}

bool CYdLidar::getDeviceHealth(uint32_t timeout) {
  if (!lidarPtr) {
    return false;
  }

  uint32_t startTs = getms();
  result_t op_result;
  device_health healthinfo;
  op_result = lidarPtr->getHealth(healthinfo, timeout);

  if (IS_OK(op_result)) {
    printf("[YDLIDAR][%fs]:Lidar running correctly ! The health status: %s\n",
           (getms() - startTs) / 1000.0,
           (int)healthinfo.status == 0 ? "good" : "bad");
    fflush(stdout);

    if (healthinfo.status == 2) {
//      fprintf(stderr,
//              "Error, YDLIDAR internal error detected. Please reboot the device to retry.\n");
//      fflush(stderr);
      return false;
    } else {
      return true;
    }

  } else {
//    fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
    return false;
  }
}

bool CYdLidar::getDeviceInfo(uint32_t timeout) {
  if (!lidarPtr) {
    return false;
  }

  uint32_t startTs = getms();
  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo, timeout);

  if (!IS_OK(op_result)) {
//    printf("get Device Information Error: %s\n",
//           ydlidar::protocol::DescribeError(lidarPtr->getDriverError()));
//    fflush(stdout);
    return false;
  }

  std::string model = format("S2Pro[%d]", devinfo.model);
  uint8_t Major = (uint8_t)(devinfo.firmware_version >> 8);
  uint8_t Minjor = (uint8_t)(devinfo.firmware_version & 0xff);
  printf("[YDLIDAR][%fs] Device Info:\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n",
         //"Serial: ",
         (getms() - startTs) / 1000.0,
         Major,
         Minjor,
         (unsigned int)devinfo.hardware_version,
         model.c_str());

//  for (int i = 0; i < 16; i++) {
//    printf("%01X", devinfo.serialnum[i] & 0xff);
//  }

//  printf("\n");
  fflush(stdout);
  m_LidarVersion.hardware = devinfo.hardware_version;
  m_LidarVersion.soft_major = Major;
  m_LidarVersion.soft_minor = Minjor / 10;
  memcpy(&m_LidarVersion.sn[0], &devinfo.serialnum[0], 16);
  checkScanFrequency();
  //checkZeroOffsetAngle();
  printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n", sample_rate);
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency() {
  uint32_t startTs = getms();
  float frequency = 6.4f;
  scan_frequency_t _scan_frequency;
  float hz = 0.f;
  m_ScanFrequency += frequency_offset;
  result_t ans = lidarPtr->getScanFrequency(_scan_frequency) ;

  if (IS_OK(ans)) {
    frequency = _scan_frequency.frequency / 100.f;
    hz = m_ScanFrequency - frequency;

    if (hz > 0) {
      while (hz > 0.95) {
        lidarPtr->setScanFrequencyAdd(_scan_frequency);
        hz = hz - 1.0;
      }

      while (hz > 0.09) {
        lidarPtr->setScanFrequencyAddMic(_scan_frequency);
        hz = hz - 0.1;
      }

      frequency = _scan_frequency.frequency / 100.0f;
    } else {
      while (hz < -0.95) {
        lidarPtr->setScanFrequencyDis(_scan_frequency);
        hz = hz + 1.0;
      }

      while (hz < -0.09) {
        lidarPtr->setScanFrequencyDisMic(_scan_frequency);
        hz = hz + 0.1;
      }

      frequency = _scan_frequency.frequency / 100.0f;
    }
  }

  ans = lidarPtr->getScanFrequency(_scan_frequency);

  if (IS_OK(ans)) {
    frequency = _scan_frequency.frequency / 100.0f;
    m_ScanFrequency = frequency;
  }

  m_ScanFrequency -= frequency_offset;
  fixed_size = sample_rate * 1000 / (m_ScanFrequency - 0.1);
  printf("[YDLIDAR INFO][%fs] Current Scan Frequency: %fHz\n",
         (getms() - startTs) / 1000.0, m_ScanFrequency);
  fflush(stdout);
  return true;
}

bool CYdLidar::checkZeroOffsetAngle() {
  bool ret = false;
  zero_offset_angle = 0.0;
  uint32_t startTs = getms();
  result_t ans;
  offset_angle_t angle;
  ans = lidarPtr->getZeroOffsetAngle(angle);

  if (IS_OK(ans)) {
    zero_offset_angle = angle.angle / 4.0;
    printf("[YDLIDAR INFO][%fs] Obtained Zero Offset Angle[%f°] \n",
           (getms() - startTs) / 1000.0, zero_offset_angle);
    fflush(stdout);
    ret = true;
  } else {
    printf("[YDLIDAR ERROR][%fs] Failed to Get Zero Offset Angle[%f°] \n",
           (getms() - startTs) / 1000.0, zero_offset_angle);
    fflush(stdout);
  }

  return ret;
}

bool CYdLidar::checkStatus() {
  uint32_t startTs = getms();

  if (!checkCOMMs()) {
    return false;
  }

  single_channel = false;
  bool ret  = getDeviceHealth();

  if (!ret) {
    ret = getDeviceHealth(100);
  }

  if (!getDeviceInfo()) {
    if (!ret) {
      ret = getDeviceInfo();

      if (!ret) {
        single_channel = true;
        ret = true;
      }
    } else {
      ret = getDeviceInfo();
    }
  } else {
    ret = true;
  }

  printf("[YDLIDAR INFO][%fs] single channel:  %d\n",
         (getms() - startTs) / 1000.0, single_channel);
  fflush(stdout);
  lidarPtr->setSingleChannel(single_channel);
  lidarPtr->setIntensity(m_Intensity);
  return ret;
}

/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new YDlidarDriver();

    if (!lidarPtr) {
      printf("Create Driver fail\n");
      fflush(stdout);
      return false;

    }

    printf("[YDLIDAR INFO] Now YDLIDAR SDK VERSION: %s\n",
           YDlidarDriver::getSDKVersion().c_str());
    fflush(stdout);

  }

  if (isConnected) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3) {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm') {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4') {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result)) {
    printf("[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
           m_SerialPort.c_str(), m_SerialBaudrate);
    fflush(stdout);
    return false;
  }

  printf("[YDLIDAR INFO] Connection established in %s[%d]:\n",
         m_SerialPort.c_str(),
         m_SerialBaudrate);
  fflush(stdout);

  isConnected = true;
  return true;
}


/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  if (!lidarPtr) {
    return false;
  }

  if (isScanning && lidarPtr->isScanning()) {
    return true;
  }

  return false;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize() {
  bool ret = true;

  if (!checkCOMMs()) {
    printf("[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
    fflush(stdout);
    return false;
  }

  if (!checkStatus()) {
    printf("[CYdLidar::initialize] Error initializing YDLIDAR check status under [%s] and [%d].\n",
           m_SerialPort.c_str(), m_SerialBaudrate);
    fflush(stdout);
    return false;
  }

  printf("LiDAR init success!\n");
  fflush(stdout);
  return ret;
}
