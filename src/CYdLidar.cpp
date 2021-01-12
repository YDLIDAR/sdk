
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
  sn_status           = false;
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
    bool sn_ok = sn_status;
    if(sn_ok){
        memcpy(&version,&m_LidarVersion,sizeof(version));
    }
    return sn_ok;
    //memcpy(&version, &m_LidarVersion, sizeof(LidarVersion));
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
CYdLidar::PIDError  CYdLidar::initPIDParams(){
   PIDError error = NoError;
   int fd =0,len = -1;
   if(!path_exists(PWMExportPath)){
    //   printf("PWMExportPath:%s,ExportPath:%s\n",PWMExportPath.c_str(),ExportPath.c_str());
   //    printf("export not exist\n");
       fflush(stdout);
       fd  = open(ExportPath.c_str(),O_WRONLY|O_CREAT|O_TRUNC);
       if(fd == -1){
           error = WriteExportError;
           return error;
       }
       char a  = '0';
       //char * a = "0";
       len = write(fd,(void*)&a,sizeof (char));
       close(fd);

       fflush(stdout);
       if(len != sizeof (char)){
           printf("write export failed\n");
          // error = WriteExportError;
          // return error;
       }

   }

   {
       fd  = open(PeriodPath.c_str(),O_WRONLY|O_CREAT|O_TRUNC);
       if(fd == -1){
           error = WritePeriodError;
           return error;
       }

       //str.clear();
       string str = "20000";
       //char * a = "0";
       len = write(fd,(void*)str.c_str(),sizeof (str.c_str()));
       close(fd);
       if(len != sizeof (str.c_str())){
           error = WritePeriodError;
           return error;
       }

       fd  = open(DutyPath.c_str(),O_WRONLY|O_CREAT|O_TRUNC);
       if(fd == -1){
           error = WriteDutyError;
           return error;
       }
       str.clear();
       str = "6000";
       //char * a = "0";
       len = write(fd,(void*)str.c_str(),sizeof (str.c_str()));
       close(fd);
       if(len != sizeof (str.c_str())){
           error = WriteDutyError;
           return error;
       }

       fd  = open(ModePath.c_str(),O_WRONLY|O_CREAT|O_TRUNC);
       if(fd == -1){
           error = WriteModeError;
           return error;
       }
       str.clear();
       str = "normal";
       //char * a = "0";
       len = write(fd,(void*)str.c_str(),sizeof (str.c_str()));;
       close(fd);
       if(len != sizeof (str.c_str())){
           error = WriteModeError;
           return error;
       }

    //   printf("enablePath:%s\n",EnablePath.c_str());
       fd  = open(EnablePath.c_str(),O_WRONLY|O_CREAT|O_TRUNC);
       if(fd == -1){
      //     printf("open enable failed\n");
       //    fflush(stdout);
           error = WriteEnableError;
           return error;
       }

       str.clear();
       str = "1";
       //char * a = "0";
       len = write(fd,(void*)str.c_str(),sizeof (str.c_str()));
       //len = write(fd,(void*)&str.,sizeof(char));
       close(fd);
       if(len != sizeof (str.c_str())){

           printf("write enable failed,size:%d\n",len);
           error = WriteEnableError;
           return error;
       }
   }
   return error;

}

bool  CYdLidar::doProcessSimple(LaserScan &scan_msg, bool &hardwareError) {
  hardwareError			= false;
  scan_msg.lidar_scan_frequency = 0.0;

  // Bound?
  if (!checkHardware()) {
    scan_msg.data.clear();
    delay(100 / m_ScanFrequency);
    hardwareError = false;
    return false;
  }
  //  wait Scan data:
  float tim_scan_start = getHDTimer(); //getTime();
  laser_packages.points.clear();
  result_t op_result = lidarPtr->grabScanData(&laser_packages);
  if(!sn_status){
      if(laser_packages.info.size < 13 || laser_packages.info.valid != 0x01){
      //    printf("ct.size:%d, valid:%d\n",laser_packages.info.size,laser_packages.info.valid);
      //    printf("ct packet size less than 13 or ct packet is invalid\n");
          fflush(stdout);
          goto end;
        //  ans = RESULT_FAIL;
         // return ans;
      }
      //获取序列号
      uint32_t  sn_ = 0;
      int year =  ((laser_packages.info.info[9] >>2) & 0x1f) + 2020;
      int month = (laser_packages.info.info[10] >>3) & 0x0f;
      int day  =  (laser_packages.info.info[11] >>2) & 0x1f;

      struct SN_tmp{
          uint32_t sn1 :7 ;
          uint32_t sn2 :7 ;
          uint32_t sn3 :2 ;
          uint32_t sn4 :3 ;
          uint32_t sn5 :2 ;
          uint32_t sn6 :11 ;
      } __attribute__((packed));
      SN_tmp * p_sn = (SN_tmp*)&sn_;
      p_sn->sn1 = laser_packages.info.info[13];
      p_sn->sn2 = laser_packages.info.info[12];
      p_sn->sn3 = (laser_packages.info.info[11]) & 0x03;
      p_sn->sn4 = (laser_packages.info.info[10]) & 0x07;
      p_sn->sn5 = (laser_packages.info.info[9]) & 0x03;
      printf("size of lidarVersion.sn:%d",sizeof (m_LidarVersion.sn));
      memset(m_LidarVersion.sn,0,sizeof (m_LidarVersion.sn));

      m_LidarVersion.sn[0+16] = year/1000;
      m_LidarVersion.sn[1+16] = (year%1000) /100;
      m_LidarVersion.sn[2+16] = (year%100) /10;
      m_LidarVersion.sn[3+16] = year%10;
      m_LidarVersion.sn[4+16] = month /10;
      m_LidarVersion.sn[5+16] = month %10;
      m_LidarVersion.sn[6+16] = day /10;
      m_LidarVersion.sn[7+16] = day %10;
//      m_LidarVersion.sn[8+16] = 0;
//      m_LidarVersion.sn[9+16] = 0;
//      m_LidarVersion.sn[10+16] = 0;
      m_LidarVersion.sn[11+16] = sn_ / 10000;
      m_LidarVersion.sn[12+16] = (sn_ % 10000) / 1000;
      m_LidarVersion.sn[13+16] = (sn_ % 1000) / 100;
      m_LidarVersion.sn[14+16] = (sn_ % 100) / 10 ;
      m_LidarVersion.sn[15+16] = sn_ % 10 ;


      int8_t Major = (laser_packages.info.info[1] >> 5) & 0x03;
      int8_t Minor = laser_packages.info.info[1] & 0x1f;
      int8_t HardwareVer = (laser_packages.info.info[4] >>4) & 0x07;
      int8_t FireMajor = (laser_packages.info.info[4]) & 0x0f;
      int8_t FireMinor = laser_packages.info.info[5];

      m_LidarVersion.hardware = HardwareVer;
      m_LidarVersion.soft_major = Major;
      m_LidarVersion.soft_minor = Minor / 10;
      m_LidarVersion.soft_patch = Minor % 10;
     // printf("minor:%d,major:%d,hardware:%d\n",Minor,Major,HardwareVer);
      m_LidarVersion.fire_major = FireMajor;
      m_LidarVersion.fire_minor = FireMinor;
   //   printf("1:%02x，4:%02x，5:%02x",laser_packages.info.info[1],laser_packages.info.info[4],laser_packages.info.info[5]);
      printf("sn:");
      for(int i=0;i<32 ;i++){
          printf(" %x",m_LidarVersion.sn[i]);
      }
      printf("\n");
      fflush(stdout);
      sn_status = true;
  }

  end:

  float tim_scan_end = getHDTimer(); //getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    LaserPoint point;

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    size_t count = laser_packages.points.size();
    float scan_time = (point_interval_time * (count - 1)) / 1000000.0;

    if (count == 1) {
      scan_time = point_interval_time / 1000000.0;
    }

    tim_scan_end -= package_transfer_time / 1000000.0;
    tim_scan_end += m_OffsetTime * 1e3;
    tim_scan_end -= point_interval_time / 1000000.0;
    tim_scan_start = tim_scan_end -  scan_time ;
    last_node_time = tim_scan_end;
    scan_msg.data.clear();
    scan_msg.config.scan_time =
      scan_time; //static_cast<float>(1.0 * scan_time / 1e9);
    scan_msg.config.fixed_size = fixed_size;
    scan_msg.config.min_angle = angles::from_degrees(m_MinAngle);
    scan_msg.config.max_angle = angles::from_degrees(m_MaxAngle);

    if (count > 1) {
      scan_msg.config.time_increment = scan_msg.config.scan_time / (double)(
                                         count - 1);
    } else {
      scan_msg.config.time_increment = scan_msg.config.scan_time;
    }

    scan_msg.system_time_stamp = tim_scan_end;
    scan_msg.config.min_range = m_MinRange;
    scan_msg.config.max_range = m_MaxRange;

    if (laser_packages.info.info[0] > 0) {
      scan_msg.lidar_scan_frequency = laser_packages.info.info[0] / 10.0;
    }
 //   printf("laser_packages.info.info[0]:%d,scan_msg.lidar_scan_frequency:%f\n",laser_packages.info.info[0],scan_msg.lidar_scan_frequency);

//    if (!checkHealth(laser_packages.info)) {
    if (lidarPtr->getDriverError() != NoError) {
      hardwareError = true;
      return false;
    }

    for (int i = 0; i < count; ++i) {
      point = laser_packages.points[i];
      point.range = point.range / 1000.f;
      point.angle = angles::from_degrees(point.angle + zero_offset_angle);

      if (m_Reversion) {
        point.angle += M_PI;
      }

      point.angle = 2 * M_PI - point.angle;
      point.angle = angles::normalize_angle(point.angle);

      if (m_GlassNoise && point.interference_sign == GLASSNOISEINTENSITY) {
        point.range = 0.0;
      }

      if (m_SunNoise && point.interference_sign == SUNNOISEINTENSITY) {
        point.range  = 0.0;
      }

      if (point.range > m_MaxRange || point.range < m_MinRange) {
        point.range = 0.0;
      }

      if (point.angle >= scan_msg.config.min_angle &&
          point.angle <= scan_msg.config.max_angle) {
//        if (scan_msg.data.empty()) {
//          scan_msg.system_time_stamp = tim_scan_start + i * point_interval_time /
//                                       1000000.0;
//        }

        scan_msg.data.push_back(point);
      }
    }

    ///>调速
    static uint32_t TickCNT = 0;
    int16_t  increase; ///> 最后得出的实际增量值
    static  int16_t  CurErr = 0;
    static  int16_t  LastErr = 0;
    static  int16_t  PreErr  = 0;
    static  int16_t  writeDuty = default_mode_duty;
    int interger_size = 4;

    if(getHDTimer() - TickCNT >= 100){
    //    printf("开始更新increase:%d\n",laser_packages.info.info[0]);
        TickCNT = getHDTimer();
        if(laser_packages.info.info[0] == 0){
            writeDuty = default_mode_duty;
        }else{
            CurErr = (int16_t)(((int)m_ScanFrequency) * 100 - laser_packages.info.info[0] *10); ///> 当前频率减去获取到的频率，固定设置频率为6，
            increase = (int16_t)(PID_P * (CurErr - LastErr) *1.0 + PID_I* CurErr *1.0 + PID_D * (CurErr -2* LastErr+ PreErr));
        //    printf("PreErr:%d,lastErr:%d,CurErr:%d,increase:%f\n",PreErr,LastErr,CurErr,increase);
            PreErr = LastErr;
            LastErr = CurErr;

            writeDuty += increase;
            interger_size = get_int_size(writeDuty);
            if(default_mode_duty + increase > 19900){
                writeDuty = 19900;
            }else if (default_mode_duty + increase < 50){
                writeDuty = 50;
            }else{

            }
        }

        char a[10];
        memset(a,0,10);
        sprintf(a,"%d",writeDuty);;

        //sprintf(value,"%d",writeDuty);
       // printf("写入值:%d\n",increase);
        std::string  dutypath = DutyPath;
        int fd = open(dutypath.c_str(),O_WRONLY | O_CREAT);
        if(fd != -1){
           int len = write(fd,a,interger_size);
           close(fd);
           if (len != interger_size){
               printf("len:%d,sizeof(value):%d",len,interger_size);
               printf("write duty failed\n");
               fflush(stdout);
           }
        }
    }


    return true;

  } else {
    if (IS_FAIL(op_result)) {
      // Error? Retry connection
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


//
void  CYdLidar::initPwdPath(int number){
    string serial = "pwmchip1";
    if(number != 1)
        serial = "pwmchip0";
    EnablePath = string("/sys/class/pwm/") + serial + string("/pwm0/enable");
    ExportPath = string("/sys/class/pwm/") + serial + string("/export");
    PWMExportPath = string("/sys/class/pwm/") + serial + string("/pwm0/export");
    PeriodPath = string("/sys/class/pwm/") + serial + string("/pwm0/period");
    DutyPath = string("/sys/class/pwm/") + serial + string("/pwm0/duty_cycle");
    ModePath = string("/sys/class/pwm/") + serial + string("/pwm0/polarity");
    UnexportPath = string("/sys/class/pwm/") + serial + string("/unexport");
}

/*-------------------------------------------------------------
                        turnoffPWM
-------------------------------------------------------------*/
int  CYdLidar::PoweroffPWM(){
    PIDError  error = NoError;
    string str = "0";
    int fd  = open(EnablePath.c_str(),O_WRONLY|O_CREAT|O_TRUNC);
    if(fd == -1){
        error = WriteEnableError;
        return error;
    }
    int len = write(fd,(void*)str.c_str(),sizeof (str.c_str()));
    close(fd);
    if(len != sizeof (str.c_str())){
        error = WriteEnableError;
        return error;
    }

    fd  = open(UnexportPath.c_str(),O_WRONLY|O_CREAT|O_TRUNC);
    if(fd == -1){
        error =  WriteUnenabelError;
        return error;
    }
    len = write(fd,(void*)str.c_str(),sizeof (str.c_str()));
    close(fd);
    if(len != sizeof (str.c_str())){
        error =  WriteUnenabelError;
        return error;
    }
    return  error;

}

//
string CYdLidar::getErrorString(int error){
    string error_str;
    switch (error) {
      case CYdLidar::NoError:
        error_str = "";
        break;
      case CYdLidar::WriteEnableError:
        error_str = "write enable error!";
        break;
      case CYdLidar::WriteDutyError:
        error_str = "write duty error!";
        break;
      case CYdLidar::WriteModeError:
        error_str = "write mode error!";
        break;
      case CYdLidar::WriteExportError:
        error_str = "write export error!";
        break;
      case CYdLidar::WritePeriodError:
        error_str = "write period error!";
        break;
      case CYdLidar::WriteUnenabelError:
        error_str = "write unenable error!";
        break;
    }
    return error_str;
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
  int  rlt = PoweroffPWM();
  string err_str = getErrorString(rlt);
  if(rlt != 0){
      printf("power off the lidar failed ,error:%s",err_str.c_str());
  }
  isScanning = false;
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
  m_LidarVersion.soft_patch = Minjor % 10;
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
