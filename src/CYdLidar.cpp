#include "CYdLidar.h"
#include "common.h"
#include <map>
#include <algorithm>
#include "timer.h"
//#include "glog/logging.h"


using namespace std;
using namespace ydlidar;
using namespace impl;

// #define DEG2RAD(x) ((x)*M_PI/180.)
// #define YDLIDAR_DEBUG 0

/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(0)
{
    m_SerialPort        = "";
    m_SerialBaudrate    = 115200;
    m_FixedResolution   = false;
    m_Reversion         = false;
    m_AutoReconnect     = false;
    m_MaxAngle          = 180.f;
    m_MinAngle          = -180.0f;
    m_MaxRange          = 8.0;
    m_MinRange          = 0.08;
    isScanning          = false;
    isConnected         = false;
    node_counts         = 720;
    each_angle          = 0.5;
    m_IgnoreArray.clear();
// #if YDLIDAR_DEBUG
//     out_file_.open("/tmp/log/lidar.rawlog", 1);
// #endif
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar()
{
    disconnecting();
}

void CYdLidar::disconnecting()
{
    if (lidarPtr) {
        lidarPtr->disconnect();
        delete lidarPtr;
        lidarPtr    = nullptr;
    }
}

/*-------------------------------------------------------------
  doProcessSimple
  -------------------------------------------------------------*/
/*  
bool  CYdLidar::doProcessSimple(mrpt::obs::CObservation2DRangeScan &outscan, bool &hardwareError, bool if_compensate){
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    return false;
  }

  node_info nodes[2048];
  size_t   count = _countof(nodes);
  int all_nodes_counts = node_counts;

  //  wait Scan data:
  result_t op_result =  lidarPtr->grabScanData(nodes, count);
  // Fill in scan data:
  if (IS_OK(op_result))
  {
    op_result = lidarPtr->ascendScanData(nodes, count);
    if (IS_OK(op_result))
    {
      if (!m_FixedResolution) {
        all_nodes_counts = count;
      } else {
        all_nodes_counts = node_counts;
      }

      struct timeval t;
      gettimeofday(&t,NULL);
      //outscan.timestamp = mrpt::system::now_double() * pow(10,9);
      //std::cout<<"outscan.timestamp : "<<outscan.timestamp<<std::endl;
      outscan.timestamp = t.tv_sec * 1e9 + t.tv_usec * 1e3;
      outscan.rightToLeft = false; //clockwise
      // outscan.rightToLeft = true; //counterclockwise
      outscan.sensorLabel = "laser";
      outscan.aperture = mrpt::utils::DEG2RAD(m_MaxAngle - m_MinAngle);
      outscan.maxRange = m_MaxRange;
      outscan.resizeScan(all_nodes_counts);
      //Todo Pierre: It should be set by laser parameters.
      outscan.sensorPose = mrpt::poses::CPose3D(0.0255, 0.0, 0.0, mrpt::utils::DEG2RAD(100), 0.0, 0.0); //

      each_angle = 360.0/all_nodes_counts;
      unsigned int i = 0;
      for( ; i < count; i++) {
        if (nodes[i].distance_q2 != 0) {
          float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
          int inter =(int)( angle / each_angle );
          float angle_pre = angle - inter * each_angle;
          float angle_next = (inter+1) * each_angle - angle;
          if (angle_pre < angle_next) {
            if(inter < all_nodes_counts) {
              outscan.setScanRange(inter, (float)nodes[i].distance_q2/4000.0f);
              outscan.setScanRangeValidity(inter, true);
            }
          } else {
            if (inter < all_nodes_counts -1) {
              outscan.setScanRange(inter+1, (float)nodes[i].distance_q2/4000.0f);
              outscan.setScanRangeValidity(inter+1, true);
            }
          }
        }

      }
      
      //Todo Pierre: Does it similar to adjustment of the laser distortion? And it should be removed to outside.
      if (if_compensate)
        compensateLidar(outscan);

      //Todo Pierre: It should be set by laser pillar parameters which are relative to robot coordinate.
      //             Furthermore, it should be calculated in one loop and removed to outside.
      //Remove covered data
      int invalid_start = 5;
      int remove_number = 28;
      for(int invalid_index = invalid_start; invalid_index < invalid_start + remove_number; ++invalid_index) {
        outscan.setScanRangeValidity(invalid_index, false);
      }

      invalid_start += 100;
      for(int invalid_index = invalid_start; invalid_index < invalid_start + remove_number; ++invalid_index) {
        outscan.setScanRangeValidity(invalid_index, false);
      }

      invalid_start += 120;
      for(int invalid_index = invalid_start; invalid_index < invalid_start + remove_number; ++invalid_index) {
        outscan.setScanRangeValidity(invalid_index, false);
      }

      invalid_start += 120;
      for(int invalid_index = invalid_start; invalid_index < invalid_start + remove_number; ++invalid_index) {
        outscan.setScanRangeValidity(invalid_index, false);
      }

      invalid_start += 120;
      for(int invalid_index = invalid_start; invalid_index < invalid_start + remove_number; ++invalid_index) {
        outscan.setScanRangeValidity(invalid_index, false);
      }

      invalid_start += 115;
      for(int invalid_index = invalid_start; invalid_index < invalid_start + remove_number; ++invalid_index) {
        outscan.setScanRangeValidity(invalid_index, false);
      }
           

      return true;


    }

  } else {
    if (op_result==RESULT_FAIL) {
      // Error? Retry connection
      //this->disconnect();
    }
  }

  return false;

} */

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, LaserScanMsg& msg, bool &hardwareError){
	hardwareError			= false;
  if (!checkHardware()) {
    hardwareError = true;
    return false;
	}
  node_info nodes[2048];
  size_t count = _countof(nodes);
  size_t all_nodes_counts = node_counts;

  //  wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(nodes, count, 500);
  outscan.scan_cnts = count;

	// Fill in scan data:
  if (IS_OK(op_result))
    {
      for(size_t i=0;i<count;i++)
      printf("[%u] [range][%f] [angle][%f] stamp[%llu]\n",i,nodes[i].distance_q2 / 4000.0f,nodes[i].angle_q6_checkbit/ 128.0,
             nodes[i].stamp);
      fflush(stdout);
    op_result = lidarPtr->ascendScanData(nodes, count);
    uint64_t max_time =nodes[0].stamp ;
    uint64_t min_time = nodes[0].stamp;
    if (IS_OK(op_result)){
      if (!m_FixedResolution){
        all_nodes_counts = count;
      } else {
        all_nodes_counts = node_counts;
      }
    each_angle = 360.0/all_nodes_counts;
    node_info *angle_compensate_nodes = new node_info[all_nodes_counts];
    memset(angle_compensate_nodes, 0, all_nodes_counts*sizeof(node_info));
    unsigned int i = 0;
    msg.data.clear();
    for( ; i < count; i++) {
      if (nodes[i].distance_q2 != 0) {
        float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
        LaserPoint point;
        point.angle = angle / 180 * M_PI;
        point.range = nodes[i].distance_q2/4000.f;
        // std::cout<<"angle : "<< point.angle<<" range is: "<<point.range<<std::endl;
        msg.data.push_back(point);
        if(m_Reversion){
          std::cout<<"m_Reversion is true"<<std::endl;
          angle=angle+180;
          if(angle>=360){ 
            angle=angle-360;
          }
          nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
        }
        unsigned int inter =(unsigned int)( angle / each_angle );
        //int inter =(int)( angle / each_angle );
        float angle_pre = angle - inter * each_angle;
        float angle_next = (inter+1) * each_angle - angle;
        if (angle_pre < angle_next) {
          if(inter < all_nodes_counts)
            angle_compensate_nodes[inter]=nodes[i];
        } else {
          if (inter < all_nodes_counts -1)
            angle_compensate_nodes[inter+1]=nodes[i];
        }
      }

      if(nodes[i].stamp > max_time) {
        max_time = nodes[i].stamp;
      }

      if(nodes[i].stamp < min_time) {
        min_time = nodes[i].stamp;
      }
    }
    LaserScan scan_msg;

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
      range = (float)angle_compensate_nodes[i].distance_q2/4000.f;
      intensity = (float)(angle_compensate_nodes[i].sync_quality);
      uint16_t disturb_value = angle_compensate_nodes[i].distance_q2 & 3;
      noise_flag = (disturb_value == 2 || disturb_value == 3); //2-sunnoise 3-glassnoise
      // uint16_t disturb_value = angle_compensate_nodes[i].distance_q2 & 3;
      // if(disturb_value == 2 || disturb_value == 3) { //2-sunnoise 3-glassnoise
      //   range = 0.0;
      // }
      uint64_t stamp = angle_compensate_nodes[i].stamp;
      if (i<all_nodes_counts/2) {
        index = all_nodes_counts/2-1-i;
      } else {
        index =all_nodes_counts-1-(i-all_nodes_counts/2);
      }
      if (m_IgnoreArray.size() != 0) {
        float angle = (float)((angle_compensate_nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
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
//          printf("[%d][%u] [range][%f] stamp[%llu][%llu]\n",i,pos,range,
//                 stamp,nodes[i].stamp);
        }
      }

      scan_msg.system_time_stamp = tim_scan_start;
      scan_msg.self_time_stamp = tim_scan_start;
      scan_msg.end_time_stamp = max_time;
      scan_msg.config.min_angle = DEG2RAD(m_MinAngle);
      scan_msg.config.max_angle = DEG2RAD(m_MaxAngle);
      scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) / (double)counts;
      scan_msg.config.time_increment = scan_time / (double)counts *1e-9f;
      scan_msg.config.min_range = m_MinRange;
      scan_msg.config.max_range = m_MaxRange;
      outscan = scan_msg;
      msg.config.max_range = m_MaxRange;
      msg.config.min_range = m_MinRange;
      delete[] angle_compensate_nodes;
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

/*-------------------------------------------------------------
                        turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn()
{
    bool ret = isScanning;
    if (!isScanning) {
        result_t ans = lidarPtr->startScan();
        ret = true;
        if(!IS_OK(ans)) {
            ret = false;
        }else {
            isScanning = true;
            printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
            fflush(stdout);
        }
	}

	return ret;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff()
{
  if (lidarPtr) {
    lidarPtr->stop();
    lidarPtr->stopMotor();        
	}
  
  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
  }
  isScanning = false;
	return true;
}

/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs()
{
    if (!lidarPtr) {
        // create the driver instance
        lidarPtr = new YDlidarDriver();
        if (!lidarPtr) {
            fprintf(stderr, "Create Driver fail\n");
            return false;

        }
        printf("[YDLIDAR INFO] Now YDLIDAR SDK VERSION: %s\n", YDlidarDriver::getSDKVersion().c_str());
        fflush(stdout);

    }
    if (isConnected) {
        return true;
    }

	// Is it COMX, X>4? ->  "\\.\COMX"
    if (m_SerialPort.size()>=3) {
        if ( tolower( m_SerialPort[0]) =='c' && tolower( m_SerialPort[1]) =='o' && tolower( m_SerialPort[2]) =='m' ) {
			// Need to add "\\.\"?
            if (m_SerialPort.size()>4 || m_SerialPort[3]>'4')
                m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
		}
	}

	// make connection...
    result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);
    if (!IS_OK(op_result)) {
        fprintf(stderr, "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",  m_SerialPort.c_str(), m_SerialBaudrate );
		return false;
	}
    printf("[YDLIDAR INFO] Connection established in %s[%d]:\n",m_SerialPort.c_str(), m_SerialBaudrate);
    fflush(stdout);

    isConnected = true;
	return true;
}


/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware()
{
    bool ret = true;
    if (!isScanning) {
        ret = false;
        if (checkCOMMs()) {   
            if (turnOn()) {
                ret = true;
            }
        }
    }

    return ret;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize()
{
	bool ret = true;
    if (!checkCOMMs()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
        fflush(stderr);
        return false;
	}
    if (!turnOn()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
        fflush(stderr);
        ret = false;
	}

    return ret;
}
