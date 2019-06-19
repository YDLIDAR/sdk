![YDLIDAR](image/index-X4.jpg  "YDLIDAR_X4")

YDLIDAR SDK [![Build Status](https://travis-ci.org/cansik/sdk.svg?branch=samsung)](https://travis-ci.org/cansik/sdk) [![Build status](https://ci.appveyor.com/api/projects/status/2w9xm1dbafbi7xc0?svg=true)](https://ci.appveyor.com/project/cansik/sdk) [![codebeat badge](https://codebeat.co/badges/3d8634b7-84eb-410c-b92b-24bf6875d8ef)](https://codebeat.co/projects/github-com-cansik-sdk-samsung)
=====================================================================


Introduction
-------------------------------------------------------------------------------------------------------------------------------------------------------

YDLIDAR(https://www.ydlidar.com/) series is a set of high-performance and low-cost LIDAR sensors, which is the perfect sensor of 2D SLAM, 3D reconstruction, multi-touch, and safety applications.

If you are using ROS (Robot Operating System), please use our open-source [ROS Driver]( https://github.com/ydlidar/ydlidar) .

Release Notes
-------------------------------------------------------------------------------------------------------------------------------------------------------
| Title      |  Version |  Data |
| :-------- | --------:|  :--: |
| SDK     |  1.4.1 |   2019-06-19  |

- [new feature] increase the deviation between correcting the zero angle of the lidar and the zero angle of the robot.




Dataset 
-------------------------------------------------------------------------------------------------------------------------------------------------------


| Model      |  Baudrate |  Sampling Frequency | Range(m)  | Scanning Frequency(HZ) | Working temperature(°C) | Laser power max(mW) | voltage(V) | Current(mA)
| :-------- | --------:|--------:|  --------:| --------:|--------:| --------:| --------:|  :--: |
| R2 |  230400 |   5000  |  0.1-16   |5-12|0-50| ~5|4.8-5.2|400-480|
| G4     |  230400 |   9000  |  0.26-16   |5-12|0-50| ~5|4.8-5.2|400-480|

How to build YDLIDAR SDK samples
---------------
    $ git clone https://github.com/ydlidar/sdk
    $ cd sdk
    $ git checkout R2
    $ cd ..
    $ mkdir build
    $ cd build
    $ cmake ../sdk
    $ make			###linux
    $ vs open Project.sln	###windows

How to run YDLIDAR SDK samples
---------------

linux:

    $ ./ydlidar_test
    $Please enter the lidar serial port:/dev/ttyUSB0

windows:

    $ ydlidar_test.exe
    $Please enter the lidar serial port:/dev/ttyUSB0


You should see YDLIDAR's scan result in the console:

	[YDLIDAR]:SDK Version: 2.0.8
	[YDLIDAR]:Lidar running correctly ! The health status: good
	[YDLIDAR] Connection established in [/dev/ttyUSB0][230400]:
	Firmware version: 1.2
	Hardware version: 3
	Model: R2
	Serial: 2018101800011111
	[YDLIDAR INFO] Current Sampling Rate : 5K
	[YDLIDAR INFO] Successfully obtained the corrected offset angle[0.0000] from the lidar[2018101800011111]
	[YDLIDAR INFO] Current AngleOffset : 0.000000°
	[YDLIDAR INFO] Current Scan Frequency : 8.000000Hz
	[YDLIDAR INFO] Now YDLIDAR is scanning ......
	Scan received: 625 ranges
	Scan received: 626 ranges
	

Data structure
-------------------------------------------------------------------------------------------------------------------------------------------------------

data structure:

	struct LaserPoint {
 	 	//angle[°]
  		float angle;
  		//range[m]
  		float range;
 	 	float intensity;
	};

	//! A struct for returning configuration from the YDLIDAR
	struct LaserConfig {
  		//! Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
 		float min_angle;
  		//! Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
  		float max_angle;
  		//! Scan resoltuion [s]
  		float time_increment;
  		//! Time between scans[s]
  		float scan_time;
  		//! Minimum range [m]
  		float min_range;
  		//! Maximum range [m]
 		 float max_range;
	};

	struct LaserScan {
 		 //! Array of laser data point
  		std::vector<LaserPoint> data;
  		//! System time when first range was measured in nanoseconds
  		uint64_t system_time_stamp;
  		//! Configuration of scan
  		LaserConfig config;
	};

example angle parsing:

    LaserScan scan;

    for(size_t i =0; i < scan.data.size(); i++) {

      LaserPoint point = scan.data[i];

      // current time stamp
      uint64_t time_stamp = scan.system_time_stamp + i * scan.config.time_increment*1e9;

      //current angle
      double angle = point.angle;//°

      //current distance
      double distance = point.range;//meters

      //current intensity
      double intensity = point.intensity;

    }
   
   
   
   Contact EAI
---------------

If you have any extra questions, please feel free to [contact us](http://www.ydlidar.cn/cn/contact)
