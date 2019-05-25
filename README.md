![YDLIDAR](image/index-X4.jpg  "YDLIDAR_X4")

YDLIDAR SDK [![Build Status](https://travis-ci.org/cansik/sdk.svg?branch=samsung)](https://travis-ci.org/cansik/sdk) [![Build status](https://ci.appveyor.com/api/projects/status/2w9xm1dbafbi7xc0?svg=true)](https://ci.appveyor.com/project/cansik/sdk) [![codebeat badge](https://codebeat.co/badges/3d8634b7-84eb-410c-b92b-24bf6875d8ef)](https://codebeat.co/projects/github-com-cansik-sdk-samsung)
=====================================================================


Introduction
-------------------------------------------------------------------------------------------------------------------------------------------------------

YDLIDAR(https://www.ydlidar.com/) series is a set of high-performance and low-cost LIDAR sensors, which is the perfect sensor of 2D SLAM, 3D reconstruction, multi-touch, and safety applications.

If you are using ROS (Robot Operating System), please use our open-source [ROS Driver]( https://github.com/yangfuyuan/ydlidar) .

Release Notes
-------------------------------------------------------------------------------------------------------------------------------------------------------
| Title      |  Version |  Data |
| :-------- | --------:|  :--: |
| SDK     |  1.4.1 |   2019-05-25  |


- [new feature] support single channel lidar.




Dataset 
-------------------------------------------------------------------------------------------------------------------------------------------------------

Support LIDAR Model(Only S4Pro and S4B support intensity)


| Model      |  Baudrate |  Sampling Frequency | Range(m)  | Scanning Frequency(HZ) | Working temperature(°C) | Laser power max(mW) | voltage(V) | Current(mA)
| :-------- | --------:|--------:|  --------:| --------:|--------:| --------:| --------:|  :--: |
| S4     |  115200|    4000 |  0.1-8        |6-12|0-40| ~5|4.8-5.2|330-380|
| S4B |  153600|    4000 |  0.1-8        |6-12|0-40| ~5|4.8-5.2|330-380|

How to build YDLIDAR SDK samples
---------------
    $ git clone https://github.com/ydlidar/sdk
    $ cd sdk
    $ git checkout tb
    $ cd ..
    $ mkdir build
    $ cd build
    $ cmake ../sdk
    $ make			###linux
    $ vs open Project.sln	###windows

How to run YDLIDAR SDK samples
---------------
    $ cd samples

linux:

    $  YDLIDAR C++ TEST
    $Lidar[ydlidar7] detected, whether to select current radar(yes/no)?:yes
	0. 115200
	1. 153600
    $Please enter the lidar serial baud rate:2
	0. false
	1. true
    $Please enter the lidar intensity:1


windows:

      YDLIDAR C++ TEST
    $Lidar[ydlidar7] detected, whether to select current radar(yes/no)?:yes
	0. 115200
	1. 153600
    $Please enter the lidar serial baud rate:2
	0. false
	1. true
    $Please enter the lidar intensity:1


You should see YDLIDAR's scan result in the console:

	[YDLIDAR]:SDK Version: 1.4.1
	[YDLIDAR]:Lidar running correctly ! The health status: good
	[YDLIDAR] Connection established in [/dev/ttyUSB0][153600]:
	Firmware version: 1.20
	Hardware version: 1
	Model: S4B
	Serial: 2018091100006004
	[YDLIDAR INFO] Now YDLIDAR is scanning ......
	Scan received: 488 ranges
	Scan received: 487 ranges
	Scan received: 491 ranges

	
	
Usage
-------------------------------------------------------------------------------------------------------------------------------------------------------

1.Single channel lidar
	
	setsetSingleChannel(true);

2.Dual Channel Radar

	setSingleChannel(false);
	

   
   Contact EAI
---------------

If you have any extra questions, please feel free to [contact us](http://www.ydlidar.cn/cn/contact)
