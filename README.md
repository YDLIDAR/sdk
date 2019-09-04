YDLIDAR SDK PACKAGE V1.4.1
=====================================================================

SDK [test](https://github.com/ydlidar/sdk/tree/S3) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================
   
    $ git clone https://github.com/ydlidar/sdk
    
    $ cd sdk
    
    $ git checkout MD
    
    $ cd ..
    
    $ mkdir build
    
    $ cd build
    
    $ cmake ../sdk
    
    $ make			###linux
    
    $ vs open Project.sln	###windows
    
How to run YDLIDAR SDK samples
=====================================================================
    $ cd samples

linux:

    $ ./ydlidar_test
    $Please enter the lidar serial port :/dev/ttyUSB0

windows:

    $ ydlidar_test.exe
    $Please enter the lidar serial port:COM3



You should see YDLIDAR's scan result in the console:

     	YDLIDAR C++ TEST
     	
	[YDLIDAR INFO] Now YDLIDAR SDK VERSION: 1.4.1
	
	fhs_lock: creating lockfile:      11796

	[YDLIDAR INFO] Connection established in /dev/ttyUSB0[115200]:
	
	[YDLIDAR INFO] Now YDLIDAR is scanning ......
	
	Scan received: 606 ranges
	
##雷达实时频率怎么计算:

	MD01型号雷达的采样频率是固定的３K,也就是说samleRate = 3000.
	采样率表示一秒钟雷达有多少数据点．
	所以采样一个点的时间就是pointTime = 1.0/sampleRate = 1.0/3000.
	假如现在获取一圈的雷达数据个数count=600,扫描时间scan_time = count*pointTime = 600/3000= 0.2(s);
	当前雷达频率scan_frequancy = 1.0/scan_time = 1/0.2 = 5(HZ)
	
##整机零位修正怎么用：
	
	1.保证整机零度朝向一面直墙, 机器离直墙距离大概1m, 直墙长度大于1.1米．
	
	2.通过接口setRobotLidarDifference函数设置机器理论零度和雷达零度的理论偏差值，　同一个方向就是０，偏差90度就是90, 反向就是180.
	
	3.通过接口函数setStartAngleOffset(true)来开启整机零位修正．
	
	3.通过接口函数getStartAngleOffset() 可以判断当前零位修正还在修正中，返回值为true, 当前雷达还在修正中，否则，没在修正中．
	
	4.通过接口函数isAngleOffetCorrected()可以判断当前机器是否已经修正过零位．返回值为true，表示已经修正过，否则，没有修正过．
	
	
	
	


