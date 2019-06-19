YDLIDAR SDK PACKAGE V1.4.1
=====================================================================

SDK [test](https://github.com/ydlidar/sdk/tree/X2) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================
   
    $ git clone https://github.com/ydlidar/sdk
    
    $ cd sdk
    
    $ git checkout X2
    
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
    $Please enter the lidar serial baud rate:115200

windows:

    $ ydlidar_test.exe
    $Please enter the lidar serial port:COM3
    $Please enter the lidar serial baud rate:115200


You should see YDLIDAR's scan result in the console:

     	YDLIDAR C++ TEST
     	
	[YDLIDAR INFO] Now YDLIDAR SDK VERSION: 1.4.1
	
	fhs_lock: creating lockfile:      11796

	[YDLIDAR INFO] Connection established in /dev/ttyUSB0[115200]:
	
	[YDLIDAR INFO] Now YDLIDAR is scanning ......
	
	Scan received: 368 ranges


