YDLIDAR SDK PACKAGE V1.4.4
=====================================================================

SDK [test](https://github.com/ydlidar/sdk/tree/feature/suitable_qfeel) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================
   
    $ git clone https://github.com/ydlidar/sdk
    
    $ cd sdk
    
    $ git checkout feature/suitable_qfeel
    
    $ cd ..
    
    $ mkdir build
    
    $ cd build
    
    $ cmake -DARCH=arm -DCMAKE_TOOLCHAIN_FILE=../rk3308.cmake ../
    
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

    [YDLIDAR INFO] Now YDLIDAR SDK VERSION: 1.4.5
    [YDLIDAR INFO] Connection established in /dev/ttyUSB0[115200]:
    [YDLIDAR]:SDK Version: 1.4.5
    [YDLIDAR][0.001000s]:Lidar running correctly ! The health status: good
    [YDLIDAR][0.003000s] Device Info:
    Firmware version: 1.3
    Hardware version: 3
    Model: S2Pro[4]
    Serial: 2020080400011111
    [YDLIDAR INFO][0.003000s] Current Scan Frequency: 6.000000Hz
    [YDLIDAR INFO] Current Sampling Rate : 3K
    [YDLIDAR INFO][0.010000s] single channel:  0
    LiDAR init success!
    [YDLIDAR INFO][1.043000s] Now YDLIDAR is scanning ......


