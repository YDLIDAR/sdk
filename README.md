YDLIDAR SDK PACKAGE V1.0.0
=====================================================================

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================
    
    $ mkdir build
    
    $ cd build
    
    $ cmake ../sdk
    
    $ make			###linux
    
    $ vs open Project.sln	###windows

    ```


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

S2-Pro:

    [YDLIDAR INFO] Now YDLIDAR SDK VERSION: 1.0.0
    [YDLIDAR INFO] Connection established in /dev/ttyUSB0[256000]:
    [YDLIDAR]:SDK Version: 1.0.0
    [YDLIDAR][0.001000s]:Lidar running correctly ! The health status: good
    [YDLIDAR][0.003000s] Device Info:
    Firmware version: 2.3
    Hardware version: 3
    Model: TSA[130]
    Serial: 2020080400011111
    [YDLIDAR INFO][0.003000s] Current Scan Frequency: 6.000000Hz
    [YDLIDAR INFO] Current Sampling Rate : 3K
    [YDLIDAR INFO][0.010000s] single channel:  0
    LiDAR init success!
    [YDLIDAR INFO][1.043000s] Now YDLIDAR is scanning ......
    Scan received: 377 ranges in 7.978732 HZ


S2:

![](run.png)

