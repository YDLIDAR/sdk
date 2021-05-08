
#pragma once
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>

#define PropertyBuilderByName(type, name, access_permission)\
    access_permission:\
        type m_##name;\
    public:\
    inline void set##name(type v) {\
        m_##name = v;\
    }\
    inline type get##name() {\
        return m_##name;\
}\


using namespace ydlidar;

class YDLIDAR_API CYdLidar
{
    PropertyBuilderByName(float,MaxRange,private)///< ���úͻ�ȡ��������෶Χ
    PropertyBuilderByName(float,MinRange,private)///< ���úͻ�ȡ������С��෶Χ
    PropertyBuilderByName(float,MaxAngle,private)///< ���úͻ�ȡ�������Ƕ�, ���ֵ180��
    PropertyBuilderByName(float,MinAngle,private)///< ���úͻ�ȡ������С�Ƕ�, ��Сֵ-180��

    PropertyBuilderByName(bool,FixedResolution,private)///< ���úͻ�ȡ�����Ƿ��ǹ̶��Ƕȷֱ���
    PropertyBuilderByName(bool,Reversion, private)///< ���úͻ�ȡ�Ƿ���ת����180��
    PropertyBuilderByName(bool,AutoReconnect, private)///< �����쳣�Ƿ�����������


    PropertyBuilderByName(int,SerialBaudrate,private)///< ���úͻ�ȡ����ͨѶ������
    PropertyBuilderByName(std::string,SerialPort,private)///< ���úͻ�ȡ����˿ں�
    PropertyBuilderByName(std::vector<float>,IgnoreArray,private)///< ���úͻ�ȡ�����޳���


public:
	CYdLidar(); //!< Constructor
	virtual ~CYdLidar();  //!< Destructor: turns the laser off.

    bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

    // Return true if laser data acquistion succeeds, If it's not
    /*bool doProcessSimple(mrpt::obs::CObservation2DRangeScan &scan,
                         bool &hardwareError, bool if_compensate);*/
    bool doProcessSimple(LaserScan &outscan, LaserScanMsg& msg, bool &hardwareError);

    //Turn on the motor enable
	bool  turnOn();  //!< See base class docs
    //Turn off the motor enable and close the scan
	bool  turnOff(); //!< See base class docs

    /** Returns true if the device is in good health, If it's not*/
	bool getDeviceHealth() const;

    /** Returns true if the device information is correct, If it's not*/
    bool getDeviceInfo(int &type);

    /** Retruns true if the heartbeat function is set to heart is successful, If it's not*/
    bool checkHeartBeat() const;

    /** Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
    bool checkScanFrequency();

    //Turn off lidar connection
    void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

    //----Add by heather.yang
    void setRotationVelocity(const double w) {w_ = w;} 

    //void compensateLidar(mrpt::obs::CObservation2DRangeScan &scan);
    /* #ifdef AI_DEBUG */
    /* mrpt::utils::CFileGZOutputStream  out_file_; */
    /* #endif */
    //---
    inline double DEG2RAD(const double x) { return x * M_PI / 180.0;}
protected:
    /** Returns true if communication has been established with the device. If it's not,
      *  try to create a comms channel.
      * \return false on error.
      */
    bool  checkCOMMs();

    /** Returns true if health status and device information has been obtained with the device. If it's not,
      * \return false on error.
      */
    bool  checkStatus();

    /** Returns true if the normal scan runs with the device. If it's not,
      * \return false on error.
      */
    bool checkHardware();

private:
    bool isScanning;
    bool isConnected;
    int node_counts ;
    double each_angle;
    YDlidarDriver *lidarPtr;
    double w_; //Add by heather.yang

};	// End of class

