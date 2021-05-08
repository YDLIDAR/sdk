#ifndef YDLIDAR_DRIVER_H
#define YDLIDAR_DRIVER_H
#include <stdlib.h>
#include <map>
#include "serial.h"
#include "locker.h"
#include "thread.h"
#include "ydlidar_protocol.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif

using namespace std;
using namespace serial;

namespace ydlidar{
std::string format(const char *fmt, ...);

	class YDlidarDriver
	{
	public:
        /**
        * A constructor.
        * A more elaborate description of the constructor.
        */
        YDlidarDriver();

        /**
        * A destructor.
        * A more elaborate description of the destructor.
        */
         virtual ~YDlidarDriver();

		/**
		* @brief �����״� \n
    	* ���ӳɹ��󣬱���ʹ��::disconnect�����ر�
    	* @param[in] port_path    ���ں�
    	* @param[in] fileMode    �����ʣ�YDLIDAR�״������¼��������ʣ�
    	*     115200 F4, G4C, S4A
    	*     128000 X4
    	*     153600 S4B
    	*     230600 F4PRO, G4
    	* @return ��������״̬
		* @retval 0     �ɹ�
    	* @retval < 0   ʧ��
    	* @note���ӳɹ��󣬱���ʹ��::disconnect�����ر�
    	* @see ����::YDlidarDriver::disconnect (��::����ָ�������ӹ���,���Կ��ĵ����disconnect�����,�����������ת��disconnect.)
    	*/
		result_t connect(const char * port_path, uint32_t baudrate);

		/**
		* @brief �Ͽ��״����� 
    	*/
		void disconnect();

		/**
		* @brief ��ȡ��ǰSDK�汾�� \n
    	* ��̬����
    	* @return ���ص�ǰSKD �汾��
    	*/
		static std::string getSDKVersion();

        /**
         * @brief ��ȡ�״��б�
         * ��̬����
         * @return map:��һ���������״����, �ڶ��������ǵ�ǰ���ں�
         */
        static std::map<std::string, std::string> lidarPortList();

		/**
		* @brief �����״�״̬ \n
    	* @return ��������״̬
		* @retval true     �ɹ�
    	* @retval false    ʧ��
    	*/
        bool isconnected() const;

        /**
        * @brief �����״��쳣�Զ��������� \n
        * @param[in] enable    �Ƿ����Զ�����:
        *     true	����
        *	  false �ر�
        */
        void setAutoReconnect(const bool& enable);


		/**
		* @brief ����ɨ�� \n
    	* @param[in] force    ɨ��ģʽ
    	* @param[in] timeout  ��ʱʱ��  
    	* @return ����ִ�н��
    	* @retval RESULT_OK       �����ɹ�
    	* @retval RESULT_FAILE    ����ʧ��
		* @note ֻ�ÿ���һ�γɹ�����
    	*/
		result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

		/**
		* @brief �ر�ɨ�� \n
    	* @return ����ִ�н��
    	* @retval RESULT_OK       �رճɹ�
    	* @retval RESULT_FAILE    �ر�ʧ��
    	*/
		result_t stop();

		
		/**
		* @brief ��ȡ�������� \n
    	* @param[in] nodebuffer �������Ϣ
		* @param[in] count      һȦ�������
    	* @param[in] timeout    ��ʱʱ��  
    	* @return ����ִ�н��
    	* @retval RESULT_OK       ��ȡ�ɹ�
    	* @retval RESULT_FAILE    ��ȡʧ��
		* @note ��ȡ֮ǰ������ʹ��::startScan��������ɨ��
    	*/
		result_t grabScanData(node_info * nodebuffer, size_t & count, uint32_t timeout = DEFAULT_TIMEOUT) ;


		/**
		* @brief ��������Ƕ� \n
		* �ѽǶ�������0��360��֮��
    	* @param[in] nodebuffer �������Ϣ
		* @param[in] count      һȦ�������
    	* @return ����ִ�н��
    	* @retval RESULT_OK       �ɹ�
    	* @retval RESULT_FAILE    ʧ��
		* @note ����֮ǰ������ʹ��::grabScanData������ȡ�������ݳɹ�
    	*/
		result_t ascendScanData(node_info * nodebuffer, size_t count);

		/**	
		* @brief �򿪵�� \n
    	* @return ����ִ�н��
    	* @retval RESULT_OK       �ɹ�
    	* @retval RESULT_FAILE    ʧ��
    	*/
		result_t startMotor();

		/**	
		* @brief �رյ�� \n
    	* @return ����ִ�н��
    	* @retval RESULT_OK       �ɹ�
    	* @retval RESULT_FAILE    ʧ��
    	*/
		result_t stopMotor();

	protected:

		/**
		* @brief ���������״������߳� \n
		* @note ���������״������߳�֮ǰ������ʹ��::startScan��������ɨͼ�ɹ�
    	*/
		result_t createThread();


        /**
        * @brief �������ӿ���ɨ�� \n
        * @param[in] force    ɨ��ģʽ
        * @param[in] timeout  ��ʱʱ��
        * @return ����ִ�н��
        * @retval RESULT_OK       �����ɹ�
        * @retval RESULT_FAILE    ����ʧ��
        * @note sdk �Զ��������ӵ���
        */
        result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;


		/**
		* @brief ����������� \n
    	* @param[in] node ����󼤹����Ϣ
		* @param[in] timeout     ��ʱʱ��
    	*/
		result_t waitPackage(node_info * node, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief �������ݵ��״� \n
    	* @param[in] nodebuffer ������Ϣָ��
    	* @param[in] count      ���������С	
		* @param[in] timeout      ��ʱʱ��	
		* @return ����ִ�н��
    	* @retval RESULT_OK       �ɹ�
		* @retval RESULT_TIMEOUT  �ȴ���ʱ
    	* @retval RESULT_FAILE    ʧ��	
    	*/
		result_t waitScanData(node_info * nodebuffer, size_t & count, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief �������ݽ����߳� \n
    	*/
		int cacheScanData();

		/**
		* @brief �������ݵ��״� \n
    	* @param[in] cmd 	 ������
    	* @param[in] payload      payload	
		* @param[in] payloadsize      payloadsize	
		* @return ����ִ�н��
    	* @retval RESULT_OK       �ɹ�
    	* @retval RESULT_FAILE    ʧ��	
    	*/
		result_t sendCommand(uint8_t cmd, const void * payload = NULL, size_t payloadsize = 0);

		/**
		* @brief �ȴ��������ݰ�ͷ \n
    	* @param[in] header 	 ��ͷ
    	* @param[in] timeout      ��ʱʱ��	
		* @return ����ִ�н��
    	* @retval RESULT_OK       ��ȡ�ɹ�
		* @retval RESULT_TIMEOUT  �ȴ���ʱ
    	* @retval RESULT_FAILE    ��ȡʧ��	
		* @note ��timeout = -1 ʱ, ��һֱ�ȴ�
    	*/
		result_t waitResponseHeader(lidar_ans_header * header, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief �ȴ��̶������������� \n
    	* @param[in] data_count 	 �ȴ����ݴ�С
    	* @param[in] timeout    	 �ȴ�ʱ��	
		* @param[in] returned_size   ʵ�����ݴ�С	
		* @return ����ִ�н��
    	* @retval RESULT_OK       ��ȡ�ɹ�
		* @retval RESULT_TIMEOUT  �ȴ���ʱ
    	* @retval RESULT_FAILE    ��ȡʧ��	
		* @note ��timeout = -1 ʱ, ��һֱ�ȴ�
    	*/
        result_t waitForData(size_t data_count,uint32_t timeout = DEFAULT_TIMEOUT, size_t * returned_size = NULL);

		/**
		* @brief ��ȡ�������� \n
    	* @param[in] data 	 ����ָ��
    	* @param[in] size    ���ݴ�С	
		* @return ����ִ�н��
    	* @retval RESULT_OK       ��ȡ�ɹ�
    	* @retval RESULT_FAILE    ��ȡʧ��	
    	*/
		result_t getData(uint8_t * data, size_t size);

		/**
		* @brief ���ڷ������� \n
    	* @param[in] data 	 ��������ָ��
    	* @param[in] size    ���ݴ�С	
		* @return ����ִ�н��
    	* @retval RESULT_OK       ���ͳɹ�
    	* @retval RESULT_FAILE    ����ʧ��	
    	*/
		result_t sendData(const uint8_t * data, size_t size);

		/**
		* @brief �ر����ݻ�ȡͨ�� \n
    	*/
		void disableDataGrabbing();

		/**
		* @brief ���ô���DTR \n
    	*/
		void setDTR();

		/**
		* @brief �������DTR \n
    	*/
		void clearDTR();


	public:
        bool     isConnected;  ///< ��������״��
        bool     isScanning;   ///< ɨͼ״̬
        bool     isAutoReconnect;  ///< �쳣�Զ���������
        bool     isAutoconnting;  ///< �Ƿ������Զ�������


		enum {
			DEFAULT_TIMEOUT = 2000,    /**< Ĭ�ϳ�ʱʱ��. */ 
			DEFAULT_HEART_BEAT = 1000, /**< Ĭ�ϼ����繦��ʱ��. */ 
			MAX_SCAN_NODES = 2048,	   /**< ���ɨ�����. */ 
            DEFAULT_TIMEOUT_COUNT = 10,
		};
		enum { 
			YDLIDAR_F4=1, /**< F4�״��ͺŴ���. */ 
			YDLIDAR_T1=2, /**< T1�״��ͺŴ���. */ 
			YDLIDAR_F2=3, /**< F2�״��ͺŴ���. */ 
			YDLIDAR_S4=4, /**< S4�״��ͺŴ���. */ 
			YDLIDAR_G4=5, /**< G4�״��ͺŴ���. */ 
			YDLIDAR_X4=6, /**< X4�״��ͺŴ���. */ 
			YDLIDAR_F4PRO=6, /**< F4PRO�״��ͺŴ���. */ 
			YDLIDAR_G4C=9, /**< G4C�״��ͺŴ���. */ 

		};
		node_info      scan_node_buf[2048];  ///< �������Ϣ
		size_t         scan_node_count;      ///< �������
		Event          _dataEvent;			 ///< ����ͬ���¼�
		Locker         _lock;				///< �߳���
        Locker         _serial_lock;		///< ������
		Thread 	       _thread;				///< �߳�id

	private:
        serial::Serial *_serial;			///< ����
        node_packages packages;

        float       IntervalSampleAngle;
        float       IntervalSampleAngle_LastPackage;
        int         PackageSampleBytes;             ///< һ���������ļ������
        bool        isSupportMotorCtrl;			///< �Ƿ�֧�ֵ������
        bool        CheckSunResult;
        uint32_t    m_baudrate;					///< ������
        uint64_t    m_ns;						///< ʱ���
        uint64_t    m_last_ns;						///< ʱ���
        uint32_t    m_signalpointTime;			///< ���������ʱ����
        uint32_t    trans_delay;				///< ���ڴ���һ��byteʱ��
        uint16_t    package_Sample_Index;
        uint16_t    FirstSampleAngle;
        uint16_t    LastSampleAngle;
        uint16_t    CheckSun;
        uint16_t    CheckSunCal;
        uint16_t    SampleNumlAndCTCal;
        uint16_t    LastSampleAngleCal;
        uint16_t    Valu8Tou16;

        std::string serial_port;///< �״�˿�      

	};
}

#endif // YDLIDAR_DRIVER_H
