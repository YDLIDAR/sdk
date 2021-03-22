#ifndef  __LOGMODULE_H_
#define  __LOGMODULE_H_


#define LINUX

#define LOGFILEPATH "/var/log/s2_3k_v2_sdk.log"

#include <stdio.h>
#include <string>
#include <stdlib.h>
using namespace std;

#ifndef LINUX
#include <windows.h>
#else
//#include <pthread.h>
#include <stdarg.h>
#define printf_s(fileptr,str)  (fprintf(fileptr,str))
#define wprintf_s(str) wprintf(str)
#define __in
#endif // ??????????????????????


struct LogVersion
{
    int		nVersion;     //????��??
    string  strDescruble; //????
};


class ILogRealization
{
public:
    virtual void Initializion(const char* path = NULL) = 0;
    virtual void LogPrintW(const wchar_t* str) = 0;
    virtual void LogPrintA(const char* str) = 0;
    void free(){
        free(this);
        //this = NULL;
    };
private:
    virtual void free(ILogRealization *pLog) = 0 ;
};


#define  ILOGFREE(LogRealizationClass)  virtual void free(ILogRealization* pLog)	\
{																					\
    LogRealizationClass* pRealization = static_cast<LogRealizationClass*>(pLog);    \
    if (pRealization != NULL){ delete pRealization;}								\
}

class LogPrint
    :public ILogRealization
{

public:
    virtual void Initializion(const char* path = NULL);
    virtual void free(ILogRealization *pLog);

    virtual void LogPrintA(const char* str);

    virtual void LogPrintW(const wchar_t* str);

};

#ifndef LINUX
class LogOutputString
    :public ILogRealization
{

public:
    virtual void Initializion(const char* path = NULL)
    {
        return ;
    }

    virtual void LogPrintW(const wchar_t* str)
    {
        OutputDebugStringW(str);
        OutputDebugStringW(L"\r\n");
    }

    virtual void LogPrintA(const char* str)
    {
        OutputDebugString((LPCTSTR)str);
        OutputDebugString("\r\n");
    }

    ILOGFREE(LogOutputString)
/*
    virtual void free(ILogRealization *pLog)
    {
        LogOutputString* pOutput = static_cast<LogOutputString*>(pLog);
        if (pOutput != NULL)
        {
            delete pOutput;
        }
    }
*/
};
#endif


class LogModule
{
public:

    enum LogLevel
    {
        DEBUG_LEVEL,
        WARNING_LEVEL,
        ERROR_LEVEL,
        INFO_LEVEL
    };

    struct LOGMODULE_INFO
    {
        LogLevel	Loglevel;       //LogLevel
        string		strFileName;  //???????
        string		strFuncName;  //????????
        int			nLineNo;	  //????��?
    }m_logInfo;

    ILogRealization* m_pRealization; //????????
public:
    static  LogModule* getInstance( __in const char* fileName, __in const char* funcName,__in int lineNo,LogLevel level,ILogRealization*plog = NULL );

    void LogPrintA(const char* format,...);

    void LogPrintW(const wchar_t* format,...);

private:
    LogModule();

    ~LogModule();

    void initLock();

    void realseLock();

    void lock();

    void unlock();

    string getCurrentTime();

    string getFormatValue(string strValue);

    string  getFormatValue(int nValue);

    string ws2s(const wstring& ws);

    wstring s2ws(const string& s);

    static LogModule*  s_pLogModule;

#ifndef LINUX
    //??
    CRITICAL_SECTION   m_Mutex;
#else
    pthread_mutex_t    m_Mutex;
#endif


};

#ifndef  UNICODE
#define  LOG(level,format,...)   LogModule::getInstance(__FILE__, __FUNCTION__, __LINE__,level)->LogPrintA(format,__VA_ARGS__);
#else
#define  LOG(level,format,...)   LogModule::getInstance(__FILE__, __FUNCTION__, __LINE__,level)->LogPrintW(format,__VA_ARGS__)
#endif


#define  LOG_DEBUG(format,...)   LOG(LogModule::DEBUG_LEVEL,format,__VA_ARGS__)
#define  LOG_INFO(format,...)    LOG(LogModule::INFO_LEVEL,format,__VA_ARGS__)
#define  LOG_ERROR(format,...)   LOG(LogModule::ERROR_LEVEL,format,__VA_ARGS__)



#endif
