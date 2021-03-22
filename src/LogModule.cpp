#include "LogModule.h"

#include <time.h>
#include <string.h>
//string 相互转换 wstring
#ifndef LINUX
#include <comutil.h>  
#pragma comment(lib, "comsuppw.lib")
#else
#include <stdlib.h>
#endif

//使用vswprintf会出现奔溃的情况如果，传入数据大于 VA_PARAMETER_MAX 就会出现崩溃
#define  VA_PARAMETER_MAX  1024 * 2

LogModule* LogModule::s_pLogModule = NULL;


LogModule* LogModule::getInstance(__in const char* fileName, __in const char* funcName, __in int lineNo,LogLevel level,ILogRealization*plog /*= NULL*/)
{
	if (s_pLogModule == NULL)
	{
		s_pLogModule = new LogModule();
	}
	s_pLogModule->m_logInfo.strFileName = fileName;
	s_pLogModule->m_logInfo.strFuncName = funcName;
	s_pLogModule->m_logInfo.nLineNo = lineNo;
	s_pLogModule->m_logInfo.Loglevel = level;
	
	if (plog != NULL)
	{
		s_pLogModule->m_pRealization->free();
		s_pLogModule->m_pRealization = plog;
	}
	return s_pLogModule;
}

LogModule::LogModule()
{
	m_logInfo.nLineNo = -1;
	m_logInfo.strFileName = "";
	m_logInfo.strFuncName = "";
#ifndef LINUX
	m_pRealization = new LogOutputString();
#else
	m_pRealization = new LogPrint();
#endif
	initLock();
}

LogModule::~LogModule()
{
	realseLock();
}

void LogModule::LogPrintA(const char* format,...)
{
	lock();
	if (m_pRealization)
	{
		string strTemp;
        //LogLevel
        strTemp.append(getFormatValue(m_logInfo.Loglevel));
		//时间
		strTemp.append(getFormatValue(getCurrentTime()));
		//文件名称
		strTemp.append(getFormatValue(m_logInfo.strFileName));
		strTemp.append(getFormatValue(m_logInfo.strFuncName));
		//行号
		strTemp.append(getFormatValue(m_logInfo.nLineNo));

		va_list ptr;
		va_start(ptr, format);
        char cValue[VA_PARAMETER_MAX] = {0};
        vsnprintf(cValue,sizeof(cValue),format,ptr);
		va_end(ptr);

		strTemp.append(getFormatValue(cValue));

		m_pRealization->LogPrintA(strTemp.c_str());

	}
	unlock();
}

void LogModule::LogPrintW(const wchar_t* format,...)
{
	lock();
	if (m_pRealization)
	{

		wstring strTemp;
		//时间
		strTemp.append(s2ws(getFormatValue(getCurrentTime())));
		//文件名称
		strTemp.append(s2ws(getFormatValue(m_logInfo.strFileName)));
		strTemp.append(s2ws(getFormatValue(m_logInfo.strFuncName)));
		//行号
		strTemp.append(s2ws(getFormatValue(m_logInfo.nLineNo)));

		va_list ptr;
		va_start(ptr, format);
        wchar_t cValue[VA_PARAMETER_MAX] = {0};
        vswprintf(cValue,VA_PARAMETER_MAX,format,ptr);
		va_end(ptr);

		strTemp.append(s2ws("["));
		strTemp.append(cValue);
		strTemp.append(s2ws("]"));

		m_pRealization->LogPrintW(strTemp.c_str());

	}
	unlock();
}

void LogModule::initLock()
{
#ifndef LINUX
	InitializeCriticalSection(&m_Mutex);
#else
    pthread_mutex_init(&m_Mutex,NULL);
#endif
	
}

void LogModule::realseLock()
{
#ifndef LINUX
	DeleteCriticalSection(&m_Mutex);
#else
	pthread_mutex_unlock(&m_Mutex);
#endif
}

void LogModule::lock()
{
#ifndef LINUX
	EnterCriticalSection(&m_Mutex);
#else
	pthread_mutex_lock(&m_Mutex);
#endif
	
}

void LogModule::unlock()
{
#ifndef LINUX
	LeaveCriticalSection(&m_Mutex);
#else
	pthread_mutex_unlock(&m_Mutex);
#endif
}

std::string LogModule::getCurrentTime()
{
	string currTime;
	//Current date/time based on current time
	time_t now = time(0); 
	// Convert current time to string
	currTime.assign(ctime(&now));

	// Last charactor of currentTime is "\n", so remove it
	string currentTime = currTime.substr(0, currTime.size()-1);
	return currentTime;
}

std::string LogModule::getFormatValue(string strValue)
{
	string strTemp;
	strTemp.append("[");
	strTemp.append(strValue);
	strTemp.append("]");
	return strTemp;
}

std::string LogModule::getFormatValue(int nValue)
{
	string strTemp;
	strTemp.append("[");
	char cValue[16];
	sprintf(cValue,"%d",nValue);
	strTemp.append(cValue);
	strTemp.append("]");
	return strTemp;
}

std::string LogModule::ws2s(const wstring& ws)
{
	string result = "";
#ifndef LINUX
	_bstr_t t = ws.c_str();  
	char* pchar = (char*)t;  
	result = pchar;  
#else
	long  wslen = ws.length();
	char* p = new char[wslen + 1];
	if (!p)
	{
		return result;
	}
	memset(p,0,sizeof(p));
	int nRet = wcstombs(p,ws.c_str(),wslen);
	result = p;
	if (nRet == -1)
	{
		result = "";
	}

	if (p)
	{
		delete[] p;
	}
#endif
	return result;  
}

std::wstring LogModule::s2ws(const string& s)
{
	wstring result = s2ws("");
#ifndef LINUX
	_bstr_t t = s.c_str();  
	wchar_t* pwchar = (wchar_t*)t;  
	result = pwchar;  
#else
	long  slen = s.length();
	wchar_t* p = new wchar_t[slen + 1];
	if (!p)
	{
		return result;
	}
	memset(p,0,sizeof(p));
	int nRet = mbstowcs(p,s.c_str(),slen);
	result = p;
	if (nRet == -1)
	{
		result = s2ws("");
	}
	if (p)
	{
		delete[] p;
	}

#endif
	return result; 
}

void LogPrint::Initializion(const char* path)
{
    return ;
}

void LogPrint::free(ILogRealization *pLog)
{
    LogPrint* pOutput = static_cast<LogPrint*>(pLog);
    if (pOutput != NULL)
    {
        delete pOutput;
    }
}

void LogPrint::LogPrintA(const char* str)
{

    FILE *fp = fopen(LOGFILEPATH,"a");
    if(!fp)
    {
        printf("%s\n","log.txt open filed!");
        return ;
    }
    printf_s(fp,str);
    printf_s(fp,"\r\n");
    fclose(fp);
}

 void LogPrint::LogPrintW(const wchar_t* str)
{
    wprintf_s(str);
    wprintf_s(L"\r\n");
}
