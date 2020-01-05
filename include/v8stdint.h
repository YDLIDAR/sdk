#ifndef V8STDINT_H_
#define V8STDINT_H_

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <signal.h>
#include <cerrno>
#include <stdexcept>
#include <csignal>
#include <sys/stat.h>
#if defined(_MSC_VER)
#include <io.h>
#endif

#if !defined(_MSC_VER)
#include <unistd.h>
#endif

#define UNUSED(x) (void)x

#if !defined(_MSC_VER)
#	define _access access
#endif

/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Type Definition Macros                                                    */
/*                                                                           */
/*---------------------------------------------------------------------------*/
#ifndef __WORDSIZE
/* Assume 32 */
#define __WORDSIZE 32
#endif

#if defined(__linux__) || defined(_DARWIN)
#include <stdint.h>
typedef int            SOCKET;
#endif

#if defined(_WIN32) && !defined(__MINGW32__)
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef short int16_t;
typedef unsigned short uint16_t;
typedef int int32_t;
typedef unsigned int uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
#else

#include <stdint.h>

#endif

#if defined(_WIN32)
struct iovec {
  void  *iov_base;
  size_t iov_len;
};

typedef int socklen_t;

#endif

#if defined(_WIN32)

#ifndef UINT8_MAX
#define UINT8_MAX  (UCHAR_MAX)
#endif
#ifndef UINT16_MAX
#define UINT16_MAX (USHRT_MAX)
#endif
#ifndef UINT32_MAX
#define UINT32_MAX (ULONG_MAX)
#endif

#if __WORDSIZE == 64
#define SIZE_MAX (18446744073709551615UL)
#else
#ifndef SIZE_MAX
#define SIZE_MAX (4294967295U)
#endif
#endif
#endif

#if defined(_WIN32)
#define ssize_t size_t
#endif

#define __small_endian

#ifndef __GNUC__
#define __attribute__(x)
#endif


#ifdef _AVR_
typedef uint8_t        _size_t;
#define THREAD_PROC
#elif defined (WIN64)
typedef uint64_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (WIN32)
typedef uint32_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (_M_X64)
typedef uint64_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
typedef unsigned long  _size_t;
#define THREAD_PROC
#elif defined (__ICCARM__)
typedef uint32_t       _size_t;
#define THREAD_PROC
#endif

typedef _size_t (THREAD_PROC *thread_proc_t)(void *);

typedef int32_t result_t;

#define RESULT_OK      0
#define RESULT_TIMEOUT -1
#define RESULT_FAIL    -2

#define INVALID_TIMESTAMP (0)

enum {
  YDLIDAR_TYPE_SERIAL = 0x0,
  YDLIDAR_TYPE_TCP = 0x1,
};


#define IS_OK(x)    ( (x) == RESULT_OK )
#define IS_TIMEOUT(x)  ( (x) == RESULT_TIMEOUT )
#define IS_FAIL(x)  ( (x) == RESULT_FAIL )

//Socket

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef htonll
#ifdef _BIG_ENDIAN
#define htonll(x)   (x)
#define ntohll(x)   (x)
#else
#define htonll(x)   ((((uint64)htonl(x)) << 32) + htonl(x >> 32))
#define ntohll(x)   ((((uint64)ntohl(x)) << 32) + ntohl(x >> 32))
#endif
#endif

/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Socket Macros                                                             */
/*                                                                           */
/*---------------------------------------------------------------------------*/
#if defined(_WIN32)
#define SHUT_RD                0
#define SHUT_WR                1
#define SHUT_RDWR              2
#define ACCEPT(a,b,c)          accept(a,b,c)
#define CONNECT(a,b,c)         connect(a,b,c)
#define CLOSE(a)               closesocket(a)
#define READ(a,b,c)            read(a,b,c)
#define RECV(a,b,c,d)          recv(a, (char *)b, c, d)
#define RECVFROM(a,b,c,d,e,f)  recvfrom(a, (char *)b, c, d, (sockaddr *)e, (int *)f)
#define RECV_FLAGS             MSG_WAITALL
#define SELECT(a,b,c,d,e)      select((int32_t)a,b,c,d,e)
#define SEND(a,b,c,d)          send(a, (const char *)b, (int)c, d)
#define SENDTO(a,b,c,d,e,f)    sendto(a, (const char *)b, (int)c, d, e, f)
#define SEND_FLAGS             0
#define SENDFILE(a,b,c,d)      sendfile(a, b, c, d)
#define SET_SOCKET_ERROR(x,y)  errno=y
#define SOCKET_ERROR_INTERUPT  EINTR
#define SOCKET_ERROR_TIMEDOUT  EAGAIN
#define WRITE(a,b,c)           write(a,b,c)
#define WRITEV(a,b,c)          Writev(b, c)
#define GETSOCKOPT(a,b,c,d,e)  getsockopt(a,b,c,(char *)d, (int *)e)
#define SETSOCKOPT(a,b,c,d,e)  setsockopt(a,b,c,(char *)d, (int)e)
#define GETHOSTBYNAME(a)       gethostbyname(a)
#define IOCTLSOCKET(a, b, c)   ioctlsocket(a,b,(u_long*)c)
#endif

#if defined(__linux__) || defined(_DARWIN)
#define ACCEPT(a,b,c)          accept(a,b,c)
#define CONNECT(a,b,c)         connect(a,b,c)
#define CLOSE(a)               close(a)
#define READ(a,b,c)            read(a,b,c)
#define RECV(a,b,c,d)          recv(a, (void *)b, c, d)
#define RECVFROM(a,b,c,d,e,f)  recvfrom(a, (char *)b, c, d, (sockaddr *)e, f)
#define RECV_FLAGS             MSG_WAITALL
#define SELECT(a,b,c,d,e)      select(a,b,c,d,e)
#define SEND(a,b,c,d)          send(a, (const int8_t *)b, c, d)
#define SENDTO(a,b,c,d,e,f)    sendto(a, (const int8_t *)b, c, d, e, f)
#define SEND_FLAGS             0
#define SENDFILE(a,b,c,d)      sendfile(a, b, c, d)
#define SET_SOCKET_ERROR(x,y)  errno=y
#define SOCKET_ERROR_INTERUPT  EINTR
#define SOCKET_ERROR_TIMEDOUT  EAGAIN
#define WRITE(a,b,c)           write(a,b,c)
#define WRITEV(a,b,c)          writev(a, b, c)
#define GETSOCKOPT(a,b,c,d,e)  getsockopt((int)a,(int)b,(int)c,(void *)d,(socklen_t *)e)
#define SETSOCKOPT(a,b,c,d,e)  setsockopt((int)a,(int)b,(int)c,(const void *)d,(int)e)
#define GETHOSTBYNAME(a)       gethostbyname(a)
#define IOCTLSOCKET(a, b, c)   ioctl(a,b,c)
#endif


/*---------------------------------------------------------------------------*/
/*                                                                           */
/* File Macros                                                               */
/*                                                                           */
/*---------------------------------------------------------------------------*/
#define STRUCT_STAT         struct stat
#define LSTAT(x,y)          lstat(x,y)
#define FILE_HANDLE         FILE *
#define CLEARERR(x)         clearerr(x)
#define FCLOSE(x)           fclose(x)
#define FEOF(x)             feof(x)
#define FERROR(x)           ferror(x)
#define FFLUSH(x)           fflush(x)
#define FILENO(s)           fileno(s)
#define FOPEN(x,y)          fopen(x, y)
//#define FREAD(a,b,c,d)      fread(a, b, c, d)
#define FSTAT(s, st)        fstat(FILENO(s), st)
//#define FWRITE(a,b,c,d)     fwrite(a, b, c, d)
#define STAT_BLK_SIZE(x)    ((x).st_blksize)

#define DEFAULT_CONNECTION_TIMEOUT_SEC 2
#define DEFAULT_CONNECTION_TIMEOUT_USEC 0

#define DEFAULT_REV_TIMEOUT_SEC 2
#define DEFAULT_REV_TIMEOUT_USEC 0




/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Misc Macros                                                               */
/*                                                                           */
/*---------------------------------------------------------------------------*/

#if defined(_WIN32)
#define STRTOULL(x) _atoi64(x)
#else
#define STRTOULL(x) strtoull(x, NULL, 10)
#endif

#if defined(_WIN32)
#define SNPRINTF _snprintf
#define PRINTF   printf
#define VPRINTF  vprintf
#define FPRINTF  fprintf
#else
#define SNPRINTF snprintf
#define PRINTF   printf
#define VPRINTF  vprintf
#define FPRINTF  fprintf
#endif



#define MILLISECONDS_CONVERSION 1000
#define MICROSECONDS_CONVERSION 1000000
#define NANOECONDS_CONVERSION 1000000000



// Determine if sigaction is available
#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE
#define HAS_SIGACTION
#endif


static volatile sig_atomic_t g_signal_status = 0;

#ifdef HAS_SIGACTION
static struct sigaction old_action;
#else
typedef void (* signal_handler_t)(int);
static signal_handler_t old_signal_handler = 0;
#endif

#ifdef HAS_SIGACTION
inline struct sigaction
set_sigaction(int signal_value, const struct sigaction &action)
#else
inline signal_handler_t
set_signal_handler(int signal_value, signal_handler_t signal_handler)
#endif
{
#ifdef HAS_SIGACTION
  struct sigaction old_action;
  ssize_t ret = sigaction(signal_value, &action, &old_action);

  if (ret == -1)
#else
  signal_handler_t old_signal_handler = std::signal(signal_value, signal_handler);

  // NOLINTNEXTLINE(readability/braces)
  if (old_signal_handler == SIG_ERR)
#endif
  {
    const size_t error_length = 1024;
    // NOLINTNEXTLINE(runtime/arrays)
    char error_string[error_length];
#ifndef _WIN32
#if (defined(_GNU_SOURCE) && !defined(ANDROID) &&(_POSIX_C_SOURCE >= 200112L))
    char *msg = strerror_r(errno, error_string, error_length);

    if (msg != error_string) {
      strncpy(error_string, msg, error_length);
      msg[error_length - 1] = '\0';
    }

#else
    int error_status = strerror_r(errno, error_string, error_length);

    if (error_status != 0) {
      throw std::runtime_error("Failed to get error string for errno: " +
                               std::to_string(errno));
    }

#endif
#else
    strerror_s(error_string, error_length, errno);
#endif
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::runtime_error(
      std::string("Failed to set SIGINT signal handler: (" + std::to_string(errno) + ")") +
      error_string);
    // *INDENT-ON*
  }

#ifdef HAS_SIGACTION
  return old_action;
#else
  return old_signal_handler;
#endif
}

inline void trigger_interrupt_guard_condition(int signal_value) {
  g_signal_status = signal_value;
  signal(signal_value, SIG_DFL);
}

inline void
#ifdef HAS_SIGACTION
signal_handler(int signal_value, siginfo_t *siginfo, void *context)
#else
signal_handler(int signal_value)
#endif
{
  // TODO(wjwwood): remove? move to console logging at some point?
  printf("signal_handler(%d)\n", signal_value);

#ifdef HAS_SIGACTION

  if (old_action.sa_flags & SA_SIGINFO) {
    if (old_action.sa_sigaction != NULL) {
      old_action.sa_sigaction(signal_value, siginfo, context);
    }
  } else {
    if (
      old_action.sa_handler != NULL &&  // Is set
      old_action.sa_handler != SIG_DFL &&  // Is not default
      old_action.sa_handler != SIG_IGN) { // Is not ignored
      old_action.sa_handler(signal_value);
    }
  }

#else

  if (old_signal_handler) {
    old_signal_handler(signal_value);
  }

#endif

  trigger_interrupt_guard_condition(signal_value);
}

namespace ydlidar {

inline void init(int argc, char *argv[]) {
  UNUSED(argc);
  UNUSED(argv);
#ifdef HAS_SIGACTION
  struct sigaction action;
  memset(&action, 0, sizeof(action));
  sigemptyset(&action.sa_mask);
  action.sa_sigaction = ::signal_handler;
  action.sa_flags = SA_SIGINFO;
  ::old_action = set_sigaction(SIGINT, action);
  set_sigaction(SIGTERM, action);

#else
  ::old_signal_handler = set_signal_handler(SIGINT, ::signal_handler);
  // Register an on_shutdown hook to restore the old signal handler.
#endif
}
inline bool ok() {
  return g_signal_status == 0;
}
inline void shutdownNow() {
  trigger_interrupt_guard_condition(SIGINT);
}

//inline bool fileExists(const std::string filename) {
//    return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!
//}

inline bool fileExists(const std::string filename) {
#ifdef _WIN32
  struct _stat info = {0};
  int ret = _stat(filename.c_str(), &info);
#else
  struct stat info = {0};
  int ret = stat(filename.c_str(), &info);
#endif
  return (ret == 0);
  /*return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!*/
}


}// namespace ydlidar


#endif  // V8STDINT_H_
