#ifndef __LOG_H__
#define __LOG_H__

#include <stdint.h>
#include <string.h>
#include "log_port.h"

#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_INFO 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_ERROR 3
#define LOG_LEVEL_FATAL 4

#define LOG_LEVEL_NUM 5 // 日志级别数量

#define LOG_CHANNEL_NUM 4 // 日志通道数量

// 日志配置
#ifndef ENABLE_LOG //可以通过在编译选项中或者具体的源文件在引用"log.h"前先定义ENABLE_LOG来控制日志开关
    #define ENABLE_LOG 1 // 是否开启日志, 1: 开启, 0: 关闭
#endif // !ENABLE_LOG
#ifndef LOG_LEVEL //可以通过在编译选项中或者具体的源文件在引用"log.h"前先定义LOG_LEVEL来控制日志级别
    #define LOG_LEVEL LOG_LEVEL_DEBUG // 日志级别, DEBUG: 打印所有日志, INFO: 打印INFO以上级别日志, WARN: 打印WARN以上级别日志, ERROR: 打印ERROR以上级别日志, FATAL: 打印FATAL级别日志
#endif // !LOG_LEVEL
#ifndef USING_FUNCTION //可以通过在编译选项中或者具体的源文件在引用"log.h"前先定义USING_FUNCTION来控制是否输出函数名
#define USING_FUNCTION 1 // 是否使用函数名, 1: 使用, 0: 不使用
#endif // !USING_FUNCTION
#ifndef USING_FILE //可以通过在编译选项中或者具体的源文件在引用"log.h"前先定义USING_FILE来控制是否输出文件名及路径
#define USING_FILE 1 // 是否使用文件名, 1: 使用, 0: 不使用
#endif // !USING_FILE
#ifndef USE_BASE_FILE //可以通过在编译选项中或者具体的源文件在引用"log.h"前先定义USE_BASE_FILE来控制是否只输出文件名，不输出路径
    #define USE_BASE_FILE 1 // 路径是否只输出文件名, 1: 使用, 0: 不使用
#endif // !USE_BASE_FILE
#ifndef USING_LINE //可以通过在编译选项中或者具体的源文件在引用"log.h"前先定义USING_LINE来控制是否输出行号
    #define USING_LINE 1 // 是否使用行号, 1: 使用, 0: 不使用
#endif // !USING_LINE
#ifndef DEFAULT_CHANNEL
    #define DEFAULT_CHANNEL 0 // 默认日志通道
#endif // !DEFAULT_CHANNEL

typedef enum _log_mode {
    LOG_MODE_NORMAL = 0,    // 普通模式
    LOG_MODE_IT             // 中断模式
} LogMode;

// 设置去除FILE前缀路径宏定义
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
// 是否使用函数名
#ifdef USING_FUNCTION
    #define LOG_OUTPUT_FUNC __FUNCTION__
#else
    #define LOG_OUTPUT_FUNC ""
#endif
// 是否使用文件名
#ifdef USING_FILE
    #if (USE_BASE_FILE == 1)    // 去除FILE前缀路径
        #define LOG_OUTPUT_DIR __FILENAME__
    #else
        #define LOG_OUTPUT_DIR __FILE__
    #endif
#else
    #define LOG_OUTPUT_DIR ""
#endif
// 是否使用行号
#ifdef USING_LINE
    #define LOG_OUTPUT_LINE __LINE__
#else
    #define LOG_OUTPUT_LINE 0
#endif

#if (ENABLE_LOG == 1)
    #if (LOG_LEVEL <= LOG_LEVEL_DEBUG)
        #define LOG_DEBUG_CHANNEL(channel, format, ...)  \
                log_output_async(LOG_MODE_NORMAL, channel, LOG_LEVEL_DEBUG, LOG_OUTPUT_DIR, LOG_OUTPUT_FUNC, LOG_OUTPUT_LINE, format"\r\n", ##__VA_ARGS__)
        #define LOG_DEBUG_CHANNEL_IT(channel, format, ...)   \
                log_output_async(LOG_MODE_IT, channel, LOG_LEVEL_DEBUG, LOG_OUTPUT_DIR, LOG_OUTPUT_FUNC, LOG_OUTPUT_LINE, format"\r\n", ##__VA_ARGS__)
        #define LOG_DEBUG(format, ...)  \
                LOG_DEBUG_CHANNEL(DEFAULT_CHANNEL, format, ##__VA_ARGS__)
        #define LOG_DEBUG_IT(format, ...)   \
                LOG_DEBUG_CHANNEL_IT(DEFAULT_CHANNEL, format, ##__VA_ARGS__)
    #else
        #define LOG_DEBUG_CHANNEL(channel, format, ...)
        #define LOG_DEBUG_CHANNEL_IT(channel, format, ...)
        #define LOG_DEBUG(format, ...)
        #define LOG_DEBUG_IT(format, ...)
    #endif
    #if (LOG_LEVEL <= LOG_LEVEL_INFO)
        #define LOG_INFO_CHANNEL(channel,format, ...)   \
                log_output_async(LOG_MODE_NORMAL, channel, LOG_LEVEL_INFO, LOG_OUTPUT_DIR, LOG_OUTPUT_FUNC, LOG_OUTPUT_LINE, format"\r\n", ##__VA_ARGS__)
        #define LOG_INFO_IT_CHANNEL(channel,format, ...)   \
                log_output_async(LOG_MODE_IT, channel, LOG_LEVEL_INFO, LOG_OUTPUT_DIR, LOG_OUTPUT_FUNC, LOG_OUTPUT_LINE, format"\r\n", ##__VA_ARGS__)
        #define LOG_INFO(format, ...)   \
                LOG_INFO_CHANNEL(DEFAULT_CHANNEL, format, ##__VA_ARGS__)
        #define LOG_INFO_IT(format, ...)   \
                LOG_INFO_IT_CHANNEL(DEFAULT_CHANNEL, format, ##__VA_ARGS__)
    #else
        #define LOG_INFO_CHANNEL(channel, format, ...)
        #define LOG_INFO_IT_CHANNEL(channel, format, ...)
        #define LOG_INFO(format, ...)
        #define LOG_INFO_IT(format, ...)
    #endif
    #if (LOG_LEVEL <= LOG_LEVEL_WARN)
        #define LOG_WARN_CHANNEL(channel, format, ...)   \
                log_output_async(LOG_MODE_NORMAL, channel, LOG_LEVEL_WARN, LOG_OUTPUT_DIR, LOG_OUTPUT_FUNC, LOG_OUTPUT_LINE, format"\r\n", ##__VA_ARGS__)
        #define LOG_WARN_IT_CHANNEL(channel, format, ...)   \
                log_output_async(LOG_MODE_IT, channel, LOG_LEVEL_WARN, LOG_OUTPUT_DIR, LOG_OUTPUT_FUNC, LOG_OUTPUT_LINE, format"\r\n", ##__VA_ARGS__)
        #define LOG_WARN(format, ...)   \
                LOG_WARN_CHANNEL(DEFAULT_CHANNEL, format, ##__VA_ARGS__)
        #define LOG_WARN_IT(format, ...)   \
                LOG_WARN_IT_CHANNEL(DEFAULT_CHANNEL, format, ##__VA_ARGS__)
    #else
        #define LOG_WARN_CHANNEL(channel, format, ...)
        #define LOG_WARN_IT_CHANNEL(channel, format, ...)
        #define LOG_WARN(format, ...)
        #define LOG_WARN_IT(format, ...)
    #endif
    #if (LOG_LEVEL <= LOG_LEVEL_ERROR)
        #define LOG_ERROR_CHANNEL(channel, format, ...)  \
                log_output_async(LOG_MODE_NORMAL, channel, LOG_LEVEL_ERROR, LOG_OUTPUT_DIR, LOG_OUTPUT_FUNC, LOG_OUTPUT_LINE, format"\r\n", ##__VA_ARGS__)
        #define LOG_ERROR_IT_CHANNEL(channel, format, ...)   \
                log_output_async(LOG_MODE_IT, channel, LOG_LEVEL_ERROR, LOG_OUTPUT_DIR, LOG_OUTPUT_FUNC, LOG_OUTPUT_LINE, format"\r\n", ##__VA_ARGS__)
        #define LOG_ERROR(format, ...)  \
                LOG_ERROR_CHANNEL(DEFAULT_CHANNEL, format, ##__VA_ARGS__)
        #define LOG_ERROR_IT(format, ...)   \
                LOG_ERROR_IT_CHANNEL(DEFAULT_CHANNEL, format, ##__VA_ARGS__)
    #else
        #define LOG_ERROR_CHANNEL(channel, format, ...)
        #define LOG_ERROR_IT_CHANNEL(channel, format, ...)
        #define LOG_ERROR(format, ...)
        #define LOG_ERROR_IT(format, ...)
    #endif
    #if (LOG_LEVEL <= LOG_LEVEL_FATAL)
        #define LOG_FATAL_CHANNEL(channel, format, ...)  \
                log_output_async(LOG_MODE_NORMAL, channel, LOG_LEVEL_FATAL, LOG_OUTPUT_DIR, LOG_OUTPUT_FUNC, LOG_OUTPUT_LINE, format"\r\n", ##__VA_ARGS__)
        #define LOG_FATAL_IT_CHANNEL(channel, format, ...)   \
                log_output_async(LOG_MODE_IT, channel, LOG_LEVEL_FATAL, LOG_OUTPUT_DIR, LOG_OUTPUT_FUNC, LOG_OUTPUT_LINE, format"\r\n", ##__VA_ARGS__)
        #define LOG_FATAL(format, ...)  \
                LOG_FATAL_CHANNEL(DEFAULT_CHANNEL, format, ##__VA_ARGS__)
        #define LOG_FATAL_IT(format, ...)   \
                LOG_FATAL_IT_CHANNEL(DEFAULT_CHANNEL, format, ##__VA_ARGS__)
    #else
        #define LOG_FATAL_CHANNEL(channel, format, ...)
        #define LOG_FATAL_IT_CHANNEL(channel, format, ...)
        #define LOG_FATAL(format, ...)
        #define LOG_FATAL_IT(format, ...)
    #endif
#else
    #define LOG_DEBUG_CHANNEL(channel, format, ...)
    #define LOG_DEBUG_IT_CHANNEL(channel, format, ...)
    #define LOG_INFO_CHANNEL(channel, format, ...)
    #define LOG_INFO_IT_CHANNEL(channel, format, ...)
    #define LOG_WARN_CHANNEL(channel, format, ...)
    #define LOG_WARN_IT_CHANNEL(channel, format, ...)
    #define LOG_ERROR_CHANNEL(channel, format, ...)
    #define LOG_ERROR_IT_CHANNEL(channel, format, ...)
    #define LOG_FATAL_CHANNEL(channel, format, ...)
    #define LOG_FATAL_IT_CHANNEL(channel, format, ...)
    
    #define LOG_DEBUG(format, ...)
    #define LOG_DEBUG_IT(format, ...)
    #define LOG_INFO(format, ...)
    #define LOG_INFO_IT(format, ...)
    #define LOG_WARN(format, ...)
    #define LOG_WARN_IT(format, ...)
    #define LOG_ERROR(format, ...)
    #define LOG_ERROR_IT(format, ...)
    #define LOG_FATAL(format, ...)
    #define LOG_FATAL_IT(format, ...)
#endif

typedef enum _log_fmt_type {
    LOG_FMT_LEVEL = 1 << 0, // 日志级别
    LOG_FMT_TIME = 1 << 1,  // 时间
    LOG_FMT_FILE = 1 << 2,  // 文件名
    LOG_FMT_FUNC = 1 << 3,  // 函数名
    LOG_FMT_LINE = 1 << 4,  // 行号
} LogFmtType;

#define LOG_FMT_ALL (LOG_FMT_LEVEL | LOG_FMT_TIME | LOG_FMT_FILE | LOG_FMT_FUNC | LOG_FMT_LINE) // 所有格式

void log_init(void);
void log_set_all_fmt(void);
void log_set_fmt(uint8_t level, uint16_t type);
void log_clear_fmt(uint8_t level, uint16_t type);
void log_set_channel(uint8_t channel, LogOut_Handle* handle);
uint8_t log_output_async(LogMode mode,uint8_t channel, uint8_t level, const char *file, const char *func,
                    const long line, const char *format, ...);
void log_disable(void);
void log_enable(void);

#endif
