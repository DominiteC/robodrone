#include "log.h"
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include "printf.h"
#include "log_port.h"

#if (LOG_BUFFER_USE_STACK == 0)
char log_buffer[LOG_BUFFER_SIZE] = {0};// 全局日志sprintf缓冲区
#endif
bool log_is_disable = false;
uint16_t log_fmt_set[LOG_LEVEL_NUM] = {0};//不同的日志级别可以设置不同的输出格式
LogOut_Handle* log_channel_Handle[LOG_CHANNEL_NUM]; // 各个日志通道的句柄

/**
 * @brief 设置日志输出所有格式
 * 
 */
void log_set_all_fmt(void)
{
    log_disable();
    for (uint8_t i = 0; i < LOG_LEVEL_NUM; i++)
    {
        log_fmt_set[i] = LOG_FMT_ALL;
    }
    log_enable();
}

/**
 * @brief 设置日志输出格式
 * 
 * @param level 要设置的日志等级
 * @param type 设置的格式
 */
void log_set_fmt(uint8_t level, uint16_t type)
{
    log_fmt_set[level] |= type;
}

/**
 * @brief 清除特定的日志输出格式
 * 
 * @param level 要清除的日志等级
 * @param type 清除的格式
 */
void log_clear_fmt(uint8_t level, uint16_t type)
{
    log_fmt_set[level] &= ~type;
}

/**
 * @brief 判断日志输出格式是否设置
 * 
 * @param level 日志等级
 * @param type 日志格式
 * @return uint8_t 1:设置 0:未设置
 */
uint8_t log_fmt_is_set(uint8_t level, LogFmtType type)
{
    return log_fmt_set[level] & type;
}

/**
 * @brief 设置日志输出通道
 * 
 * @param channel 通道号
 * @param handle 日志输出的句柄
 */
void log_set_channel(uint8_t channel, LogOut_Handle* handle)
{
    if (channel < LOG_CHANNEL_NUM)
    {
        log_channel_Handle[channel] = handle;
    }
}

/**
 * @brief 初始化log
 * 
 */
void log_init(void)
{
    log_port_init();
    LOG_INFO("Log init success");
}

/**
 * @brief 异步输出日志
 * 
 * @param level 日志等级
 * @param file 文件名
 * @param func 函数名
 * @param line 行号
 * @param format 格式化字符串
 * @return uint8_t 0:成功 其他失败
 */
uint8_t log_output_async(LogMode mode, uint8_t channel ,uint8_t level, const char *file, const char *func,
                    const long line, const char *format, ...)
{
    uint16_t len = 0;
    uint16_t res_len;
    if (log_is_disable)
    {
        return 4;
    }
    if(channel >= LOG_CHANNEL_NUM || log_channel_Handle[channel] == NULL) {
        return 6; // 通道未设置
    }
#if (LOG_BUFFER_USE_STACK == 1)
    char log_buffer[LOG_BUFFER_SIZE];
#elif (LOG_BUFFER_USE_STACK == 0)
    // 如果使用全局变量作为缓冲区，建议添加锁机制，防止多线程同时使用log_buffer 
    if(log_sprintf_buffer_lock(UINT32_MAX) != 0 ) {
        return 5; // 获取锁失败
    }
#endif
#define MIN(x,y) ((x) < (y) ? (x) : (y))
    va_list args;
    va_start(args, format);
    // 判断并输出日志等级
    if (log_fmt_is_set(level, LOG_FMT_LEVEL))
    {
        const char *level_str = NULL;
        switch (level)
        {
        case LOG_LEVEL_DEBUG:
            level_str = "DEBUG";
            break;
        case LOG_LEVEL_INFO:
            level_str = "INFO";
            break;
        case LOG_LEVEL_WARN:
            level_str = "WARN";
            break;
        case LOG_LEVEL_ERROR:
            level_str = "ERROR";
            break;
        case LOG_LEVEL_FATAL:
            level_str = "FATAL";
            break;
        default:
            level_str = "UNKNOWN";
            break;
        }
        res_len = snprintf(log_buffer, LOG_BUFFER_SIZE, "[%s] ", level_str);
        len += MIN(res_len, LOG_BUFFER_SIZE - len);
    }
    // 输出时间
    if (log_fmt_is_set(level, LOG_FMT_TIME))
    {
        res_len = snprintf(log_buffer + len, LOG_BUFFER_SIZE - len, "[%u] ", log_port_get_time());
        len += MIN(res_len, LOG_BUFFER_SIZE - len);
    }
    // 输出文件名
    if (log_fmt_is_set(level, LOG_FMT_FILE))
    {
        res_len = snprintf(log_buffer + len, LOG_BUFFER_SIZE - len, "%s|", file);
        len += MIN(res_len, LOG_BUFFER_SIZE - len);
    }
    // 输出函数名
    if (log_fmt_is_set(level, LOG_FMT_FUNC))
    {
        res_len = snprintf(log_buffer + len, LOG_BUFFER_SIZE - len, "%s():", func);
        len += MIN(res_len, LOG_BUFFER_SIZE - len);
    }
    // 输出行号
    if (log_fmt_is_set(level, LOG_FMT_LINE))
    {
        res_len = snprintf(log_buffer + len, LOG_BUFFER_SIZE - len, "%ld: ", line);
        len += MIN(res_len, LOG_BUFFER_SIZE - len);
    }
    res_len = vsnprintf(log_buffer + len, LOG_BUFFER_SIZE - len, format, args);    // 输出剩余内容
    len += MIN(res_len, LOG_BUFFER_SIZE - len);
    va_end(args);
    int res;
    if (mode == LOG_MODE_NORMAL) {
        res = log_port_output(log_channel_Handle[channel],(uint8_t *)log_buffer, len);
    } else {
        res = log_port_output_IT(log_channel_Handle[channel],(uint8_t *)log_buffer, len);
    }
#if (LOG_BUFFER_USE_STACK == 0)
    log_sprintf_buffer_unlock();
#endif
    return res;
}

/**
 * @brief 关闭日志
 * 
 */
void log_disable(void)
{
    log_is_disable = true;
}

/**
 * @brief 开启日志
 * 
 */
void log_enable(void)
{
    log_is_disable = false;
}
