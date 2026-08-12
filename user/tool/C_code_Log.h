#ifndef _C_CODE_LOG_H_
#define _C_CODE_LOG_H_
#include "printf.h"
#if (__ARMCC_VERSION >= 6010050)           /* 使用AC6编译器时 */
#else
#pragma diag_suppress 870 //防止printf打印中文时报警告
#endif
/*
*示例：LOG("INFO", "This is a %s message with number %d", "test", 123);
*输出：[INFO|main@your_file.c:your_line_number] This is a test message with number 123
*
*
*/
#define LOG_ENABLE 1

#define USE_SERIAL          1   // 日志使用串口输出(需要使用堆栈)
#define USE_SERIAL_BLOCK    2   // 日志使用串口阻塞输出
#define USE_SEGGER_RTT      3   // 日志使用RTT输出
#define LOG_MODE    USE_SERIAL  // 选择日志输出模式

#if LOG_MODE == USE_SERIAL
    #include "log.h"
#elif LOG_MODE == USE_SERIAL_BLOCK
    #define LOG_LEVEL 2    //ERROR, INFO
    // #include <stdio.h>
#elif LOG_MODE == USE_SEGGER_RTT
    #define LOG_LEVEL 2    //ERROR, INFO
    #include "SEGGER_RTT.h"
#else
    #error "LOG_MODE must be USE_SERIAL or USE_SERIAL_BLOCK or USE_SEGGER_RTT"
#endif

static inline void c_code_log_init(void)
{
#if (LOG_MODE == USE_SERIAL)    // 使用串口输出
    log_set_all_fmt();
    log_clear_fmt(LOG_LEVEL_DEBUG, LOG_FMT_ALL); // 清除DEBUG等级的文件名和函数名输出
    log_init();
#elif (LOG_MODE == USE_SEGGER_RTT)    // 使用RTT输出
    SEGGER_RTT_Init();
#endif
}

// 尽量不要使用LOG()，因为串口输出和RTT输出的不一样
#if LOG_ENABLE == 1
    #if (LOG_MODE == USE_SERIAL_BLOCK)    // 使用串口输出
        #define LOG(level, format, ...) \
                printf("[%s|%s@%s:%d] " format "\r\n", \
                    level, __func__, __FILE__, __LINE__, ##__VA_ARGS__ )
    #endif
    #if (LOG_MODE == USE_SEGGER_RTT)    // 使用RTT输出
        #define LOG(level, color, format, ...) \
                SEGGER_RTT_printf(0,"  %s[%s|%s@%s:%d]" format "\r\n%s", \
                                level,                                  \
                                color,                                  \
                                __func__, __FILE__, __LINE__,           \
                                ##__VA_ARGS__,                          \
                                RTT_CTRL_RESET)
    #endif
#else
    #define LOG(level, format, ...) 
#endif

#if LOG_LEVEL == 1
    #if (LOG_MODE == USE_SERIAL_BLOCK)    // 使用串口输出
        #define LOG_ERROR(format, ...) LOG("ERROR", format, ##__VA_ARGS__)
    #endif
    #if (LOG_MODE == USE_SEGGER_RTT)    // 使用RTT输出
        #define LOG_ERROR(format, ...) LOG(RTT_CTRL_TEXT_BRIGHT_RED, "ERROR", format, ##__VA_ARGS__)
    #endif
    #define LOG_INFO(format, ...) 
    #endif
#if LOG_LEVEL == 2
    #if (LOG_MODE == USE_SERIAL_BLOCK)    // 使用串口阻塞输出
        #define LOG_ERROR(format, ...) LOG(("ERROR"), format, ##__VA_ARGS__) //“ERROR”加上括号，不然用不了
        #define LOG_INFO(format, ...) LOG(("INFO"), format, ##__VA_ARGS__)
    #endif
    #if (LOG_MODE == USE_SEGGER_RTT)    // 使用RTT输出
        #define LOG_ERROR(format, ...) LOG(RTT_CTRL_TEXT_BRIGHT_RED, ("ERROR"), format, ##__VA_ARGS__) //“ERROR”加上括号，不然用不了
        #define LOG_INFO(format, ...) LOG(RTT_CTRL_TEXT_BRIGHT_GREEN, ("INFO"), format, ##__VA_ARGS__)
    #endif
#endif

#endif // !_C_CODE_LOG_H_
