/*
 * app_system.h
 * 声明 app 层系统初始化入口，供启动编排模块调用。
 */
#ifndef __APP_SYSTEM_H
#define __APP_SYSTEM_H

/* 执行系统、设备和业务模块初始化，初始化顺序在实现文件中集中维护。 */
void App_InitSystemModules(void);

#endif /* __APP_SYSTEM_H */
