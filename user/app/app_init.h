#ifndef __APP_INIT_H
#define __APP_INIT_H

/*
 * app 层启动编排接口。
 * App_InitTask 是当前启动入口，系统初始化和任务创建由 app 内部分模块负责。
 */
void App_Init(void);
void App_StartTasks(void);
void App_LogStartupComplete(void);
void App_InitTask(void *param);

#endif /* __APP_INIT_H */
