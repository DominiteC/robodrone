#ifndef __APP_INIT_H
#define __APP_INIT_H

/*
 * app 层启动编排接口。
 * App_* 接口是当前推荐入口，RTOS_* 接口仅保留为兼容入口。
 */
void App_Init(void);
void App_InitSystemModules(void);
void App_StartTasks(void);
void App_LogStartupComplete(void);
void App_InitTask(void *param);

#endif /* __APP_INIT_H */
