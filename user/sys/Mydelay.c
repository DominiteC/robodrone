/**
 * @file Mydelay.c
 * @author Sevenfite (lin481399413@163.com)
 * @brief 轮询用的延时函数
 * @version 0.1
 * @date 2024-06-09
 * 
 * @copyright Copyright (c) 2024
 * 移植说明：需要实现getGlobalTime()函数，返回全局时间，单位ms，全局时间为只增不减的时间
 * 
 */
//在这套代码中，大于50ms的延时函数尽量都用myDelay函数，不要用HAL_Delay函数
#include "Mydelay.h"
#include "stdlib.h"

#include "FreeRTOS.h"
#include "semphr.h"

QueueHandle_t mydelay_mutex;

void Mydelay_Init(void)
{
    mydelay_mutex = xSemaphoreCreateMutex();
}

struct MyDelayTypeDef{
    uint32_t id;
    int16_t label;   // 用于标记延时函数的标签，表示是否和之前的标签不一样,可以使用__LINE__来标记
    uint32_t targetTime;
    struct MyDelayTypeDef *next;
};
static struct MyDelayTypeDef *head = NULL;
/**
 * @brief 根据id寻找节点
 * 
 * @param id 
 * @return struct MyDelayTypeDef* 
 */
struct MyDelayTypeDef * findMyDelay(uint32_t id){
    struct MyDelayTypeDef *p = head;
    while(p != NULL){
        if(p->id == id){
            return p;
        }
        p = p->next;
    }
    return NULL;
}
#include "C_code_Log.h"
/**
 * @brief 添加延时节点
 * 
 * @param id 
 * @param targetTime 
 */
static void addMyDelay(uint32_t id, uint32_t targetTime, int16_t label){
    struct MyDelayTypeDef *p = head;
    struct MyDelayTypeDef *new = (struct MyDelayTypeDef *)malloc(sizeof(struct MyDelayTypeDef));
    if(new == NULL){
        LOG_ERROR("Mydelay malloc failed");
        return;
    }
    new->id = id;
    new->label = label;  // 设置标签
    new->targetTime = targetTime;
    new->next = NULL;
    if(p == NULL){
        head = new;
    }else{
        while(p->next != NULL){
            p = p->next;
        }
        p->next = new;
    }
}
/**
 * @brief 根据id删除节点
 * 
 * @param id 
 */
void deleteMyDelay(uint32_t id){
    struct MyDelayTypeDef *p = head;
    struct MyDelayTypeDef *pre = NULL;
    while(p != NULL){
        if(p->id == id){
            if(pre == NULL){
                head = p->next;
            }else{
                pre->next = p->next;
            }
            free(p);
            return;
        }
        pre = p;
        p = p->next;
    }
}

/**
 * @brief 延时函数，不阻塞
 * 
 * @param id 一个唯一标识，建议用函数名的指针强转成uint32_t
 * @param delayTime 延时时间，单位ms，与getGlobalTime()的单位一致
 * @param label 延时任务的标签，可以使用__LINE__来标记，表示是否和之前的标签不一样
 * @return true 延时完成
 * @return false 延时未完成
 * @note 首次调用时自动根据id添加延时任务并返回false，延时完成后返回true并自动删除延时任务
 * @example 
 *          while(1){
 * 			if(myDelay((uint32_t)main,3000,__LINE__)==true){
 *				LCD_DisplayChar(Line1,1,'1');break;//3秒后显示1
 *			    }	
 *          }
 */
bool myDelay_label(uint32_t id,uint32_t delayTime, int16_t label){
    if (xPortIsInsideInterrupt())
        return true; // 在中断中不支持延时，直接返回true
    xSemaphoreTake(mydelay_mutex,portMAX_DELAY); // 获取互斥锁
    if(delayTime == 0){
        return true;
    }
    struct MyDelayTypeDef *p = findMyDelay(id);
    if(p == NULL){
        addMyDelay(id, getGlobalTime() + delayTime, label);
        xSemaphoreGive(mydelay_mutex); // 释放互斥锁
        return false;
    }
    if(p->label != label){
        // 如果标签不一样，说明是新的延时任务，重新设置目标时间
        p->targetTime = getGlobalTime() + delayTime;
        p->label = label;  // 更新标签
        LOG_INFO("Mydelay is refresh: id=%u, label=%d, delayTime=%u", id, label, delayTime);
        xSemaphoreGive(mydelay_mutex); // 释放互斥锁
        return false;
    }
    if(getGlobalTime() >= p->targetTime){
        deleteMyDelay(id);
        xSemaphoreGive(mydelay_mutex); // 释放互斥锁
        return true;
    }
    xSemaphoreGive(mydelay_mutex); // 释放互斥锁
    return false;
}
