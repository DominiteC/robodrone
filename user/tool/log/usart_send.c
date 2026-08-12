#include "usart_send.h"
#include <string.h>
#include <stdarg.h>
#include "lwrb.h"
#include "printf.h"

#define malloc(x) pvPortMalloc(x)
#define free(x) vPortFree(x)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define CHECK_ERROR() while(1) // 检测点

USART_SendType *usartSendHead_handle = NULL;  // 串口数据结构体链表头
static void USART_SendAttach(USART_SendType *this);

uint8_t queue_size_ringbuff; // 全局环形缓冲区队列长度
uint16_t buff_size_ringbuff; // 全局使用的环形缓冲区总大小

lwrb_t ringbuff;
uint8_t ringbuff_data[USART_RING_BUFF_SIZE];
USART_SendType* canUsedRb_handle = NULL; // 当前可以使用环形缓冲区的串口发送结构体

DataQueue dataQueue_pool[USART_QUEUE_RINGBUFFER_SIZE];
uint8_t dataQueue_pool_used[(USART_QUEUE_RINGBUFFER_SIZE+7)/8] = {0}; // 位图标记池中数据使用情况

DataQueue* toRelease = NULL; // 待释放数据队列头

#define PRINT_BUFFER_SIZE 256
static char print_buffer[PRINT_BUFFER_SIZE];

extern void USART_Enter_Critical(void);
extern void USART_Exit_Critical(void);
extern void USART_port_output(USART_SendType *this,uint8_t *data, uint16_t size);

static DataQueue* findDataQueueInPool_Unlock(void)
{
    for (uint8_t i = 0; i < USART_QUEUE_RINGBUFFER_SIZE; i++)
    {
        if ((dataQueue_pool_used[i / 8] & (1 << (i % 8))) == 0)
        {
            dataQueue_pool_used[i / 8] |= (1 << (i % 8)); // 标记为已使用
            return &dataQueue_pool[i];
        }
    }
    return NULL; // 未找到可用节点
}
static void freeDataQueueInPool_Unlock(DataQueue* node)
{
    uint8_t index = node - dataQueue_pool;
    if (index < USART_QUEUE_RINGBUFFER_SIZE)
    {
        dataQueue_pool_used[index / 8] &= ~(1 << (index % 8)); // 标记为未使用
    }
}
/**
 * @brief 串口发送初始化
 * 
 * @param this 串口发送结构体
 * @param huart 串口句柄
 * @param txSize_Max 缓冲区大小
 */
void USART_SendInit(USART_SendType *this, UART_sendDevice_Handle *huart)
{
    this->handle = huart;
    this->usart_tx_sta = USART_SEND_IDLE;
    this->head = NULL;
    this->tail = NULL;
    this->queue_size_malloc = 0;
    this->buff_size_malloc = 0;

    USART_SendAttach(this);
    // 初始化环形缓冲区, 只初始化一次
    if (lwrb_is_ready(&ringbuff) == 0)
    {
        lwrb_init(&ringbuff, ringbuff_data, USART_RING_BUFF_SIZE);
        canUsedRb_handle = this;
    }
#if UART_SEND_MUTITHTEAD == 1
    USART_Mutex_Init(&this->mutex);
#endif
}

static void USART_SendAttach(USART_SendType *this)
{
    USART_SendType *target = usartSendHead_handle;
    if(target == NULL)
    {
        usartSendHead_handle = this;
        this->next = NULL;
        return;
    }
    while (1)
    {
        if (target == this)
            return;
        if(target->next == NULL)
        {
            target->next = this;
            this->next = NULL;
            return;
        }
        target = target->next;
    }
}

/**
 * @brief 构造节点并拷贝数据
 * 
 * @param data 
 * @param size 
 * @return DataQueue* 
 */
DataQueue * createQueueNode_malloc(uint8_t* data, uint16_t size)
{
    DataQueue *newData = (DataQueue *)malloc(sizeof(DataQueue));    // 申请队列内存
    if (newData == NULL)
    {
        return NULL;
    }
    newData->data = (uint8_t *)malloc(size*sizeof(uint8_t));        // 申请数据内存
    if (newData->data == NULL)
    {
        free(newData);
        return NULL;
    }
    memcpy(newData->data, data, size);  // 拷贝数据, 保证数据有效
    newData->size = size;
    newData->next = NULL;
    return newData;
}
DataQueue * createQueueNode_Static(uint8_t* data, uint16_t size)
{
    USART_Enter_Critical(); // 关键区域
    DataQueue *newData = findDataQueueInPool_Unlock();    // 申请队列内存
    USART_Exit_Critical();  // 退出关键区域
    if(newData == NULL)
    {
        return NULL;
    }
    newData->data = data;        // 申请数据内存
    newData->size = size;
    newData->next = NULL;
    return newData;
}
/**
 * @brief 检查数据大小和队列长度，并更新统计数据
 * 
 * @param this 
 * @param size 
 * @param mode 
 * @return Usart_SendState 
 */
static Usart_SendState USART_SendData_CheckSize(USART_SendType *this, uint16_t size, Usart_SendMode mode){
    if(mode == USART_USE_RING_BUFF){
        //可能在中断中使用,必须使用严格的并发安全策略
        USART_Enter_Critical(); // 关键区域
        if(queue_size_ringbuff >= USART_QUEUE_RINGBUFFER_SIZE)
        {
            USART_Exit_Critical();  // 退出关键区域
            return USART_SEND_QUEUE_FULL;
        }
        if (buff_size_ringbuff + size > USART_RING_BUFF_SIZE || lwrb_get_free(&ringbuff) < size)
        // 检测环形缓冲区是否有足够的空间,两者都要判断,因为拷贝时是允许中断的
        {
            USART_Exit_Critical();  // 退出关键区域
            return USART_SEND_BUFF_FULL;
        }
        queue_size_ringbuff++;
        buff_size_ringbuff += size;
        USART_Exit_Critical();  // 退出关键区域
    }
    else if (mode == USART_USE_MOLLOC)
    {
        #if UART_SEND_MUTITHTEAD == 1
        USART_Mutex_Lock(&this->mutex);
        #endif
        if(this->queue_size_malloc >= USART_QUEUE_MAX_SIZE)
        {
            #if UART_SEND_MUTITHTEAD == 1
            USART_Mutex_Unlock(&this->mutex);
            #endif
            return USART_SEND_QUEUE_FULL;
        }
        if(this->buff_size_malloc + size >= USART_BUFF_MALLOC_SIZE)
        {
            #if UART_SEND_MUTITHTEAD == 1
            USART_Mutex_Unlock(&this->mutex);
            #endif
            return USART_SEND_BUFF_FULL;
        }
        this->queue_size_malloc++;
        this->buff_size_malloc += size;
        #if UART_SEND_MUTITHTEAD == 1
        USART_Mutex_Unlock(&this->mutex);
        #endif
    }
    else if(mode == USART_NO_MOLLOC){
        USART_Enter_Critical(); // 关键区域
        if(queue_size_ringbuff >= USART_QUEUE_RINGBUFFER_SIZE)
        {
            USART_Exit_Critical();  // 退出关键区域
            return USART_SEND_QUEUE_FULL;
        }
        queue_size_ringbuff++;
        USART_Exit_Critical();  // 退出关键区域
    }
    return USART_SEND_SUCCESS;
}
static void addNodeToSendQueue_Unlock(USART_SendType *this, DataQueue* newData){
    // 将数据加入队列中
    if (this->head == NULL)
    {
        this->head = newData;
        this->tail = newData;
    }
    else
    {
        this->tail->next = newData;
        this->tail = newData;
    }
}
static DataQueue* removeNodeFromSendQueue_Unlock(USART_SendType *this){
    DataQueue *oldData = NULL;
    if (this->head != NULL)
    {
        oldData = this->head;
        this->head = this->head->next;
    }
    // 如果队列为空, 尾指针也为空
    if (this->head == NULL)
    {
        this->tail = NULL;
    }
    return oldData;
}
/**
 * @brief 串口发送数据到环形缓冲区,基本需要全程中断保护,避免中断嵌套
 * 
 * @param this 
 * @param data 
 * @param size 
 * @return Usart_SendState 
 */
Usart_SendState USART_SendData_Ringbuffer(USART_SendType *this, uint8_t *data, uint16_t size){
    Usart_SendState state;
    state = USART_SendData_CheckSize(this, size, USART_USE_RING_BUFF);
    if (state != USART_SEND_SUCCESS) return state;

    //构造节点
    DataQueue* newData;
    newData = createQueueNode_Static(NULL, size);  // 一般不会失败，因为前面已经检查过了
    if (newData == NULL) {
        CHECK_ERROR();  // 检测点
    }

    // 写入环形缓冲区，拷贝数据
    USART_Enter_Critical();  // 关键区域
    newData->data = lwrb_get_linear_block_write_address(&ringbuff);
    if (lwrb_write(&ringbuff, data, size) != size) {
        CHECK_ERROR();  // 检测点
    }
    newData->mode = USART_USE_RING_BUFF;
    addNodeToSendQueue_Unlock(this, newData);
    USART_Exit_Critical();  // 退出关键区域
    USART_Enter_Critical(); // 关键区域
    //启动发送
    if(this->usart_tx_sta == USART_SEND_IDLE && this->head != NULL)//必须要判断head不为空，因为可能在中断中已经发送完并清空队列
    {
        this->usart_tx_sta = USART_SEND_BUSY;
        Usart_SendMode mode_headNode = this->head->mode;//headNode可能在线程级已经改变，但是中断中先把sta置为BUSY并运行到这里
        if(mode_headNode == USART_USE_RING_BUFF){
            uint16_t len = lwrb_get_linear_block_read_length(&ringbuff);
            uint16_t send_size = MIN(len, this->head->size);
            USART_port_output(this, this->head->data, send_size); // 发送环形缓冲区的数据
        } else {
            USART_port_output(this, this->head->data, this->head->size); // 直接发送队列的数据
        }
    }
    USART_Exit_Critical();  // 退出关键区域
    return USART_SEND_SUCCESS;
}
static void freeOldNode(){
    while (1) {
        USART_Enter_Critical();  // 进入临界区
        if (toRelease == NULL) {
            USART_Exit_Critical();  // 退出临界区
            break;
        }
        DataQueue* node = toRelease;
        toRelease = toRelease->next;
        USART_Exit_Critical();  // 退出临界区
        // 回滚统计数据
        USART_SendType* temp = usartSendHead_handle;
        uint8_t usart_index = (uint8_t)node->mode;
        while (temp) {
            if (usart_index == 0) {
                break;
            }
            usart_index--;
            temp = temp->next;
        }
        uint16_t freed_size = node->size;
        free(node->data);
        free(node);
    #if UART_SEND_MUTITHTEAD == 1
        if (temp) {
            USART_Mutex_Lock(&temp->mutex);
        }
    #endif
        if (temp) {
            temp->queue_size_malloc--;
            temp->buff_size_malloc -= freed_size;
        }
    #if UART_SEND_MUTITHTEAD == 1
        if (temp) {
            USART_Mutex_Unlock(&temp->mutex);
        }
    #endif
    }
}
/**
 * @brief 串口发送数据
 * 
 * @param this 串口发送结构体
 * @param data 要发送的数据
 * @param size 数据大小
 */
Usart_SendState USART_SendData(USART_SendType *this, uint8_t *data, uint16_t size, Usart_SendMode mode)
{
    if(mode == USART_USE_RING_BUFF){//把中断的数据发送放到专门的函数中
        if(this != canUsedRb_handle) {
            return USART_SEND_FAIL; // 当前不能使用环形缓冲区
        }
        return USART_SendData_Ringbuffer(this, data, size);
    }
    //检查是否需要释放数据
    freeOldNode();
    //判断缓冲区大小和队列长度
    Usart_SendState state;
    state = USART_SendData_CheckSize(this, size, mode);
    if(state != USART_SEND_SUCCESS) return state;

    //申请节点并拷贝数据到队列中
    DataQueue* newData;
    if(mode == USART_USE_MOLLOC){
        newData = createQueueNode_malloc(data, size);
        if(newData == NULL){// 申请失败，回滚统计数据
            #if UART_SEND_MUTITHTEAD == 1
            USART_Mutex_Lock(&this->mutex);
            #endif
            this->queue_size_malloc--;
            this->buff_size_malloc -= size;
            #if UART_SEND_MUTITHTEAD == 1
            USART_Mutex_Unlock(&this->mutex);
            #endif
            return USART_SEND_MALLOC_FAIL;
        }
        newData->mode = mode;
    }else if(mode == USART_NO_MOLLOC){
        newData = createQueueNode_Static(data, size);
        if (newData == NULL) {
            // 申请失败，回滚静态池队列计数
            USART_Enter_Critical();
            if (queue_size_ringbuff > 0) {
                queue_size_ringbuff--;
            }
            USART_Exit_Critical();
            return USART_SEND_QUEUE_FULL;
        }
        newData->mode = mode;
    }

    //将节点加入队列,有可能在中断中使用,必须使用严格的并发安全策略
    USART_Enter_Critical(); // 关键区域
    addNodeToSendQueue_Unlock(this, newData);
    USART_Exit_Critical();  // 退出关键区域

    USART_Enter_Critical(); // 关键区域
    //启动发送
    if(this->usart_tx_sta == USART_SEND_IDLE && this->head != NULL)//必须要判断head不为空，因为可能在中断中已经发送完并清空队列
    {
        this->usart_tx_sta = USART_SEND_BUSY;
        Usart_SendMode mode_headNode = this->head->mode;//headNode可能在线程级已经改变，但是中断中先把sta置为BUSY并运行到这里
        if(mode_headNode == USART_USE_RING_BUFF){
            uint16_t len = lwrb_get_linear_block_read_length(&ringbuff);
            uint16_t send_size = MIN(len, this->head->size);
            USART_port_output(this, this->head->data, send_size); // 发送环形缓冲区的数据
        } else {
            USART_port_output(this, this->head->data, this->head->size); // 直接发送队列的数据
        }
    }
    USART_Exit_Critical();  // 退出关键区域
    return USART_SEND_SUCCESS;
}

/**
 * @brief 串口发送数据,线程不安全
 * 
 * @param this 串口发送结构体
 * @param format 格式化字符串
 * @return Usart_SendState 发送状态
 */
Usart_SendState USART_Printf(USART_SendType *this, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    uint16_t size = vsnprintf(print_buffer, PRINT_BUFFER_SIZE, format, args);
    va_end(args);
    if (size >= PRINT_BUFFER_SIZE) {
        size = PRINT_BUFFER_SIZE - 1;
    }
    return USART_SendData(this, (uint8_t *)print_buffer, size, USART_USE_MOLLOC);
}
/**
 * @brief 从队列中删除节点，释放节点内存，释放数据内存
 * 
 * @param this 
 */
void popQueueNode_ForSendComplete(USART_SendType *this){
    USART_Enter_Critical(); // 进入临界区
    if(this->head == NULL){
        CHECK_ERROR();  //检测点
    }
    Usart_SendMode mode_headNode = this->head->mode;
    if (mode_headNode == USART_USE_RING_BUFF) {
        uint16_t read_length = lwrb_get_linear_block_read_length(&ringbuff);
        if (read_length < this->head->size) {  // 环形缓冲区是否还有数据未发送完
            lwrb_skip(&ringbuff, read_length);  // 跳过已经发送完的数据
            this->head->size -= read_length;    // 更新数据大小
            this->head->data = lwrb_get_linear_block_read_address(&ringbuff); // 更新数据指针
            buff_size_ringbuff -= read_length;
        } else {
            uint16_t finished_size = this->head->size;
            lwrb_skip(&ringbuff, finished_size);  // 跳过已经发送完的数据
            DataQueue* node = removeNodeFromSendQueue_Unlock(this); // 删除发送完成的节点
            freeDataQueueInPool_Unlock(node);// 释放节点
            buff_size_ringbuff -= finished_size;
            queue_size_ringbuff--;
        }
    }
    else if(mode_headNode == USART_USE_MOLLOC){
        DataQueue* node = removeNodeFromSendQueue_Unlock(this); // 删除发送完成的节点
        //添加到待释放数据内存链表
        if(toRelease == NULL){
            toRelease = node;
            node->next = NULL;
        } else{
            node->next = toRelease;
            toRelease = node;
        }
        //想办法记录当前节点是哪个串口的，以便释放时更新统计数据
        //使用Usart_SendMode字段存储第几个串口
        node->mode = 0;;
        USART_SendType *temp = usartSendHead_handle;
        while (temp)
        {
            if (temp == this)
            {
                break;
            }
            node->mode++;
            temp = temp->next;
        }
    }
    else if(mode_headNode == USART_NO_MOLLOC){
        DataQueue* node = removeNodeFromSendQueue_Unlock(this); // 删除发送完成的节点
        freeDataQueueInPool_Unlock(node);// 释放节点
        queue_size_ringbuff--; // 静态池队列长度回滚
    }
    USART_Exit_Critical();  // 退出临界区
}
/**
 * @brief 串口发送完成回调函数
 * 
 * @param this 串口发送结构体
 */
void USART_SendCallback(UART_sendDevice_Handle *handle)
{
    // 存在并发问题
    USART_SendType *this = usartSendHead_handle;
    while (this)
    {
        if (handle == this->handle)   // 找到对应的串口
        {
            //拿出节点
            popQueueNode_ForSendComplete(this);

            //打开下一次传输
            USART_Enter_Critical(); // 进入临界区
            if(this->usart_tx_sta == USART_SEND_IDLE){ //由其他程序开启了发送
                CHECK_ERROR();  //检测点
            }
            // 判断是否还有数据需要发送
            if(this->head == NULL)
            {
                this->usart_tx_sta = USART_SEND_IDLE;   // 转换为空闲状态
                USART_Exit_Critical();  // 退出临界区
                return ;
            }
            Usart_SendMode mode_headNode = this->head->mode;
            if(mode_headNode == USART_USE_RING_BUFF){
                uint16_t len = lwrb_get_linear_block_read_length(&ringbuff);
                uint16_t send_size = MIN(len, this->head->size);
                USART_port_output(this, this->head->data, send_size); // 继续发送
            } else {
                USART_port_output(this, this->head->data, this->head->size); // 继续发送
            }
            USART_Exit_Critical();  // 退出临界区
            return ;
        }
        this = this->next;
    }
}

