# Log日志

此代码可以将串口的日志信息清晰的输出出来，包括文件名，函数名，时间。

特色功能：支持使用DMA进行无阻塞发送，内部集成发送队列和缓冲区，可以在中断中使用

## 快速使用

1. 通过`log_set_all_fmt()`和`log_set_fmt()`函数配置日志输出内容，`log_set_fmt()`中的参数可以使用`|`或起来，如：LOG_FMT_LEVEL|LOG_FMT_TIME；一开始默认所有额外信息都不输出，只会输出你要输出的内容。

2. 日志默认开启，可以通过`log_disable()`和`log_enable()`来进行关闭和开启。

3. 需要在`log_port.c`和`log_port.h`文件中配置输出接口，默认使用串口。

`log_port.c`的示例如下：
```c
USART_SendType channel_0;//定义串口输出句柄
USART_SendType channel_1;//定义串口输出句柄
// 日志接口初始化
void log_port_init(void)
{
    // 最好直接在这里设置日志输出通道，这里要输入的是你的接口句柄
    log_set_channel(0, &channel_0);//绑定输出句柄和通道数
    log_set_channel(1, &channel_1);
    // 对句柄进行初始化
    USART_SendInit(&channel_0, (&huart1));
    USART_SendInit(&channel_1, (&huart2));
}

/**
 * @brief 日志输出接口
 * 
 * @param this 日志通道,这个值是log_set_channel()设置的指针
 * @param data 数据
 * @param size 数据大小
 * @return uint8_t 0:成功 其他失败
 */
uint8_t log_port_output(LogOut_Handle *this, uint8_t *data, uint16_t size)
{
    return USART_SendData(this, data, size, USART_USE_MOLLOC);
}

uint8_t log_port_output_IT(LogOut_Handle *this, uint8_t *data, uint16_t size)
{
    return USART_SendData(this, data, size, USART_USE_RING_BUFF);
}

// 获取时间戳接口
uint32_t log_port_get_time(void) {
    extern uint32_t get_time();
    return get_time();
}
```

`log_port.h`的示例如下：

```C
typedef USART_SendType LogOut_Handle;//定义LogOut_Handle的类型，为了满足.c中的要求
```

4. 如需要使用`usart_send`功能则需要要在`usart_send_port.c`中进行配置发送功能，如：

```c
/**
 * @brief 发送接口(Unlock版本,在调用此函数时已经进入临界区)
 * 
 * @param this USART_SendType 结构体指针
 * @param data 数据
 * @param size 数据大小
 */
void USART_port_output(USART_SendType *this,uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit_DMA(this->handle, data, size); // 直接发送队列的数据
}

// 串口发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    //在串口使用DMA把数据发送完成后调用以下函数
    USART_SendCallback(huart);
}
```

`usart_send_port.h`的示例如下

```C
typedef UART_HandleTypeDef UART_sendDevice_Handle;//定义UART_sendDevice_Handle的类型，方便.c中使用
```

5. 关于`log.h`和`usart_send.h`中的配置项可以在[相关配置](#相关配置)中进行查看
6. 基础使用方法可以在[使用实例](#使用实例)中查看，更多api接口和使用方法可以在[高级使用](#高级使用)中自行查看
7. 关于各个函数的作用请在[函数引用](#函数引用)中自行查看
6. 更多配置请在.c和.h中自行探索

### 使用实例

```c
// test.c
void send_init()
{
    log_set_all_fmt();
    log_init();
}

void fun()
{
    // 这个例子输出就是 [INFO][123]test.c|fun():55:this is the default channel
    // 依次为输出等级、时间、文件名、函数名、行号、内容
    LOG_INFO("this is the default channel");//默认的默认通道是通道0,可以通过宏来改变，这句话会在通道0中输出
    LOG_DEBUG_CHANNEL(1,"this is channel 1");//从通道1输出
    // ...more
}
void TIME_IT()
{
    //这是定时器中断
    LOG_INFO_IT("this is interrupt");//可以在中断中使用带IT结尾的API，默认从通道0中输出
}
```

### 常见问题

为什么我在线程模式中有输出而在中断中使用后没有输出？

检查你是否使用是`_IT`结尾的API，然后，中断中只有一个日志通道可以使用，如果你使用默认通道的话，还必须**保证**你的默认是第一个初始化的，具体见[中断级的发送](#中断级的发送)

## 相关配置

### log配置

| 设置项 | 类型 | 默认值 | 配置说明 |
| ----- | ----- | ------ | ------- |
| LOG_CHANNEL_NUM | uint8_t | 4(个通道) | 可以设置的最大通道数量 |
| LOG_BUFFER_SIZE（在log_port.h中） | uint16_t | 256(个字节) | 一次性最大输出长度 |
| ENABLE_LOG | 1/0 | 1(开启) | 是否开启日志 |
| LOG_LEVEL | (LOG_LEVEL) | LOG_LEVEL_DEBUG(打印所有日志) | 开启的日志级别（此级别以上的会输出） |

输出格式设置（此为全局的静态选项，还有运行时可调的动态选项，动态选项可查看[函数引用](#函数引用)和[输出格式](#关于输出格式)）

| 设置项          | 类型    | 默认值  | 配置说明                                                     |
| --------------- | ------- | ------- | ------------------------------------------------------------ |
| USE_BASE_FILE   | 1/0     | 1(开启) | 路径是否只输出基本的文件名（如 aa.c,不使用则为C://dir//aa.c） |
| USING_FUNCTION  | 1/0     | 1(开启) | 是否使用函数名                                               |
| USING_FILE      | 1/0     | 1(开启) | 是否使用文件名                                               |
| DEFAULT_CHANNEL | uint8_t | 0       | 使用通道0为默认，在不指定输出通道时，使用此通道              |

输出格式和`ENABLE_LOG`和`LOG_LEVEL`和的设置可以每个.c配置得不一样，可以在编译时设置此宏，也可以在`#include "log.h"`前先定义其中某些宏

比如，我在特定的某个.c，要专门使用通道1，那么，我在引用`log.h`之前，先定义下默认通道为1，这样，这个文件的默认的输出通道就是1，其他的还是0

### usart_send配置

| 设置项 | 类型 | 默认值 | 配置说明 |
| ----- | ----- | ------ | ------- |
| USART_QUEUE_MAX_SIZE | uint8_t | 30(个) | 待发送队列最大长度
| USART_BUFF_SIZE | uint16_t | 1024(个字节) | 发送缓冲区最大长度(普通发送+中断发送)
| USART_RING_BUFF_SIZE | uint16_t | 1024(个字节) | 环形缓冲区大小

## 高级使用

### API说明

```C
#define LOG_DEBUG_CHANNEL(channel, format, ...)
#define LOG_DEBUG_CHANNEL_IT(channel, format, ...)
#define LOG_DEBUG(format, ...)
#define LOG_DEBUG_IT(format, ...)
//使用示例
LOG_DEBUG_CHANNEL(1,"hello world%d",2);//在通道1输出hello world2
LOG_DEBUG("hello %f",1.2)//在默认通道输出hello 1.2
_IT 结尾的宏可以在中断中使用
```

### 大致原理和行为模式

以上可调用的API都是一些宏，具体的调用是使用`log_output_async()`函数，这个函数的功能是异步打印日志，会处理日志的格式，输出格式要求，如是否有时间，是否有行号等（查看[输出格式](#关于输出格式)），最后把这些格式统一转化成字符串，再调用底层的串口发送API，进行异步的发送

#### 转化为字符串

统一转化成字符串的过程，需要使用sprintf函数，为此，引入了一个printf的**第三方库**，这样，就可以不管编译器而可以正常的使用，如果你不想使用这个库，可以在log.c的开头，把这个引用换成标准C库

因为使用sprintf，所以需要一个字符串缓冲区，这个缓冲区有两种选择，一个是使用全局变量，一个是使用栈，即局部变量。

如果使用全局变量的话，那么就需要做全局保护，如果是只用在线程级，则可以用互斥量，如果是需要用在中断中，则只能关中断

*或者，要追求更高的性能，可以使用多个缓冲区，既然这里用了多个缓冲区，甚至可以用动态分配的缓冲区，那么，是否可以把这些缓冲区直接使用底层的接口，使用手动管理的模式来发送，而不使用底层的MALLOC模式，这样，可以少一次拷贝的时间（TODO）*

目前，在`log_port.h`中，有一个宏`LOG_BUFFER_USE_STACK`，由这个宏决定使用哪种模式，1: 使用栈空间, 0: 使用全局变量，在内存RAM足够的情况下，直接使用栈是最好的，如果使用全局变量，需要自行实现`log_sprintf_buffer_lock()`，如果要使用中断，那么这个函数就只能是关中断，这样会影响实时性

如果不使用中断，那么可以使用操作系统中的互斥量

#### 发送的大致思想

使用DMA来发送，就可以非阻塞的发送消息。把要发送的消息，先放在缓冲区内，等前面的数据发送完了，再从缓冲区内取数据再次发送

缓冲区分为三种，一种是用户自行管理的，此版本对于此功能使用较少。第二种是malloc出来的，第三种是环形缓冲区

malloc在线程级中使用，环形缓冲区在中断中使用。

为了管理这些数据，还需要一个发送队列来管理数据的大小，数据的指针位置，那么，队列节点的空间也分为两种，一种是malloc出来的，一种是静态的，malloc出来的节点，用来放malloc出来的数据，用在线程级，静态的用来放环形缓冲区的和用户自行管理的。

#### 线程级的发送

在调用`LOG_DEBUG`等不带`IT`结尾的宏时，会使用`MALLOC模式`调用`USART_SendData()`，然后在底层会malloc出一块空间来，把上面字符串缓冲区中的内容，拷贝到那块空间中去，然后再调用DMA（port中实现的）去发送，malloc的次数和申请出来的空间大小，以对象为单位，受两个宏限制

以对象为单位的意思是，如在[快速使用](#快速使用)的示例中`USART_SendType channel_0;//定义串口输出句柄`这就是串口发送对象，一般，不同的日志通道会对应着不同的串口发送对象，或称为句柄，在这个类型的定义中，有两个变量

```C
    uint8_t queue_size_malloc; // 当前malloc队列长度
    uint16_t buff_size_malloc; // 当前使用的malloc缓冲区总大小
```

这两个变量受`usart_send.h`中的两个宏限制

```C
#define USART_QUEUE_MAX_SIZE 30 // 单个对象发送队列最大长度
#define USART_BUFF_MALLOC_SIZE 1024    // 单个对象malloc出来的发送缓冲区最大长度
```

当然，free后会归还回去，请注意,中断不能调用free，free的具体行为请看[发送完成](#发送完成)

既然是线程级，那么就会说到多线程，如果使用是操作系统，在不同的线程中使用，上述两个变量就必须受到互斥量的保护

所以在`usart_send_port.h`中，定义了`UART_SEND_MUTITHTEAD`，1: 启用, 0: 不启用，如果启用的话，那么`USART_SendType`类型中，就会定义出一个互斥量来。如果开启了这个宏，还需要在`usart_send_port.c`中定义`void USART_Mutex_Lock(UART_MUTEX *mutex);`和`void USART_Mutex_Unlock(UART_MUTEX *mutex);`两个函数

发送的流程见[开发说明](#开发说明)

#### 中断级的发送

在中断中调用，需调用带有`IT`结尾的宏，中断是会嵌套的，嵌套也可以调用，不会出问题，这正是这个库的特点。

需要注意的是，如果sprintf的缓冲区是栈模式，那嵌套调用将可能产生巨大的栈空间，小心爆掉。如果使用全局变量模式，那么sprintf的过程是会关中断的，在那个过程就不会嵌套了，不过，在sprintf完到后面拷贝完数据后，到调用开始发送的函数前，是可以发生嵌套的

中断发送的数据，会存放在lwrb库管理的环形缓冲区中，所以引入了lwrb库。

因为使用了环形缓冲区，所以可能会出现一帧数据要分成两次发送的情况，在环形缓冲区的头尾位置。

使用lwrb库时，是需要关中断的，所以拷贝的过程是关中断的，会影响实时性，而线程级别的拷贝，是不关中断的，因为malloc出来的空间是唯一的，环形缓冲区只有一个

为了避免有很多环形缓冲区，加上在中断中发送的需求不会很多，所以，本库只有一个全局的环形缓冲区，本来应该每个串口发送结构体都有一个的，但是感觉没必要（好吧，是因为一开始做错了，让全部共用一个缓冲区了，后面懒得加了）

不同的串口发送结构体不能共用同一个环形缓冲区，会造成数据冲突，所以在串口发送结构体初始化时，就规定了，只有第一个初始化的结构体才能使用环形缓冲区，所以，你在中断中，只能使用第一个初始化的结构体来发送数据

发送的流程见[开发说明](#开发说明)

#### 发送完成

发送完成会释放资源，检查队列中是否还有数据需要发送，如果有，就开启下一次发送，如果没有，就结束发送

如果你使用DMA，发送完成会有DMA完成中断，需要在中断中调用发送完成回调函数，如见`usart_send_port.c`:

```C
// 串口发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    //在串口使用DMA把数据发送完成后调用以下函数
    USART_SendCallback(huart);
}
```

如果是环形缓冲区中的资源，则直接归还环形缓冲区，节点也归还静态节点数组，如果是malloc出来的，因为在中断中不能使用free，所以只能把他们放到一个链表中去，然后在线程模式调用发送的时候，再进行释放资源

### 一些特性

#### 关于输出格式

上文有提到，输出格式分为静态和动态，静态是编译期就确定的，在.h中定义发了，动态则为运行时决定。

可以为不同的日志等级设置不同的输出格式，如info等级就只输出字符串，ERROR等级就输出文件名行号时间等格式

不支持不同的通道设置不同的输出格式，输出格式只能和等级相关

静态的设置比动态设置优先级**高**，静态关闭后动态无法再次打开

输出格式的使用可以参考以下示例和[函数引用](#函数引用);


​    // 恢复 INFO 的完整格式，便于后续测试对比

​    log_set_fmt(LOG_LEVEL_INFO, LOG_FMT_FILE | LOG_FMT_FUNC | LOG_FMT_LINE);)的具体说明

```C
typedef enum _log_fmt_type {
    LOG_FMT_LEVEL = 1 << 0, // 日志级别
    LOG_FMT_TIME = 1 << 1,  // 时间
    LOG_FMT_FILE = 1 << 2,  // 文件名
    LOG_FMT_FUNC = 1 << 3,  // 函数名
    LOG_FMT_LINE = 1 << 4,  // 行号
} LogFmtType;

#define LOG_FMT_ALL (LOG_FMT_LEVEL | LOG_FMT_TIME | LOG_FMT_FILE | LOG_FMT_FUNC | LOG_FMT_LINE) // 所有格式

// 格式可以使用“按位或”来把多个格式合在一起
    log_set_all_fmt();//设置所有的日志等级打开所有的输出格式
    LOG_INFO("[fmt] all fields: level, time, file, func, line");

    // 将 INFO 清除文件/函数/行号，仅保留等级与时间
    log_clear_fmt(LOG_LEVEL_INFO, LOG_FMT_FILE | LOG_FMT_FUNC | LOG_FMT_LINE);
    LOG_INFO("[fmt] level + time only");

    // 将 DEBUG 清除所有附加格式，观察纯消息
    log_set_fmt(LOG_LEVEL_DEBUG,LOG_FMT_ALL);//DEBUG添加所有的格式
    log_clear_fmt(LOG_LEVEL_DEBUG, LOG_FMT_LEVEL | LOG_FMT_TIME | LOG_FMT_FILE | LOG_FMT_FUNC | LOG_FMT_LINE);//把这些格式去除
    LOG_DEBUG("[fmt] message only (no extra info)");

    // 恢复 INFO 的完整格式，便于后续测试对比
    log_set_fmt(LOG_LEVEL_INFO, LOG_FMT_FILE | LOG_FMT_FUNC | LOG_FMT_LINE);//给INFO添加这些格式
```

#### 关于实时性

可以全局搜索`实时性`来看一些场景的说明

本库比较注重实时性，而不那么注重效率性，意为，如果有一大段的临界区，在保证安全性的前提下，会更倾向于拆分多个小的临界区，让其实时响应性能更好。

如果你的临界区不想关全部中断，也可以只关使用到这个组件以下等级的中断

#### 关于操作系统

虽然使用操作系统，您有大概率不需要用到此日志组件，因为如果单开一个线程来负责做日志的发送，可以做得特别优雅。但是，如果要使用，也是可以的，需要添加一个地方，`usart_send_port.h`中的`UART_SEND_MUTITHTEAD`宏，和它相关的两个锁函数

## 函数引用

### log_init

```c
void log_init(void);
```

初始化log，请在输出串口前进行调用。

### log_set_all_fmt

```c
void log_set_all_fmt(void);
```

设置日志输出所有格式，可以在调用`log_init()`前进行调用。设置所有的日志等级打开所有的输出格式

### log_set_fmt

```c
void log_set_fmt(uint8_t level, uint16_t type);
```

设置日志输出格式。

参数：
- `level`:需要设置的等级
- `type`:要设置的类型,在枚举变量`LogFmtType`中进行选择

如：
```c
// 显示INFO中的日志级别和时间
log_set_fmt(LOG_LEVEL_INFO,LOG_FMT_LEVEL|LOG_FMT_TIME);
```

### log_clear_fmt

```c
void log_clear_fmt(uint8_t level, uint16_t type);
```

清除日志输出格式。

参数：
- `level`:需要设置的等级
- `type`:要设置的类型,在枚举变量`LogFmtType`中进行选择

如：
```c
// 清除(不显示)INFO中的日志级别和时间
log_clear_fmt(LOG_LEVEL_INFO,LOG_FMT_LEVEL|LOG_FMT_TIME);
```

### log_set_channel

```c
void log_set_channel(uint8_t channel, LogOut_Handle* handle);
```

设置日志输出通道

参数：
- `channel`:要选择的输出通道
- `handle`:要绑定的输出句柄，是给后面在log_port.c中调用输出函数时用的

例子可以在[使用实例](#使用实例)中进行查看

### log_disable 和 log_enable

```c
void log_disable(void);
void log_enable(void);
```

开启和关闭日志，即是否开启日志输出。

## 开发说明

请先仔细阅读[高级使用](#高级使用)，了解清楚整体的行为逻辑

几个文件的关系如下：

<img src=".\.doc\核心文件关系结构图.drawio.svg"  />

 log.c中没什么好说的，自行看源文件即可看懂

usart_send.c中，有几个事情需要点明

1. USART_SendType类型的数据的定义通常由外部完成，即是其他文件定义的全局变量，然后通过`USART_SendInit()`函数，将这个全局变量注册到一个全局的链表中，目前是只能加不能删。然后这个文件内部的所有操作都是通过这个全局的链表来操作和索引的，只有第一个注册进来的结构体才能使用_IT,即拥有环形缓冲区的使用权。Init函数没有做好多线程保护，请自行注意，一般不会发生问题。如果需要做的话，应该是注意保护那个全局的链表

2. 还有一个free的链表，因为不能在中断中调用free函数，所以需要把要释放的节点，放到这个链表中，然后在线程调用时，先去释放free后再malloc新的节点。因为需要记录该节点所在的USART_SendType类型的对象，所以通过`enum Usart_SendMode mode`来取巧存储，这是个整形，存储在全局链表中的第几位。

3. 这里有节点和数据两种东西需要记录存储和管理。节点中有数据的地址，数据的大小，发送的模式几个内容，同时，节点是通过链表管理的，在`USART_SendType`类型中，有一个头节点指针和尾节点指针，就是用来管理整个链表的。

   节点分为动态分配的节点和静态分配的节点，动态分配为malloc，用来管理malloc的数据，静态分配是用一个数组，然后再用一个bool数组来记录数组的使用情况。静态分配是用来管理环形缓冲区和自定义管理的数据的

4. 外部委托进来的数据，除了自定义管理的数据，都要拷贝一次到这个文件自己管理的地方去，比如malloc，或者使用环形缓冲区，然后通过节点来进行管理，每个数据会挂到一个节点上，节点又会挂到`USART_SendType`对象中去

5. 每次接收到委托时，都会先申请新的节点，然后拷贝数据（自行管理的情况除外），然后再把节点插入到`USART_SendType`对象的管理的链表中中去，然后再查看`USART_SendType`对象是否在发送中，如果是空闲的，就开启一次发送，如果在发送中，就不管。发送完成会有回调函数，会检查是否还有数据需要发送，如果还有，就继续发，如果没有，就标记为空闲。正常情况下，一个节点只会调用一次发送，但是，在环形缓冲区头尾相接的地方，一个节点要分成两次发

   下面是并行时的一些分析，分析并发安全时的一些思考

   <img src="./.doc/分析并发安全.jpg" style="zoom:33%;" />

   

## 风险说明
日志初始化和串口发送结构体初始化没有加入线程安全的保护，一般不会出问题 
不要在多核的机器上使用，没有考虑相应的并发情况

## 未来规划

增加同步发送和插队发送机制，同步发送超时机制

*或者，要追求更高的性能，可以使用多个缓冲区，既然这里用了多个缓冲区，甚至可以用动态分配的缓冲区，那么，是否可以把这些缓冲区直接使用底层的接口，使用手动管理的模式来发送，而不使用底层的MALLOC模式，这样，可以少一次拷贝的时间（TODO）*（sprintf中的缓冲区问题），这样，底层的手动模式可能需要增加回调函数

暂不考虑做操作系统的支持，因为可以用easylog，easylog正常是同步模式，但是其使用os后可以使用异步模式，easylog使用异步模式后应该也是可以在中断中使用的

[GitHub - armink/EasyLogger: An ultra-lightweight(ROM<1.6K, RAM<0.3k), high-performance C/C++ log library. | 一款超轻量级(ROM<1.6K, RAM<0.3k)、高性能的 C/C++ 日志库](https://github.com/armink/EasyLogger)

如果要做操作系统的支持， 那么就应该要做彻底，要加入一个专门的发送线程来管，把在中断中打印的数据，先把参数列表存到一个全局的变量里，然后在这个线程中去读取，解析，格式化，发送。当然，目前这个库是多线程安全的，是可以使用os的，但是效果没有做多一个专门的发送线程来得好
