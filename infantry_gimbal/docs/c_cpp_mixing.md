# STM32 工程中的 C 与 C++ 混编

## 写在前面

这份文档面向的是 STM32 + CubeMX + GCC（`arm-none-eabi`）+ FreeRTOS 这条常见组合。文中出现的代码、
符号和链接脚本片段来自一个真实的 STM32F407 工程（下文称"示例工程"），它们用来把规则落地，看的时候
不必关心那个工程具体在做什么。

STM32 工程几乎必然是混编的，原因很具体：

- **HAL / LL / CMSIS 只有 C。** `stm32f4xx_hal_*.c`、`core_cm4.h`、CMSIS-DSP，全是 C，没有 C++ 版本。
- **CubeMX 只生成 C。** `main.c`、`freertos.c`、`stm32f4xx_it.c`、`stm32f4xx_hal_msp.c` 都是 C，而且
  每次点"生成代码"都会重写一遍。
- **启动文件和链接脚本是 C 风格。** `startup_stm32f407xx.s`、`STM32F407XX_FLASH.ld`，以及 OEM 的
  `.ioc` 工作流，都按 C 工程组织。
- **而业务层想用 C++。** 姿态解算、控制律、状态机、模块封装，用 C++ 写确实更省心。

所以问题不是"要不要混编"，而是"在哪一层切换语言、切换点怎么写"。这篇文章讲的就是这个切换点。

## 一、先把边界划出来：CubeMX 管的地方，和你管的地方

STM32 工程和一般嵌入式工程最大的区别，是有一大片代码**不属于你**，而且会被工具反复重写。开写之前
先划清：

| 区域 | 谁生成 | 被覆盖的风险 | 语言 |
| --- | --- | --- | --- |
| `Core/Src/main.c`、`freertos.c`、`stm32f4xx_it.c`、`stm32f4xx_hal_msp.c` | CubeMX | 每次生成代码都重写（`USER CODE` 块除外） | C，别改成 C++ |
| `Core/Startup/*.s`、`*.ld` | CubeMX | 同上 | 汇编 / 链接脚本 |
| `Drivers/`、`Middlewares/` | ST / 第三方 | 升级库或重新生成时整体替换 | C，一个字都别改 |
| `Core/Inc/main.h`、`FreeRTOSConfig.h` | CubeMX | 同上 | C，配置靠 .ioc 改 |
| 你自己的模块（示例工程里是 `user/`、`utils/`） | 你 | 无 | **C++ 的主场** |

由此得到两条硬规矩：

**第一条：CubeMX 生成的文件保持 C。** 把 `main.c` 改名成 `main.cpp` 能编过，但下次生成代码时 CubeMX
会再造一个 `main.c` 出来，工程里就有两个 `main`。同理，`freertos.c`、`stm32f4xx_it.c` 都别碰。
自定义代码塞进 `USER CODE BEGIN / END` 块里，或者在别处写模块、在生成文件里只留一行调用。

**第二条：边界上的函数越少越好。** 生成的那些 `.c` 文件里，每个都只留一句 `XxxInit()` / `XxxTask()`
调用，背后是 C++ 的模块。这样 CubeMX 重新生成一万次，你的代码一行都不会丢。

示例工程的做法就是：`main.c` 在进 RTOS 之前调一次 `InfantryGimbalInit()`，`freertos.c` 里的任务入口
循环调 `GimbalTask()`、`CanTask()` 这类函数，`main.c` 的错误处理分支里调两个从 C++ 侧读出来的标志位
查询函数。整个工程跨语言的地方就这三处，其余全在 `user/` 里用 C++ 写。

## 二、混编的麻烦，九成来自名字修饰

C 的函数符号就是函数名本身。C++ 不行——有重载、有命名空间、有类，编译器必须把类型信息编进符号名
里。这就是 name mangling。看同一个固件里的两个函数：

```
$ arm-none-eabi-nm firmware.elf | grep -E "GimbalTask|DjiMotorSend"
0800e6a4 T GimbalTask
08010f78 T _Z12DjiMotorSendP17CAN_HandleTypeDefmssss
```

第一个是干净的 C 符号，因为声明它的时候用了 `extern "C"`。第二个是修饰后的 C++ 符号，拆开读就是：
`12` 个字符的函数名 `DjiMotorSend`，参数依次是 `CAN_HandleTypeDef*`（`P17...`）、`unsigned long`
（`m`）和四个 `short`（`ssss`）。

知道这件事，混编的报错就都能读懂了。最常见的一种，链接时报：

```
undefined reference to `foo(int, float)'
```

注意引号里那串参数类型。链接器在找一个**干净的** `foo`，说明引用方是 C 编译单元，而定义方按修饰名
`_Z3fooif` 存放，两边对不上。绝大多数情况下，原因就是那个头文件漏了 `extern "C"`。

反过来还有一种，报 `undefined reference to 'foo'`，而定义明明写在 `.cpp` 里。这通常是那个 `.cpp`
**没有 include 声明它的头文件**，于是它自己按 C++ 方式定义了一遍，符号变成修饰名，谁也对不上谁。
这条规则请背下来：**每个实现文件的第一件事，就是 include 它自己对应的头文件。** 这不是风格问题，
是正确性问题。

## 三、STM32 的好消息：ST 的头文件已经帮你处理了 `extern "C"`

先说一个能省下大量工作的结论：**ST 官方的这些头文件，全都自带 `extern "C"` 包裹**，所以从 C++ 里
include 它们、直接调用，符号就是对的，不需要你自己包一层：

| 头文件 | 里面有 `extern "C"` 吗 |
| --- | --- |
| `stm32f4xx_hal_uart.h` 等所有 HAL 外设头 | 有 |
| `core_cm4.h`（CMSIS Core） | 有 |
| `FreeRTOS.h`、`cmsis_os.h` | 有 |
| CubeMX 生成的 `main.h` | 有 |

所以 `HAL_UART_Transmit(&huart1, ...)`、`osDelay(1)`、`__disable_irq()` 这些，C++ 里直接写就行。

但**你自己写的、要两边共用的接口头，必须自己加**。ST 帮你管的是它的头，不是你的：

```c
#ifndef MODULE_NAME_H_
#define MODULE_NAME_H_

#include "main.h"        /* 需要时正常 include，它在 extern "C" 里也无所谓 */

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
        uint8_t id;
        float   scale;
    } SomeConfig;

    int SomeFunc(SomeConfig* cfg);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_NAME_H_ */
```

两个 `#ifdef __cplusplus` 都要有：C 编译器不认识 `extern "C"` 这个语法，直接写会编译报错。默认情况
（C 编译器）什么都不加，只有 C++ 编译器才把它包起来。

最后提醒一句：`extern "C"` 只管名字。它**不管**参数类型检查、结构体布局、浮点 ABI，也不管你传进去
的是不是 C++ 类。别把它当成"兼容开关"——决定两个编译单元能否真正互操作的是 ABI（见第九节）。

## 四、头文件只有三种形态，别发明第四种

### 形态 A：纯 C 接口

C 和 C++ 都能包含，内容只用 C 类型。这是跨语言接口的**默认选择**——哪怕你现在觉得"只有 C++ 会用到"，
也先按 A 写，以后 `main.c` 想调一下就省事。

### 形态 B：纯 C++ 头

里面是 class、模板、命名空间，C 编译器处理不了。整个实体部分包在 `#ifdef __cplusplus` 里：

```cpp
#ifndef BSP_IMU_H_
#define BSP_IMU_H_

#ifdef __cplusplus
#include <cstdint>
#include "spi.h"

class Imu {
public:
    void Init(SPI_HandleTypeDef* hspi);
    void Update();
private:
    SPI_HandleTypeDef* hspi_;
    float gyro_[3];
};

#endif /* __cplusplus */
#endif /* BSP_IMU_H_ */
```

**这个形态有个很贵的坑：C 文件 include 一个纯 C++ 头不会报错，只会得到一个空文件。**
`#ifdef __cplusplus` 不成立，里面所有声明都被预处理掉了，编译安静通过，直到链接才出现
`undefined reference`。而错误现场离原因非常远——你会去翻调用点、翻 `CMakeLists.txt`，很难想到问题
出在"这个头在 C 里等于不存在"。

所以纯 C++ 的头都建议加一句保险：

```c
#ifndef __cplusplus
#error "bsp_imu.h is C++ only, do not include it from a .c file"
#endif
```

宁可让它在编译期就炸，也别留到链接期。这是 STM32 混编工程里性价比最高的一行代码。

### 形态 C：一个头文件同时服务两边

当同一个模块既需要一个 C++ 类给业务层用，又需要几个函数给 CubeMX 生成的 `.c` 文件用时，就写成两段：
类定义包在 `#ifdef __cplusplus` 里，C 接口包在 `extern "C"` 里。

```c
/* 第一段：只有 C++ 能看到 */
#ifdef __cplusplus
#include <cstdint>

class Board {
public:
    uint8_t GetStopFlag(void) { return stop_flag_; }
    void SetStopFlag(uint8_t f) { stop_flag_ = f; }
private:
    uint8_t stop_flag_;
};

extern Board board;
#endif

/* 第二段：C 和 C++ 都能看到 */
#ifdef __cplusplus
extern "C" {
#endif

    uint8_t Board_GetStopFlag(void);

#ifdef __cplusplus
}
#endif
```

实现文件里做转发，只有一行：

```cpp
extern "C" uint8_t Board_GetStopFlag(void) {
    return board.GetStopFlag();
}
```

定义处那个 `extern "C"` 其实是多余的——头文件已经声明过，`.cpp` 又包含了它，链接属性会继承下来。
写上只为了让读代码的人一眼看到"这是给 C 用的门"。两种做法都行，团队内统一即可。

### 顺带一句：`-D` 宏两边不共享

`main.c` 和你的 `.cpp` 是两个独立的编译前端，`-D` 宏、`__STDC__`、`__cplusplus` 各编各的。判断
"这里是 C 还是 C++"只能靠 `__cplusplus`。特别注意 `USER CODE BEGIN Includes` 这类位置——在里面
`#define` 的东西，只有那个 C 文件看得到。

## 五、C++ 调 HAL / CMSIS / FreeRTOS：基本无痛，但有四个细节

**1. HAL 的结构体在 C++ 里就是普通的类类型。** 直接 `UART_InitTypeDef cfg;` 然后逐个赋值，
`HAL_UART_Init(&huart, &cfg)` 传指针。不要包成 C++ 类、不要 `new`、尽量别按值传。示例工程里初始化
CAN 实例就是这么干的：`CAN_InitTypeDef` 填一遍，注册进去。

**2. 函数指针类型的括号里写什么，两边含义不一样。** `void (*cb)();` 在 C 里是"参数表未指定"，在
C++ 里是"没有参数"。你在 HAL 或自己写的 BSP 里看到 `void (*pCallback)();` 这种写法时，注意它其实是
个 C 的宽松声明。这种地方建议统一写 `void (*cb)(void)`，把"未指定"这个特殊含义消掉。

**3. 不要把 C++ 的头（尤其是标准库头）include 进 `extern "C"` 块里。** 标准库头里满是模板和重载，
放进 `extern "C"` 会得到一串莫名其妙的报错。这类 include 一律放块外。反过来，把 `main.h`、`spi.h`
这类 C 头放在 `extern "C"` 里 include 是没问题的，也是常见做法——前提是它确实是 C 头。

**4. `enum` 和 `bool` 在两边含义有细微差别。** `enum` 在 C 里基本就是 `int`，在 C++ 里是独立类型；
C99 的 `_Bool` 和 C++ 的 `bool` 也不是同一个东西（虽然通常兼容）。跨语言接口上用 `uint8_t` 而不是
`bool`，用 `HAL_StatusTypeDef` 这种现成的 C 枚举而不是自己造，能省掉很多麻烦。顺便说，HAL 里
`HAL_OK`/`HAL_ERROR` 这类返回码和 C++ 的错误处理习惯正好合拍——直接用，别包成异常。

## 六、C 调 C++：一定要有一层门

`main.c`、`freertos.c`、`stm32f4xx_it.c` 这些都是 C，而且由 CubeMX 管着，你不可能在里面写 C++。所以
C++ 模块想被调用，就自己出口一个 C 风格的函数。

### 门的六条规矩

1. **只用 C 有的类型**：`uint8_t`、`int32_t`、`float`、指针、`struct`、`enum`、HAL 自己的类型。
   不要出现引用、类、模板、`std::` 里的任何东西。
2. **不传 C++ 对象，哪怕按值。** C 侧没有那个类型，编译器会直接拒绝。
3. **不让异常穿过去。** 见第十节，这是硬性要求。
4. **函数名加统一前缀**（`Imu_Init`、`Board_GetStopFlag`），跟 C++ 侧的成员函数区分开。
5. **门函数只做转发和类型转换**，业务逻辑写在类里。否则以后 C++ 侧想复用就得先拆一遍。
6. **每个门函数都有能看懂它的头文件声明**，放在形态 A 或形态 C 的头里。`.c` 文件不应该自己写
   `extern` 声明——那等于绕过了头文件这个唯一的事实来源。

### 不透明句柄：把 C++ 对象交给 C 侧持有

如果 C 侧需要"持有一个 C++ 对象"（比如一个驱动实例），不要试图把类的定义暴露给 C，用不透明句柄：

```c
/* bsp_imu.h —— C 侧只看到一个不完整的类型 */
typedef struct imu_s imu_t;

imu_t* Imu_Create(void);
void   Imu_Destroy(imu_t* imu);
int    Imu_Read(imu_t* imu, float* gyro_out);
```

```cpp
/* bsp_imu.cpp —— 结构体的定义只在这里出现 */
#include "bsp_imu.h"

#include <cstdint>
#include <cstdlib>
#include <new>

#include "spi.h"

struct imu_s {
    SPI_HandleTypeDef* hspi;
    float              gyro[3];
    uint32_t           sample_count;
};

extern "C" imu_t* Imu_Create(void) {
    imu_t* imu = static_cast<imu_t*>(std::malloc(sizeof(imu_s)));
    if (imu == nullptr) {
        return nullptr;
    }
    new (imu) imu_s;          /* placement new，跑构造函数 */
    imu->hspi = &hspi1;       /* 真正的初始化需要在系统初始化之后 */
    return imu;
}

extern "C" void Imu_Destroy(imu_t* imu) {
    if (imu == nullptr) {
        return;
    }
    imu->~imu_s();            /* 显式跑析构函数 */
    std::free(imu);
}
```

好处很实在：C 侧的 `sizeof(imu_t)` 是非法的、成员不可见，只能通过函数操作；C++ 侧随便改结构体的
内部布局，都不用动 C 侧一行代码。代价是多一层指针间接，以及生命周期管理职责——所以
`Create`/`Destroy` 必须成对出现，并且写清楚谁负责释放。

如果不想引入堆（STM32 上应该尽量如此），就让调用方提供存储空间。注意参数不能用 `imu_t*`——那是
不完整类型，C 侧连对象都声明不出来，只能用 `void*`：

```c
/* 调用方提供内存，模块只负责在上面构造对象 */
uint32_t Imu_Sizeof(void);
uint32_t Imu_Alignof(void);
void     Imu_Construct(void* mem);   /* mem 的大小和对齐必须满足上面两个函数 */
```

调用方那边配合一个对齐的静态缓冲区就行：

```c
static uint8_t imu_storage[256] __attribute__((aligned(8)));
/* 断言 Imu_Sizeof() <= sizeof(imu_storage)，然后 Imu_Construct(imu_storage) */
```

这种写法没有运行时失败路径，容易做内存预算，在实时系统里是首选。

### 覆盖 HAL 的弱回调：STM32 上最容易踩的一个坑

HAL 里所有的回调都是弱定义，你在自己文件里写一个同名函数就覆盖了它——`HAL_UART_RxCpltCallback`、
`HAL_UARTEx_RxEventCallback`、`HAL_CAN_RxFifo0MsgPendingCallback`、`HAL_SPI_TxRxCpltCallback`，
都是这个机制。中断里 `HAL_xxx_IRQHandler` 处理完硬件后回调它们。

问题来了：这个覆盖函数如果写在 `.cpp` 里，还能生效吗？**答案取决于你有没有包含 HAL 的头文件。**
实际编译两种写法看符号名：

```cpp
/* 写法一：没包含 HAL 头，自己声明参数类型 */
typedef struct { int dummy; } CAN_HandleTypeDef;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) { (void)hcan; }

/* 写法二：老老实实包含 HAL 头 */
#include "stm32f4xx_hal.h"
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) { (void)hcan; }
```

```
写法一： 00000000 T _Z33HAL_CAN_RxFifo0MsgPendingCallbackP17CAN_HandleTypeDef
写法二： 00000000 T HAL_CAN_RxFifo0MsgPendingCallback
```

写法一的符号名被修饰了，而 HAL 库里的弱符号叫 `HAL_CAN_RxFifo0MsgPendingCallback`。两者名字不同，
所以**链接器认为它们是两个函数，覆盖没有发生**——链接成功，没有警告，中断能进，但你的回调永远不会
被调用。现象是"CAN 收不到数据"、"串口收不到数据"，然后你去查波特率、查滤波器、查 DMA，方向全错。

原因是第三节讲过的：ST 的 HAL 头里有 `extern "C"`，包含了它，你的定义就继承了 C 链接，符号正确；
不包含（忘了 include，或者只写了个前置声明），它就按 C++ 规则修饰了名字。

验证只需要一行：

```
$ arm-none-eabi-nm build/your_firmware.elf | grep HAL_CAN_RxFifo0MsgPendingCallback
080105cc T HAL_CAN_RxFifo0MsgPendingCallback     ← T，好，覆盖成功
08005a34 W HAL_CAN_RxFifo0MsgPendingCallback     ← W，坏，这是 HAL 的空实现，你的没生效
```

`T` 是强符号、`W` 是弱符号。看到 `W` 或者带 `_Z` 前缀的，就是没覆盖上。

示例工程的做法更省事：这些回调全部写在 `.c` 文件里（`bsp_can.c`、`bsp_uart.c`），从根上绕开这个
问题。**建议照这个来——HAL 会反向调用的那些钩子，尽量留在 C 这一侧。** 如果一定要写在 `.cpp` 里，
就确保包含 HAL 头，并且编完用 `nm` 确认一次。

### 全局对象做 C 接口时，`const` 会咬人

C++ 里有一条反直觉的规则：**命名空间作用域上的 `const` 对象默认是内部链接**（等价于 `static`）。
所以下面这两种写法，链接行为完全不同：

```cpp
const float kImuScale = 0.001065f;        /* 内部链接，别的编译单元根本看不见它 */
```

```cpp
/* 头文件里 */
extern const float kImuScale;             /* 外部链接，别的编译单元能引用 */
```

示例工程里有一个 `extern const GimbalMode gimbalmode;`，C 侧要引用它，靠的就是头文件里那句 `extern`
把链接属性带过去；谁要是顺手删掉那句 `extern`，链接立刻失败，而代码看上去毫无变化。

更稳妥的态度是：**跨语言共享的全局变量越少越好**。常量用 `#define` 或 `enum`，非要共享可变状态
就共享访问函数，别共享对象本身。

## 七、启动过程：从复位向量到 `main`，再到你的 C++ 对象

STM32 的启动路径是固定的，理解它才知道 C++ 的全局对象到底在哪一步活过来。`Reset_Handler` 大致做
四件事（示例工程的 `startup_stm32f407ighx.s`）：

```
Reset_Handler:
    /* 1. 把 .data 段从 FLASH 拷到 RAM */
    /* 2. 把 .bss 段清零 */
    bl  SystemInit           /* 3. 配时钟、配中断向量表偏移 */
    bl  __libc_init_array    /* 4. 跑全局对象的构造函数 ← C++ 从这里开始 */
    bl  main
```

`__libc_init_array` 本身是 newlib 里的函数，它遍历链接脚本给出的 `.init_array` 段，把里面登记的
函数挨个调一遍。而这些登记的条目，就是编译器为每个"有非平凡构造函数的全局对象"生成的初始化函数
（符号名形如 `_GLOBAL__sub_I_xxx`）。

### 链接脚本里那个 `KEEP` 不是装饰

`.init_array` 段必须显式写进链接脚本，而且要用 `KEEP` 保护起来。示例工程的
`STM32F407IGHX_FLASH.ld` 里是这样：

```
  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH
```

为什么要 `KEEP`：工程开了 `-Wl,--gc-sections`，链接器会丢掉所有"看起来没人引用"的段。而
`.init_array` 里的条目是被启动代码用符号区间遍历的，没有任何代码直接引用它们——链接器完全可能
把整个段当垃圾丢掉。丢了之后的现象是：全局对象全是零值、构造函数不跑，**而且没有任何报错**。

所以：如果你手工改过链接脚本，一定要确认 `.init_array` 段在、且有 `KEEP`；用 CubeMX 生成的脚本
默认是对的，别随手删。

### 构造函数跑的时候，STM32 还没准备好

这一点比上面那条更容易踩。`__libc_init_array` 在 `SystemInit` 之后、`main` 之前执行，那时：

- 时钟刚配好，但外设时钟（`__HAL_RCC_GPIOA_CLK_ENABLE()` 之类）还没使能；
- `HAL_Init()` 还没调，SysTick 还没起，`HAL_GetTick()` 返回 0；
- 看门狗没喂，出错了没有日志可看。

所以初始化要分三层，示例工程的分工也正好是这样：

| 阶段 | 位置 | 只应该做什么 |
| --- | --- | --- |
| 全局对象构造 | `main` 之前（`__libc_init_array`） | 把成员置成确定的初值（`nullptr` 指针、默认参数） |
| 系统初始化 | `main` 里，`MX_xxx_Init()` 之后 | 硬件相关的一次性初始化，此时外设时钟、HAL、DMA 都就绪 |
| 任务初始化 | FreeRTOS 启动后，任务上下文里 | 需要阻塞、需要调度器、需要其它模块就绪的初始化 |

对应到示例工程：全局对象只赋初值，`InfantryGimbalInit()` 在 `main` 的外设初始化之后调，
`GimbalInit()`、`INS_Init()` 在各自任务里调。这条分工是人为约定的，编译器不会帮你检查——谁在
构造函数里调了 `HAL_CAN_Start()`，就得自己去解释为什么冷启动偶尔失败、热复位又正常。

### 跨编译单元的构造顺序是不确定的

C++ 不保证不同编译单元里全局对象的构造顺序，也不保证 `A` 的构造函数里能安全地用全局对象 `B`。所以
**别让一个全局对象的构造函数去碰另一个全局对象**。真需要依赖关系，就退化成显式的 `Init()`，或者
用"函数内静态变量"做惰性构造（但要注意首次调用在中断/多线程环境下的安全性）。

顺便说一个实践细节：全局对象的构造函数是在 `main` 之前跑的，这段时间没有任何东西保护你。所以
构造函数越简单越好，别在里面做任何可能失败的事，更别在里面等标志位。

## 八、内存：STM32 上你会同时遇到两个堆

这一节是混编在 STM32 上特有的麻烦，而且很容易被忽略：**C++ 的 `new` 和 FreeRTOS 的
`pvPortMalloc` 用的是两个完全独立的堆。**

| 分配入口 | 底层实现 | 容量由什么决定 |
| --- | --- | --- |
| `malloc`、`new`、`std::` 容器、`std::string` | newlib 的 `_sbrk`（`Core/Src/sysmem.c`） | 链接脚本里的 `_Min_Heap_Size`（示例工程 `0x4000` = 16 KB） |
| `pvPortMalloc`、`osThreadCreate`、`osMessageCreate`、`osTimerCreate` | FreeRTOS `heap_4.c` | `FreeRTOSConfig.h` 里的 `configTOTAL_HEAP_SIZE`（示例工程 15360 字节） |

FreeRTOS 的堆是一块静态数组（`heap_4.c` 里的 `ucHeap`），编译期就落进 `.bss`——示例工程里它是
`0x3c00` 字节，等于 `configTOTAL_HEAP_SIZE` 那 15360 字节，这块 RAM 是实打实被扣掉的。newlib 的堆
是另一回事：它靠 `_sbrk` 在运行时从 `_end` 往上"长"，上限是 `_estack - _Min_Stack_Size`，也就是
说 **newlib 堆和主栈（MSP）共用同一片区间，从两头往中间长，撞上就是 HardFault**。示例工程的
`sysmem.c` 把这个范围画得很清楚：

```
###########################################################################
#  .data  #  .bss  #       newlib heap       #          MSP stack        #
#         #        #                         # Reserved by _Min_Stack_Size #
###########################################################################
^-- RAM start      ^-- _end                             _estack, RAM end --^
```

后果是：`xPortGetFreeHeapSize()` 返回的数字只反映 FreeRTOS 那一半，和你还能 `new` 多少字节毫无
关系。混编工程里"内存莫名耗尽"最难查的形态，就是两套分配混着用，然后看着其中一个的剩余量发呆。

所以建议是：

- **别用 `new` / `delete`**，也就顺便和 STL 容器说再见了。理由在第十节，这里补充一条：它走的是
  另一个堆。
- 需要动态创建 RTOS 对象（任务、队列、信号量），就用 `osXxxCreate` 系列，它们走 FreeRTOS 堆。
- 真要动态内存，统一走一套；两套混用之前，先想清楚谁在哪个区间里。
- 改 `_Min_Heap_Size` 之前先确认真的需要。CubeMX 新生成的链接脚本给的默认值很小（`_Min_Heap_Size
  = 0x200`、`_Min_Stack_Size = 0x400`），示例工程在 flash 版本的脚本里调成了 `0x4000`
  ——而同一个工程里那份"从 RAM 运行"的脚本还是默认值，两份脚本不一致，看的时候别串了。
- 这两个数字不是备注，是**真的会在 `.bss` 之后预留出来的地址空间**。链接脚本里那段
  `._user_heap_stack` 把自己的注释写得很直白："used to check that there is enough RAM memory
  left"——RAM 放不下就直接报 `region RAM overflowed`。示例工程的 `.map` 里能看到它占了
  `0x8004` 字节（`0x4000` 堆 + `0x4000` 栈 + 4 字节对齐），而这 32 KB 是从 128 KB 的 SRAM 里
  切出去的，所以调大它确实会让 `.bss` 可用空间变小。

### CCMRAM 的坑

STM32F407 有 64 KB 的 CCM RAM（`0x10000000`），链接脚本里定义了它：

```
MEMORY
{
  CCMRAM    (xrw)    : ORIGIN = 0x10000000,   LENGTH = 64K
  RAM       (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH     (rx)     : ORIGIN = 0x8000000,    LENGTH = 1024K
}
```

但默认情况下**没有任何段被分配进 CCMRAM**，它就是空着的。想把 C++ 的大对象、算法工作区放进去，
得手工在 `.ld` 里加段。放之前记住两件事：CCM 挂在 CPU 的内核总线上，**DMA 访问不到**（把
`HAL_UART_Transmit_DMA` 的缓冲区放 CCM 是经典错误，现象是数据永远是垃圾）；另外启动时的 `.data`
拷贝和 `.bss` 清零只处理常规 RAM，放进 CCM 的段得自己处理初始化。

### 全局对象占多少 RAM

有非平凡构造函数的全局对象，对象本身和 C 的全局变量一样待在 `.data`（有非零初值）或 `.bss`
（零初始化）里，**不额外占 RAM**。多出来的只是构造函数本身——它躺在 `.init_array` 指向的代码里，
在 `main` 之前被执行。

代价在启动时间上：`.init_array` 里有 N 个全局对象，启动时就有 N 次函数调用。一般几十个对象也就
几微秒，无所谓；但如果有上电就要立刻响应的需求（比如总线上电同步、看门狗喂狗窗口），值得数一下
自己到底建了多少全局对象。

## 九、工具链与构建：`arm-none-eabi` 的那些开关

### 两套编译器，但只有一个链接器，而且必须是 C++ 那个

`.c` 交给 `arm-none-eabi-gcc`，`.cpp` 交给 `arm-none-eabi-g++`，这没什么好说的。真正容易出问题的
是链接：**必须用 `arm-none-eabi-g++` 来链接**。C++ 的链接驱动会额外带上 C++ 运行时库和静态初始化
支持，用 `gcc` 链接会漏掉它们，表现是一堆莫名其妙的 `undefined reference`，或者全局对象的构造
函数死活不跑。

示例工程的 CMake 里，链接规则名就是 `CXX_EXECUTABLE_LINKER`，用的正是 `arm-none-eabi-g++`。自己
手写 Makefile 的话，把 `$(CC)` 换成 `$(CXX)` 就对了。

### 这几个选项 C 和 C++ 必须完全一致

```
-mcpu=cortex-m4 -mthumb -mthumb-interwork
-mfloat-abi=hard -mfpu=fpv4-sp-d16
```

这些是 ABI。浮点参数走 VFP 寄存器还是走栈、结构体怎么对齐，都跟它们有关。不一致的话链接器一般
能拦住（典型报错是 "uses VFP register arguments, output does not"），这算好事——问题暴露得早。

但这里有个 STM32 特有的连带问题：**CMSIS-DSP 的预编译库是按浮点 ABI 分变体的。** 示例工程链接的是

```
Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a
```

文件名里的 `l` 是 little-endian，`f` 表示硬浮点（fpv4-sp-d16）。ARM 原版 CMSIS-DSP 的发行包里
按内核和浮点类型提供了一整套变体（`libarm_cortexM4l_math.a`、`libarm_cortexM7lfdp_math.a` 等等），
而 CubeMX 生成工程时通常只带进来匹配当前配置的那一个——示例工程的 `Lib/` 目录下就只有
`libarm_cortexM4lf_math.a` 一个文件。所以换芯片或换浮点选项时要注意：

- 改成软浮点，就得换软浮点的那个版本；
- 从 F4 换到 F7/H7，FPU 指令集从 fpv4 变成 fpv5，`-mfpu` 和库都要跟着改。
- 本地只有匹配的那一个变体，换配置时得从 CMSIS-DSP 发行包里把对应的库取过来。

忘了换的表现通常是链接期报库属性不匹配，但别指望它一定能拦住——所以**改浮点 ABI 的时候，把
CMSIS-DSP 库的变体当作检查清单里的一项**。

### 让链接器帮你丢掉没用的代码

`-ffunction-sections -fdata-sections` 配合链接选项 `--gc-sections`，对 C++ 工程基本是必备的。C++
里一个类往往只用到几个成员函数，其余的 inline 函数、模板实例、全局对象的析构逻辑都会产生代码。
没有这组选项，没人调用的东西会实打实占 flash。

示例工程同时还开了 `--print-memory-usage` 和 `-Map=`，编完能直接在终端看到 flash/RAM 的占用，
`.map` 文件里则能查到"是哪个函数把 flash 吃掉了"。新增一个类之后如果 `.text` 突然涨了几 KB，去
`.map` 里搜一下谁被拖进来了——常见的元凶是一个"看起来很无害"的日志或格式化函数。

### 标准版本可能没有真的钉死

构建系统里写"用 C++17"和实际传给编译器的参数，未必是一回事。以 CMake 为例：

```cmake
set(CMAKE_CXX_STANDARD 17)
```

这行**不保证**命令行里出现 `-std=gnu++17`。CMake 会先探测编译器的默认标准，如果默认值已经满足要求
（GCC 13 的默认就是 `gnu++17`），它就认为不需要再加参数。后果是：某天换了个默认标准更低的工具链，
代码突然编译不过，而构建脚本看起来明明是对的。

想让标准真的固定下来，就显式加：

```cmake
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
```

或者在团队里统一工具链版本。两条路选一条，别都不做。验证方法很简单：打开
`compile_commands.json`（CMake 的 `CMAKE_EXPORT_COMPILE_COMMANDS`）看 C++ 文件的真实命令行，别猜。

### newlib 还是 newlib-nano

示例工程**没有**开 `--specs=nano.specs`，用的是完整版 newlib。完整版的 `printf` 带浮点格式化，体积
可观；flash 紧张时可以换 nano，代价是格式化能力变弱（默认连 `%f` 都不支持，要额外加
`-u _printf_float`）。

比 `printf` 更需要注意的是 C++ 这一侧：`iostream`、`std::string`、`std::function` 会连锁拉进
locale、异常支持、动态分配一堆东西，在 STM32 上基本等于把 flash 送出去。这也是为什么业务层用
C++，但日志输出还是走 `snprintf` + 串口/USB CDC 更实在。

### 换工具链要重新确认的事项

上面这些结论是 GNU 工具链的。换成 Keil MDK 或 IAR 时，语言规则（`extern "C"`、名字修饰、C++ 对象
模型）完全一样，但入口机制、静态初始化、运行时库都不一样：GCC 靠启动文件里显式调
`__libc_init_array`，另外两家各有自己的机制。**换工具链时，重新确认"全局对象的构造函数到底会不会
被调用"这一件事**，别照搬 GCC 的经验。

## 十、中断、异常与 RTOS

### 异常：在 STM32 上是硬性禁止

两个理由，第二个更致命：

1. 异常需要 unwinder、类型信息表、展开栈帧的代码，Cortex-M 上没有 MMU、栈又小，这一套的体积和栈
   开销都不便宜。
2. **异常不能穿越 C 代码。** STM32 工程的调用栈上几乎必然有 C 帧——HAL 的中断处理函数、
   FreeRTOS 的 `port.c`（纯 C 加汇编）、CMSIS 的启动代码。异常穿过这些帧时找不到对应的 handler，
   结果是 `std::terminate`、`abort`，或者在中断上下文里直接 HardFault 甚至复位。

FreeRTOS 那个例子值得单独说：任务切换点写在汇编里，它不在 C++ 异常处理模型的覆盖范围内。异常一旦
越过任务切换的边界，程序状态就无法恢复，而且这种故障是概率性的——取决于当时压栈了多少内容。

替代方案很朴素：返回 `HAL_StatusTypeDef` 或者自定义错误码，配合输出参数。丑一点，但行为可预测。

### `-fno-exceptions -fno-rtti` 买的是什么

值得开，但它买的不是体积。如果代码里本来没人用异常，再开上 `-ffunction-sections` 和
`--gc-sections`，那些支持代码会被链接器丢掉，体积上基本看不出差别。它真正的价值是**约束**：以后
有人手滑写了 `throw` 或者 `dynamic_cast`，编译期立刻报错，而不是等到某条异常路径真的被执行才发现。
在多人协作的工程里，这个保险很值。

### 中断服务函数里能做什么

可以调 C++ 函数，限制和 C 完全一样：

- 只能碰 `volatile` 的、无锁的数据；和任务共享的状态要用临界区保护。
- 不能阻塞，不能调用可能睡眠的 API（FreeRTOS 的 `xxxFromISR` 系列除外）。
- 别做长耗时的事。析构函数本身没问题，但如果析构里关外设、等标志位，就等于在中断里做了长事——
  这种"藏在析构里的等待"在混编工程里特别难查。

还有一条容易忽略的：**函数内的 `static` 局部变量，如果类型非平凡（有构造/析构），别在中断里用。**
C++11 起它们的初始化带线程安全保护，会引入 `__cxa_guard_acquire` 之类的运行时调用。真要缓存状态，
用文件作用域的 POD 变量。

### FreeRTOS 那边要注意的

**任务入口必须是 C 函数指针。** `osThreadCreate` 要的是 `void (*)(void*)`。CubeMX 生成的
`freertos.c`（C）里的 `StartXxxTask` 就是这个角色，里面循环调你的 C++ 门函数：

```c
void StartControlTask(void const* argument) {
    Control_Init();          /* 门函数，内部创建 C++ 对象 */
    for (;;) {
        Control_Update();    /* 门函数 */
        osDelay(1);
    }
}
```

**栈的单位别猜。** FreeRTOS 的 `xTaskCreate` 收的是 `StackType_t` 的**个数**，在 Cortex-M4 上就是
字（1 字 = 4 字节）；而 CMSIS-RTOS 的封装层是否做单位换算，各家不一样。示例工程用的那层封装把参数
原样透传，所以任务定义里写的 `512` 实际是 2 KB 而不是 512 字节；`FreeRTOSConfig.h` 里的
`configMINIMAL_STACK_SIZE` 也是同一个单位（`128` 就是 512 字节）。**去读你那一层封装的源码**，
别按经验记。

**C++ 的局部对象吃栈。** 一个函数里如果声明了几个带成员的类对象，又套了几层临时对象，栈需求会比
看上去大不少。调栈的时候别靠猜，用 `uxTaskGetStackHighWaterMark()` 看实际水位（需要
`INCLUDE_uxTaskGetStackHighWaterMark`），跑一段时间再看——只看启动时的数字会低估。

**栈不够时的优先动作是减局部对象**（把临时对象改成成员、减少嵌套调用层数），而不是无脑加栈。加栈
只是把问题推到 RAM 里，一个任务一个任务地试下去，最后一定会撞到内存上限。

### 这些 C++ 特性在 STM32 上的取舍

| 特性 | 建议 | 代价 / 风险 |
| --- | --- | --- |
| 类、命名空间、构造析构 | 用 | 理论上零开销，是 C++ 最大的价值 |
| 函数重载、`constexpr`、`enum class` | 用 | 纯编译期，零成本 |
| 模板 | 谨慎用 | 每个实例化都生成一份代码，容易吃 flash |
| 虚函数 | 谨慎用 | 每个对象多一个 vptr，vtable 占 flash；没有继承体系时是纯开销 |
| 异常 | 别用 | 见上文，中断上下文里尤其危险 |
| RTTI（`dynamic_cast`、`typeid`） | 别用 | 引入类型元数据，体积大 |
| `new`/`delete`、STL 容器 | 别用 | 走另一个堆，且有碎片问题（第八节） |
| `std::string`、`iostream` | 别用 | 体积大、依赖重定向、隐含动态分配 |
| `std::array`、`<algorithm>`、`<type_traits>` | 可以用 | 定长、无堆分配，编译期展开 |

## 十一、STM32 混编特有的报错，按现象查

这些报错和真实原因之间往往隔得很远，列一张对照表：

| 现象 | 可能原因 |
| --- | --- |
| 中断能进，但自己的回调不执行 | HAL 弱回调没覆盖上：`nm` 看是 `W` 还是带 `_Z` 前缀（第六节） |
| `nm` 里该是 `T` 的符号变成了 `_Z...` | 头文件漏了 `extern "C"`，或者定义处没包含 HAL 头 |
| `undefined reference to 'foo(int, float)'`，参数类型出现在报错里 | 同上，引用方是 C，定义方是修饰名 |
| `undefined reference to 'foo'`，但 `.cpp` 里明明定义了 | 实现文件没 include 自己的头文件，被定义成了修饰名 |
| `undefined reference to '_sbrk'` / `'_write'` / `'_exit'` | `syscalls.c`、`sysmem.c` 没被加进构建，或被人从工程里删了 |
| `region FLASH overflowed by N bytes` | 模板实例太多、`iostream`、完整版 `printf` 被拉进来了 |
| `multiple definition of 'main'` | `main.c` 被改名成 `main.cpp`，CubeMX 又重新生成了一个 |
| 上电跑不起来，复位一下又正常 | 全局对象构造函数里碰了外设，或看门狗没喂 |
| 串口/USB 的 `printf` 没输出 | `_write` 没重定向，或者链接用了错误的重定向实现 |
| 全局对象是零值，成员仿佛没初始化 | `.init_array` 段被 `--gc-sections` 丢掉，或启动文件没调 `__libc_init_array` |
| HardFault 落进 `Default_Handler` | 野指针、栈溢出、DMA 缓冲区放进了 CCMRAM、未对齐访问 |
| 同一个结构体两边 `sizeof` 不一致 | 有一侧漏了打包指令，或者某处又抄了一份定义 |
| 链接器报 "uses VFP register arguments, ... does not" | 浮点 ABI 选项两边不一致 |
| 加了新类，`.text` 涨了好几 KB | 有东西被意外引用（虚表、RTTI、模板实例、日志函数），去 `.map` 里查 |
| 改了头文件的函数签名，只有一边报错 | 增量构建没跟上头文件依赖，全量重编再判断 |

其中"回调不触发"和"全局对象没初始化"这两条，是 STM32 混编工程里最常见的两个隐形故障——都不报错，
都只能在符号表或 `.map` 里看出来。

## 十二、动手前过一遍

**写跨语言的接口时：**

- [ ] 头文件有包含保护，C 侧可见的声明包在 `#ifdef __cplusplus / extern "C" / #endif` 里
- [ ] C 侧能看到的类型只有定宽整数、`float`、指针、`struct`、`enum`、HAL 类型
- [ ] 纯 C++ 的头加了 `#error` 拒绝 C 编译器，而不是让它静默变空
- [ ] 每个实现文件都 include 了自己的头文件
- [ ] 桥接函数只做转发和类型转换，不写业务逻辑，不抛异常
- [ ] 结构体/枚举定义只有一处，打包方式两边一致
- [ ] 跨编译单元共享的对象用 `extern` 声明在头里、定义在 `.cpp` 里（特别是 `const`）

**写 C++ 模块时：**

- [ ] 构造函数里不碰外设、不依赖其它全局对象、不等标志位
- [ ] 初始化分成"构造 → `MX_xxx_Init` 之后 → 任务里"三层，每一层只做该做的事
- [ ] 没有 `new`/`delete`/STL 容器/`iostream`；非要用堆，说清楚走的是哪个堆
- [ ] 继承体系是真需要的，不是为了"看起来面向对象"
- [ ] 模板的实例化数量在预期内

**HAL 交互时：**

- [ ] 覆盖 HAL 弱回调的函数，要么写在 `.c` 里，要么确保包含了 HAL 头
- [ ] 编译后用 `nm` 确认回调符号是 `T` 而不是 `W` 或 `_Z`
- [ ] 中断相关的代码里没有阻塞调用、没有非平凡类型的函数内 `static`

**构建和验证：**

- [ ] 链接用的是 `arm-none-eabi-g++`，不是 `gcc`
- [ ] C 和 C++ 的 `-mcpu` / `-mthumb` / `-mfloat-abi` / `-mfpu` 完全一致
- [ ] CMSIS-DSP 预编译库的变体和浮点选项匹配（F4 是 `M4lf`，换芯片要一起换）
- [ ] 开了 `-ffunction-sections -fdata-sections` 和 `--gc-sections`
- [ ] 链接脚本里 `.init_array` 段存在且有 `KEEP`
- [ ] C++ 标准版本确认过（看 `compile_commands.json`，不看构建脚本的意图）
- [ ] 至少 `-Wall` 开着
- [ ] 编完看一眼 `--print-memory-usage` 的输出，flash/RAM 都在预算内

## 附：两个可以照抄的模板

**给 C 和 C++ 共用的接口头文件**

```c
#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include "spi.h"          /* 需要 HAL 类型时正常 include */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    SPI_HandleTypeDef* hspi;
    float              scale;
} ImuConfig;

void Imu_Init(const ImuConfig* cfg);
int  Imu_Read(float* gyro_out);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
```

**对应的 C++ 实现（含那道门）**

```cpp
#include "imu.h"             /* 第一行，不能省 */

#include "internal_imu.h"

namespace {

bool         g_inited = false;
ImuConfig    g_cfg{};
InternalImu  g_imu;          /* 构造只置初值，不碰 SPI */

}  /* namespace */

extern "C" void Imu_Init(const ImuConfig* cfg) {
    if (cfg == nullptr) {
        return;
    }
    g_cfg = *cfg;            /* 参数校验、类型转换 */
    g_imu.Init(g_cfg);       /* 真正的初始化在这里，此时 SPI 已经就绪 */
    g_inited = true;
}

extern "C" int Imu_Read(float* gyro_out) {
    if (!g_inited || (gyro_out == nullptr)) {
        return -1;
    }
    *gyro_out = g_imu.GetGyroZ();
    return 0;                /* 错误码，而不是异常 */
}
```

这个模板里有几个细节值得注意：`#include "imu.h"` 放在第一行；全局对象只在构造时置初值、不碰 SPI；
真正的初始化在 `Imu_Init` 里，由 `main` 在外设初始化之后调用；出错返回错误码而不是抛异常；参数有
校验。这五点基本就是 STM32 混编工程里 C++ 侧的日常。

---

如果只记一句话：**C 和 C++ 的边界是一道门，不是一层纱。** 门上有 `extern "C"`，门两边只用 C 能
理解的类型说话，进门之前确认对方能看懂。在 STM32 上，这三点做对了，剩下的意外基本就只有两类——
HAL 弱回调没覆盖上，以及 `.init_array` 里的构造函数没跑。这两类都能用 `arm-none-eabi-nm` 一眼
看出来，所以养成编完看一眼符号表的习惯，比记规则管用。
