
# 参考文件
    https://github.com/mincrmatt12/stm32-platformio-freertos
    https://platformio.org/lib/show/6696/STM32Cube%20Middleware-FreeRTOS/installation

# 文件架构
	采用cmsis_os封装
	不用cmsis_os2的原因是cmsis_os具有硬件优化，而2没有. 见#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
	CMSIS_RTOS:
		cmsis_os.c
		cmsis_os.h
	portable:
		heap_4.c
		port.c
		freeRTOSConfig.h
		portmacro.h
	kernel:
		all other files

# 在jiexianhe原始的freeRTOS的基础上如下修改：
 ## freeRTOSConfig.h
	1 添加 #define configSUPPORT_STATIC_ALLOCATION  0
	2 修改 #define configTOTAL_HEAP_SIZE ((size_t) 64 * 1024) //采用CCMRAM的64k内存全用于rtos
	3 修改 #define configIDLE_SHOULD_YIELD 1 // 空闲任务会被相同优先级的任务抢占
	4 添加 #define INCLUDE_uxTaskGetStackHighWaterMark 1 // 查看任务的栈使用最大情况（检查是否栈溢出）
	5 修改 #define INCLUDE_vTaskCleanUpResources 0 // 不是=1更好吗，不知道为什么，估计和ccmram的使用有关
	6 注释 #define xPortSysTickHandler SysTick_Handler
	7 添加 #define configAPPLICATION_ALLOCATED_HEAP 1 // ucHeap allocated in freertos.c
## 添加中间层[封装]文件：
	1 cmsis_os.c, cmsis_os.h
	2 在cmsis_os.h 里添加 
		1 #define osSystickHandler SysTick_Handler
		2 typedef struct os_thread_def  {
  			const char *name;        ///< Thread name  --> 原来是char *name;
	    3 typedef void (*os_pthread) (void *argument); // 原来是 typedef void (*os_pthread) (void const *argument);
		4 typedef void (*os_ptimer) (void *argument);  // 原来是 typedef void (*os_ptimer) (void const *argument);
	3 在cmsis_os.c 里添加 
		1	__attribute__((section(".ccmram")))
			uint8_t ucHeap[configTOTAL_HEAP_SIZE];
## 添加FreeRTOS-openocd.c文件，用于调试freeRTOS的支持
## LD文件添加ccmram的定义和使用
    1
		MEMORY
		{
		CCMRAM    (xrw)    : ORIGIN = 0x10000000,   LENGTH = 64K
		RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
		FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 768K
		NVM (r)         : ORIGIN = 0x80C0000, LENGTH = 256K
		}
	2
		_siccmram = LOADADDR(.ccmram);

		/* CCM-RAM section 
		* 
		* IMPORTANT NOTE! 
		* If initialized variables will be placed in this section,
		* the startup code needs to be modified to copy the init-values.
		* Oskar: Added NOLOAD to remove this section from .bin outputs
		*/
		.ccmram (NOLOAD):
		{
			. = ALIGN(4);
			_sccmram = .;       /* create a global symbol at ccmram start */
			*(.ccmram)
			*(.ccmram*)
			
			. = ALIGN(4);
			_eccmram = .;       /* create a global symbol at ccmram end */
		} >CCMRAM /*AT> FLASH*/
		
		
	

    

# 任务切换时间(task switching time)
	官方给出是84clock，实测有84-430clock

# CMSIS_RTOS应用
 ## 任务开启
  ### 方法1
	1 定义任务ID：osThreadId led_thread;
	2 创建任务信息：osThreadDef(ledControl, vTaskLedBreathing, osPriority::osPriorityIdle, 0, 256); // 0[第4个]参数默认0即可，没有特殊含义
	3 创建任务：led_thread = osThreadCreate(osThread(ledControl), NULL);
  ### 方法2
	1 创建任务信息：osThreadDef(ledControl, vTaskLedBreathing, osPriority::osPriorityIdle, 0, 256);
	2 创建任务：osThreadCreate(osThread(ledControl), NULL);
 ## 任务开启检查
  ### 每个任务分配的stack size是存储任务函数里包含的局部变量等信息，一定要确认其足够打，
	1 使用 int remain_stack_size = (int)uxTaskGetStackHighWaterMark(NULL);来获取stack size是否足够大
	2涉及到的问题，如果是任务中调用其他函数，那么该函数的stack是否会叠加到任务的stack上呢？
		a 调用全局函数(没有static和内联)，则不会累加到任务的stack上
		b static声明
			(a) 如果只在该任务中被调用，且有且只调用一次，则会累加 （被优化）
			(b) 如果在其他地方还调用，则不会
		c 总结：如果不被优化的话，是不会累加的
  ### 开启之前还可以检查总的heap size还有多少可以分配
		(int)xPortGetFreeHeapSize()
		(int)xPortGetMinimumEverFreeHeapSize()
 ## 任务同步
  ### 基于信号的同步
	osSemaphoreId sem_usb_irq;
    osSemaphoreDef(sem_usb_rx);
  	sem_usb_rx = osSemaphoreCreate(osSemaphore(sem_usb_rx), 1);
  	osSemaphoreWait(sem_usb_rx, time); 
	osSemaphoreRelease(sem_usb_rx)

# RTOS的不兼容性
 ## http://www.nadler.com/embedded/newlibAndFreeRTOS.html
	为了可靠性，较小的嵌入式系统通常不使用动态内存，从而避免相关的泄漏、碎片问题以及由此导致的内存不足崩溃。在纳德勒＆Associates的更小的嵌入式项目（大型航空电子产品），我们历来使用没有运行时动态内存。
	某些工具链（尤其是 C++ 的某些功能，单独文章的主题）在内部需要动态内存，即使您的应用程序不使用空闲存储调用。newlib C 运行时库（在许多嵌入式工具链中使用）在内部使用它自己的 malloc 系列例程。newlib 维护一些内部缓冲区并需要一些线程安全支持。
	本文解释了如何在带有 GNU 工具链的 FreeRTOS 项目中安全地使用 newlib。
 ## 因此，使用轻量级的printf函数
	https://github.com/mpaland/printf

# RTOS的特点：
 ## osSignalSet 语句耗时2.5us(168MHz)≈420cycles