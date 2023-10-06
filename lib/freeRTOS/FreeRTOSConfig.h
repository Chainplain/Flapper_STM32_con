/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.2.1
 * Portion Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Portion Copyright (C) 2019 StMicroelectronics, Inc.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifdef __cplusplus
extern "C"{
#endif

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */   	      
/* Section where include file can be added */
/* USER CODE END Includes */ 

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
	//针对不同的编译器调用不同的stdint.h文件
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
#endif

/*****************************************************************
              FreeRTOS与内存申请有关配置选项                                               
*****************************************************************/
#define configSUPPORT_STATIC_ALLOCATION          0
#define configSUPPORT_DYNAMIC_ALLOCATION         1
// configTOTAL_HEAP_SIZE的size单位是字节
// L432 48kB SRAM is available, so 16kB for freeRTOS is OK, can run near 16 tasks
// F405 128kB SRAM is available, so 32kB for freeRTOS is OK, can run near 32 tasks
//      F405采用CCMRAM用于freeRTOS的内存，64k
#define configTOTAL_HEAP_SIZE                    ((size_t) 64 * 1024)


/************************************************************************
 *               FreeRTOS基础配置配置选项 
 *********************************************************************/
/* 置1：RTOS使用抢占式调度器；置0：RTOS使用协作式调度器（时间片）
 * 
 * 注：在多任务管理机制上，操作系统可以分为抢占式和协作式两种。
 * 协作式操作系统是任务主动释放CPU后，切换到下一个任务。
 * 任务切换的时机完全取决于正在运行的任务。
 */
#define configUSE_PREEMPTION                     1

//1使能时间片调度(默认式使能的)
//#define configUSE_TIME_SLICING					1

#define configENABLE_FPU                         1
#define configENABLE_MPU                         0

#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )

// RTOS系统节拍中断的频率。即一秒中断的次数，每次中断RTOS都会进行任务调度
#define configTICK_RATE_HZ                       ((TickType_t)1000)

/* 任务的优先级设置，实际可用的优先级为 0 -> configMAX_PRIORITIES -1 ; 
 * 所以这里是 0 -> 14
 * 任务的优先级数值越小，那么此任务的优先级越低，空闲任务的优先级是 0
 * cortex内核决定最高优先级最好不要超过32
 */
#define configMAX_PRIORITIES					( 15 ) 

// 空闲任务的堆栈大小为configMINIMAL_STACK_SIZE
// 注意这里的stack size单位是word，
// 所以空闲任务所要占用整个freeRTOS申请下来的内存（total_heap_size定义了此块RAM大小）的大小是
// 128 * 4 + TCB(84) = 512 + 84 = 596字节
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)

#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_16_BIT_TICKS                   0
//空闲任务可以放弃CPU使用权给其他同优先级的用户任务
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8

/* 某些运行FreeRTOS的硬件有两种方法选择下一个要执行的任务：
 * 通用方法和特定于硬件的方法（以下简称“特殊方法”）。
 * 
 * 通用方法：
 *      1.configUSE_PORT_OPTIMISED_TASK_SELECTION 为 0 或者硬件不支持这种特殊方法。
 *      2.可以用于所有FreeRTOS支持的硬件
 *      3.完全用C实现，效率略低于特殊方法。
 *      4.不强制要求限制最大可用优先级数目
 * 特殊方法：
 *      1.必须将configUSE_PORT_OPTIMISED_TASK_SELECTION设置为1。
 *      2.依赖一个或多个特定架构的汇编指令（一般是类似计算前导零[CLZ]指令）。
 *      3.比通用方法更高效
 *      4.一般强制限定最大可用优先级数目为32
 * 一般是硬件计算前导零指令，如果所使用的，MCU没有这些硬件指令的话此宏应该设置为0！
 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1


/* 置1：使能低功耗tickless模式；置0：保持系统节拍（tick）中断一直运行
 * 假设开启低功耗的话可能会导致下载出现问题，因为程序在睡眠中,可用以下办法解决
 * 
 * 下载方法：
 *      1.将开发版正常连接好
 *      2.按住复位按键，点击下载瞬间松开复位按键
 *     
 *      1.通过跳线帽将 BOOT 0 接高电平(3.3V)
 *      2.重新上电，下载
 *    
 * 			1.使用FlyMcu擦除一下芯片，然后进行下载
 *			STMISP -> 清除芯片(z)
 */
//#define configUSE_TICKLESS_IDLE													0



/* USER CODE BEGIN MESSAGE_BUFFER_LENGTH_TYPE */
/* Defaults to size_t for backward compatibility, but can be changed
   if lengths will always be less than the number of bytes in a size_t. */
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t
/* USER CODE END MESSAGE_BUFFER_LENGTH_TYPE */ 

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( 2 )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             256

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet             1
#define INCLUDE_uxTaskPriorityGet            1
#define INCLUDE_vTaskDelete                  1
#define INCLUDE_vTaskCleanUpResources        0
#define INCLUDE_vTaskSuspend                 1
#define INCLUDE_vTaskDelayUntil              1
#define INCLUDE_vTaskDelay                   1
#define INCLUDE_xTaskGetSchedulerState       1
#define INCLUDE_uxTaskGetStackHighWaterMark  1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
 /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

/****************************************************************
 定义适配freeRTOS的中断的最高和最低优先级
*****************************************************************/
/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
// 注意系统时钟滴答器的中断 TICK_INT_PRIORITY =  0x0F = 15，因此这里的值要 大于等于15
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15 
/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );} 
/* USER CODE END 1 */

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: This define is commented when used with STM32Cube firmware, when the timebase source is SysTick,
              to prevent overwriting SysTick_Handler defined within STM32Cube HAL */
 
// #define xPortSysTickHandler SysTick_Handler

/* USER CODE BEGIN Defines */   	      
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
#define configAPPLICATION_ALLOCATED_HEAP 1 // ucHeap allocated in freertos.c							
/* USER CODE END Defines */ 


#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_CONFIG_H */
