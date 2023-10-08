

#ifndef _UART_IMU_H
#define _UART_IMU_H

#include "stm32f4xx_hal.h"
#include "communication/uart_232.h"

#define IMU_ACC_RANGE (4)
#define IMU_GYPO_RANGE (2000)

struct IMU_TypeOf_Handle
{
    // 姿态角度 x-y-z
    // x-roll  [-180, 180]
    // y-pitch [-90, 90]
    // z-yaw   [-180, 180]
    float zitai[3];

    // 四元数
    // q0-q1-q2-q3
    // [0 1]
    float siyuanshu[4]; 

    // 陀螺仪 x-y-z
    // 旋转速率 [-2000, 2000] unit °/S
    float gyro[3];

    // 加速度 x-y-z
    // [-4, 4] unit G, 1G=9.8m/s^2
    float acc[3];

    // 磁力 x-y-z
    // [-32768, 32768]
    float magnet[3];

    // 温度 摄氏度
    float temperature;

    // 气压 unit Pa
    float pressure;

    // 海拔高度 unit m
    float altitude;
};

UART232_Handle *rs232_imu_init(void);

void imu_baud_set(uint8_t baudrate);
void imu_returnrate_set(uint8_t returnrate);
void imu_returncontext_set(uint8_t returncontext);
void imu_alg_set(uint8_t alg);
void imu_save(void);
void imu_gyro_range_set(int range);
void imu_acc_range_set(int range);
void imu_led_set(bool on);

extern IMU_TypeOf_Handle imuHandle;

#endif
