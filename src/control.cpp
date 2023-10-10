#include "control.h"
#include "main.h"
#include "math.h"

#include "dataAcquisition/dataAcqSetup.h"
#define DATA_ACQUISITION_EN 1
#define STREAMING_PERIOD 500
osThreadId dataacq_thread_id;

sAdcVal sval;

void vTaskControl(void *pvParameters)
{
    osDelay(100);
    char buf[100];
    char sendout_buf[500];

    sprintf(buf, "FMAV controller is running by chainplain.\r\n");
    userComm->send(buf);

    float duty = 0.1f;
    for (int i = 0; i < 7; i++)
    {
        PWM_duty_set(i, duty);
    }
        
    while (1)
    {
        
        vTaskDelayMs(500);
        // sprintf(sendout_buf, "IMU info: zitai[%0.0f, %0.0f, %0.0f], acc[%g, %g, %g]m/s^2\r\tmagnet[%0.0f, %0.0f, %0.0f], pressure[%0.0f]Pa, altitude[%0.3f]m, temperature[%0.0f]C\r\n",
		// 				imuHandle.zitai[0], imuHandle.zitai[1], imuHandle.zitai[2],
		// 				imuHandle.acc[0], imuHandle.acc[1], imuHandle.acc[2],
		// 				imuHandle.magnet[0], imuHandle.magnet[1], imuHandle.magnet[2],
		// 				imuHandle.pressure, imuHandle.altitude, imuHandle.temperature);
		// userComm->send(sendout_buf);
        // sprintf(buf, "color [x:%d, y:%d], error [x=%0.0f, y=%0.0f], current [a:%0.1f, b:%0.1f, c:%0.1f]\r\n", colorPosX, colorPosY, colorXPID.err, colorYPID.err, curPid[0].ref, curPid[1].ref, curPid[2].ref);
        // userComm->send(buf);
    }

    // 如果任务不是永久性的需要调用 vTaskDelete 删除任务
    // vTaskDelete(NULL);
}

void vTASKSteamingOnboardInfos(void *pvParameters)
{
    char sendout_buf[500];
    while (1)
    {
        vTaskDelayMs(500);
        sprintf(sendout_buf, "IMU info: zitai[%0.0f, %0.0f, %0.0f], acc[%g, %g, %g]m/s^2\r\tmagnet[%0.0f, %0.0f, %0.0f], pressure[%0.0f]Pa, altitude[%0.3f]m, temperature[%0.0f]C\r\n",
						imuHandle.zitai[0], imuHandle.zitai[1], imuHandle.zitai[2],
						imuHandle.acc[0], imuHandle.acc[1], imuHandle.acc[2],
						imuHandle.magnet[0], imuHandle.magnet[1], imuHandle.magnet[2],
						imuHandle.pressure, imuHandle.altitude, imuHandle.temperature);
		userComm->send(sendout_buf);
        // sprintf(buf, "color [x:%d, y:%d], error [x=%0.0f, y=%0.0f], current [a:%0.1f, b:%0.1f, c:%0.1f]\r\n", colorPosX, colorPosY, colorXPID.err, colorYPID.err, curPid[0].ref, curPid[1].ref, curPid[2].ref);
        // userComm->send(buf);
    }
}

void adc_data_received(sAdcVal *adc_val)
{
    sval = *adc_val;
   // uint8_t i;
}
