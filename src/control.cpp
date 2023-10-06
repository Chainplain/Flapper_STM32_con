/************************************************************
 * FileName:        control.cpp
 * Description:     控制
 *                  STM32
 * Auther:          Jinsheng
 * CreateDate:      2021-07-12
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include "control.h"
#include "main.h"
#include "math.h"

#include "dataAcquisition/dataAcqSetup.h"
#define DATA_ACQUISITION_EN 1
osThreadId dataacq_thread_id;

sAdcVal sval;

void vTaskControl(void *pvParameters)
{
    osDelay(100);
    char buf[100];
    sprintf(buf, "FMAV controller is running\r\n");
    userComm->send(buf);

    float duty = 0.1f;
    for (int i = 0; i < 7; i++)
    {
        PWM_duty_set(i, duty);
    }
        
    while (1)
    {
        
        vTaskDelayMs(100);

        // sprintf(buf, "color [x:%d, y:%d], error [x=%0.0f, y=%0.0f], current [a:%0.1f, b:%0.1f, c:%0.1f]\r\n", colorPosX, colorPosY, colorXPID.err, colorYPID.err, curPid[0].ref, curPid[1].ref, curPid[2].ref);
        // userComm->send(buf);
    }

    // 如果任务不是永久性的需要调用 vTaskDelete 删除任务
    // vTaskDelete(NULL);
}

void adc_data_received(sAdcVal *adc_val)
{
    sval = *adc_val;
   // uint8_t i;
}
