/************************************************************
 * FileName:        Enable Management.cpp
 * Description:     所有开关的使能控制
 *                  STM32
 * Auther:          Jinsheng
 * CreateDate:      2021-07-12
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include "dataAcquisition/dataAcqSetup.h"
#include "main.h"
#include "arm_math.h"
#include "fifo.h"
#include "dataAcquisition/dataAcquisition.h"

const int16_t colorPosXRef = 640 / 2;
const int16_t colorPosYRef = 480 / 2;
// void currentFbGet(float *vol, float *cur);

// 数据采样方式：是否采样内部循环的方法
#define USE_INNER_LOOP 0

float sinVal = 0, cosVal = 0;
int loopPeriodMs = 1;

// ------------------------------------------------------------------------------------
void vThreadDataAcqSetup(void *pvParameters)
{
    // uint8_t state = 0;
    // char info[200];

    // delay(10); // wait for uart ready
    // usb_init();

    // sprintf(info, "starting motor control\n");
    // userComm->send(info);
    // vTaskDelay(10);

    // float sinFreq = 1.0f; // 1Hz
    // float sinT = 0;
    // float sinTs = loopPeriodMs * 1e-3f;

    // 数据采集初始化
    dataAcq.init(loopPeriodMs * 1e-3f, userComm); // loop周期loopPeriodMs毫秒， 通讯接口为userComm

    // 后面的3行可以由上位机配置
    dataAcq._sampleData[0].en = true;
    dataAcq._sampleData[0].period = 10e-3f;
    dataAcq._sampleData[0].value = &sinVal;

    dataAcq._sampleData[1].en = true;
    dataAcq._sampleData[1].period = 10e-3f;
    dataAcq._sampleData[1].value = &cosVal;

    if (USE_INNER_LOOP == 1)
        dataAcq.startInnerLoop();

    while (1)
    {
        // // 生成一个正弦数据
        // sinT += sinTs;
        // if (sinT > 1.0f)
        //     sinT -= 1.0f;
        // sinVal = sinf(2 * PI * sinFreq * sinT);

        // 调用数据采集处理器
        dataAcq.process();

        // loop周期
        delay(loopPeriodMs);
    }

    vTaskDelete(NULL);
}

// 重写 数据源 成员函数
int DataAcquisition::customerSourceSelect(int index, int source)
{
    if (index >= SAMPLE_DATA_NUM)
        return -1;

    _sampleData[index].sourceIndex = source;
    switch (source)
    {
        // an example:
    // case 0:
    //{
    //     _sampleData[index].value = &speed;
    //     break;
    // }
    case 0: // lambdaAngle
    {
        _sampleData[index].value = &sinVal;
        break;
    }
    case 1: // lambdaAngleRef
    {
        _sampleData[index].value = &cosVal;
        break;
    }
    default:
        //_sampleData[index].value = &_nullData[index];
        break;
    }

    return 0;
}
