#ifndef _DataAcquisition_H
#define _DataAcquisition_H

#include <stddef.h>
#include <stdint.h>
#include "communication/commDataAnalysis.h"
#include "fifo.h"
#include "cmsis_os.h"

#define SAMPLE_DATA_NUM 3
#define SAMPLE_DATA_FIFO_LENGTH (4096)

void vTimerInnerLoopOfDataAquisition(void *pvParameters);

class DataAcquisition
{
public:
    DataAcquisition(float ts = 1.0f);
    ~DataAcquisition();

    // 这里是必要的初始化
    // 1 采样补偿的计算周期，要小于等于采样时间。此周期ts就是放置process函数的外部loop函数的周期
    // 2 通讯接口类CommunicationProtocol的实例
public:
    void init(float ts, CommunicationProtocol *com);
    void samplePeriodSet(float ts) { _Ts = ts; }
    void commSet(CommunicationProtocol *com) { comm = com; }
    int startInnerLoop(void);
    int stop(void);
    bool IsUseInnerLoop() { return _isInnerLoop; }
    bool IsPeriodReached(uint8_t dataIndex); // 查看是否马上就要上传数据了

    // 这里是和通讯有关的两个接口函数
public:
    int send(uint8_t *buf, int len);
    int receiveCmd(uint8_t *buf);
    int customerSourceSelect(int index, int source);

    // 这里是和下位机有关的数据选择和定时发送接口函数
public:
    int sourceSelect(int index, float *source); // 选择数据源
    int process(void);                          // 放置于外部的一个循环(loop)函数里面
    friend void vTimerInnerLoopOfDataAquisition(void *pvParameters);

private:
    float _Ts;         // 采样时间
    bool _isInnerLoop; // 是否把process放到用内部循环里面
    osThreadId _uploadThreadID = NULL;
    osTimerId _innerLoopTimerID = NULL;
    int innerLoopProcess(void);
    int commandSend(uint8_t *buf, int len);

public:
    struct DataConstruct
    {
        bool en;
        float period;
        float tick;
        FIFOClass *fifo;
        float *value;
        int sourceIndex;
    };

public:
    DataConstruct _sampleData[SAMPLE_DATA_NUM];
    bool _en; // 总开关

private:
    float _nullData[SAMPLE_DATA_NUM];
    FIFOClass _fifo[SAMPLE_DATA_NUM];
    uint8_t _sampleFifoMemory[SAMPLE_DATA_NUM][SAMPLE_DATA_FIFO_LENGTH];

private:
    CommunicationProtocol *comm = NULL;
};

extern DataAcquisition dataAcq;

#endif