#include "dataAcquisition.h"
#include "cmsis_os.h"
#include "math.h"
#include "printf.h"
#include "string.h"

DataAcquisition dataAcq;

void vThreadUploadOfDataAquisition(void *pvParameters);
// void vTimerInnerLoopOfDataAquisition(void *pvParameters);

DataAcquisition::DataAcquisition(float ts)
{
    _Ts = ts;
    _isInnerLoop = false;
    _en = false;
    for (int i = 0; i < SAMPLE_DATA_NUM; i++)
    {
        _fifo[i].init(SAMPLE_DATA_FIFO_LENGTH, _sampleFifoMemory[i]);
        _sampleData[i].fifo = &_fifo[i];
        _sampleData[i].value = &_nullData[i];
        _sampleData[i].period = 1.0f;
        _sampleData[i].tick = 0;
        _sampleData[i].en = false;
        _sampleData[i].sourceIndex = -1;
    }
}

void DataAcquisition::init(float ts, CommunicationProtocol *com)
{
    _Ts = ts;
    comm = com;

    osThreadDef(uploadOfDataAcquisition, vThreadUploadOfDataAquisition, osPriority::osPriorityLow, 0, 256);
    _uploadThreadID = osThreadCreate(osThread(uploadOfDataAcquisition), this);
}

DataAcquisition::~DataAcquisition()
{
    if (_uploadThreadID != NULL)
        osThreadTerminate(_uploadThreadID);
}

// 默认的command字符是'f', 你也可以修改为其它字符；修改后上位机也要一同修改
int DataAcquisition::send(uint8_t *buf, int len)
{
    if (comm != NULL)
        return comm->send('f', buf, len);
    else
        return -1;
}

// 默认的command字符是'F', 你也可以修改为其它字符；修改后上位机也要一同修改
int DataAcquisition::commandSend(uint8_t *buf, int len)
{
    if (comm != NULL)
        return comm->send('F', buf, len);
    else
        return -1;
}

int DataAcquisition::sourceSelect(int index, float *source)
{
    if (index >= SAMPLE_DATA_NUM)
        return -1;

    _sampleData[index].value = source;

    return 1;
}

int DataAcquisition::process(void)
{
    if (_isInnerLoop)
        return -1;
    return innerLoopProcess();
}

int DataAcquisition::innerLoopProcess(void)
{
    if (!_en)
        return -1;

    DataConstruct *data;
    for (int i = 0; i < SAMPLE_DATA_NUM; i++)
    {
        data = &_sampleData[i];
        if (data->en)
        {
            data->tick += _Ts;
            if (data->tick >= data->period)
            {
                data->tick -= data->period;
                if (data->fifo->remainedSize() >= 4)
                {
                    data->fifo->write((uint8_t *)(data->value), 4);
                }
            }
        }
    }

    return 0;
}

int DataAcquisition::receiveCmd(uint8_t *buf)
{
    float tmp[SAMPLE_DATA_NUM];
    uint8_t *sourceBuf;
    char command[256];
    char *sbuf = &command[1];
    bool flag = false;

    switch (buf[0])
    {
    case 'a': // all, [en, period, source]
    {
        if (buf[1] == 's')
        {
            // buf2-4: en, buf5-16: period, buf17-19: sourceIndex
            bytes2float(&buf[SAMPLE_DATA_NUM + 2], tmp, SAMPLE_DATA_NUM);
            sourceBuf = &buf[SAMPLE_DATA_NUM * 5 + 2];
            for (int i = 0; i < SAMPLE_DATA_NUM; i++)
            {
                _sampleData[i].en = (buf[i + 2] == 1);
                _sampleData[i].period = tmp[i];
                _sampleData[i].tick = 0;
                customerSourceSelect(i, sourceBuf[i]);
                _sampleData[i].fifo->reset();
            }

            command[0] = 'a';
            sprintf(sbuf, "sample set done\r\n");
            commandSend((uint8_t *)command, strlen(command));
        }
        else if (buf[1] == 'r')
        {
            command[0] = 'A';
            sprintf(sbuf, "en=[%u, %u, %u], period=[%0.3f, %0.3f, %0.3f], source=[%d, %d, %d]\r\n",
                    _sampleData[0].en,
                    _sampleData[1].en,
                    _sampleData[2].en,
                    _sampleData[0].period,
                    _sampleData[1].period,
                    _sampleData[2].period,
                    _sampleData[0].sourceIndex,
                    _sampleData[1].sourceIndex,
                    _sampleData[2].sourceIndex);
            commandSend((uint8_t *)command, strlen(command));
        }
        break;
    }
    case 'b': // 更改 采样周期， ts，要严格和循环周期一致
    {
        if (buf[1] == 's')
        {
            bytes2float(&buf[2], tmp, 1);
            _Ts = tmp[0];
            if ((_isInnerLoop) && (_innerLoopTimerID != NULL))
            {
                osTimerStart(_innerLoopTimerID, roundf(_Ts * 1000));
            }

            command[0] = 'b';
            sprintf(sbuf, "loop Ts set done\r\n");
            commandSend((uint8_t *)command, strlen(command));
        }
        else if (buf[1] == 'r')
        {
            command[0] = 'B';
            sprintf(sbuf, "Ts=%0.3f\r\n", _Ts);
            commandSend((uint8_t *)command, strlen(command));
        }
        break;
    }
    case 'e': // 总开关
    {
        flag = false;
        if (buf[1] == '1')
        {
            _en = true;
            flag = true;
        }
        else if (buf[1] == '0')
        {
            _en = false;
            flag = true;
        }

        if (flag)
        {
            command[0] = 'e';
            if (_en)
                sprintf(sbuf, "dataAcq start...\r\n");
            else
                sprintf(sbuf, "dataAcq stop\r\n");
            commandSend((uint8_t *)command, strlen(command));
        }
        break;
    }
    case 'i': // inner loop 设置
    {
        if (buf[1] == 's')
        {
            if (buf[2] == '1')
            {
                startInnerLoop();
            }
            else if (buf[2] == '0')
            {
                stop();
            }

            command[0] = 'i';
            sprintf(sbuf, "inner loop set done\r\n");
            commandSend((uint8_t *)command, strlen(command));
        }
        break;
    }
    case 't': // en, period
    {
        bytes2float(&buf[SAMPLE_DATA_NUM + 1], tmp, SAMPLE_DATA_NUM);
        for (int i = 0; i < SAMPLE_DATA_NUM; i++)
        {
            _sampleData[i].en = (buf[i + 1] == '1');
            _sampleData[i].period = tmp[i];
            _sampleData[i].tick = 0;
        }

        command[0] = 't';
        sprintf(sbuf, "en & period set done\r\n");
        commandSend((uint8_t *)command, strlen(command));
        break;
    }
    case 's': // source
    {
        sourceBuf = &buf[1];
        for (int i = 0; i < SAMPLE_DATA_NUM; i++)
        {
            customerSourceSelect(i, sourceBuf[i]);
            _sampleData[i].fifo->reset();
        }

        command[0] = 's';
        sprintf(sbuf, "source set done\r\n");
        commandSend((uint8_t *)command, strlen(command));
        break;
    }
    default:
        break;
    }

    return 0;
}

// 这个是用户自定义的数据源选择
// 更新这个函数方法:
// 1 把该函数注释掉，然后在其它文件里重新写这个函数，注意其它文件需要引用 #include "dataAcquisition.h"
// 2 在这里编辑该函数，需要声明所用到的数据变量来源，采用
//     ---- extern 的方式
//     ---- #include 的方式
__attribute__((weak)) int DataAcquisition::customerSourceSelect(int index, int source)
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
    default:
        //_sampleData[index].value = &_nullData[index];
        break;
    }

    return 0;
}

int DataAcquisition::startInnerLoop(void)
{
    osTimerDef(innerLoopOfDataAcquisition, vTimerInnerLoopOfDataAquisition);
    _innerLoopTimerID = osTimerCreate(osTimer(innerLoopOfDataAcquisition), osTimerPeriodic, this);
    osTimerStart(_innerLoopTimerID, roundf(_Ts * 1000));
    _isInnerLoop = true;
    return 0;
}

int DataAcquisition::stop(void)
{
    if (_isInnerLoop)
    {
        if (_innerLoopTimerID != NULL)
        {
            osTimerStop(_innerLoopTimerID);
            osTimerDelete(_innerLoopTimerID);
        }
        _isInnerLoop = false;
    }

    for (int i = 0; i < SAMPLE_DATA_NUM; i++)
    {
        _sampleData[i].en = false;
    }

    return 0;
}

bool DataAcquisition::IsPeriodReached(uint8_t dataIndex)
{
    if ((_sampleData[dataIndex].tick + _Ts) >= _sampleData[dataIndex].period)
        return true;
    else
        return false;
}

// 上传数据给上位机的任务
void vThreadUploadOfDataAquisition(void *pvParameters)
{
    uint8_t buf[256];
    DataAcquisition *dtAcqHandle = (DataAcquisition *)pvParameters;
    DataAcquisition::DataConstruct *data;
    int len, sendlen;
    while (1)
    {
        data = dtAcqHandle->_sampleData;
        for (int i = 0; i < SAMPLE_DATA_NUM; i++)
        {
            len = data[i].fifo->occupiedSize();
            while (len > 0)
            {
                sendlen = (len < 240) ? len : 240;
                len -= sendlen;
                if (sendlen > 0)
                {
                    buf[0] = i;
                    buf[1] = sendlen;
                    int result = data[i].fifo->peek(&buf[2], sendlen);
                    if (result > 0)
                    {
                        while (data[i].fifo->pop(result) < 0)
                        {
                        }
                        dtAcqHandle->send(buf, result + 2);
                    }
                }
            }
        }

        osDelay(100); // 0.1s发送一次
    }
}

// 内部循环定时器
// 由于timer没法传递参数的，只能用全局变量来搞，这个有点不好，freeRTOS需要改进
void vTimerInnerLoopOfDataAquisition(void *pvParameters)
{
    // timer传递的是个假参数
    //((DataAcquisition *)pvParameters)->process();
    dataAcq.innerLoopProcess();
}
