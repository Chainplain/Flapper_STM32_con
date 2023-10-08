#ifndef setup_h
#define setup_h

#include <stm32f4xx_hal.h>

struct SAMPLE_PLOT
{
    uint8_t mode; // 采样模式选择 [无采样，触发模式，连续模式]
    uint8_t controlindex; // 触发模式下第几个controllist的序号被触发
    uint8_t plotSource[3]; // 数据源选择
    float frequency; // 采样频率
    uint32_t number; // 采样个数
    uint32_t numcnt;
    float timecnt;
};


void vThreadDataAcqSetup(void *pvParameters);
void vThreadDataAquisition(void *pvParameters);

extern SAMPLE_PLOT sample_plot;

#endif
