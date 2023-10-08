#ifndef commDataAnalysis_h
#define commDataAnalysis_h

#include <stm32f4xx_hal.h>
#include "protocol.h"

class CommDataAnalysis : public CommunicationProtocol
{
public:
    CommDataAnalysis();

public:
    void data_connect();
    void rs232_data_reset();
    void rs232_data_string(void);
    void rs232_data_control(void);
    void data_aquisition();
    void data_read();

protected:
    void dataAnalysis();
    enum COMM_COMMAND
    {
        NO_CMD = 0,
        CONTROL_CMD = 'a',
        AQUISITION_CMD = 'f',
        RESET_CMD = 'r',
        STRING_CMD = 's',
        TEST_CMD = 'z',
        READ_CMD = 'd',
    };
};

void float2bytes(const float *const fval, uint8_t *bval, int floatNum);
void double2bytes(const double *const dval, uint8_t *bval, int doubleNum);
void bytes2float(const uint8_t *const bval, float *fval, int floatNum);
void bytes2double(const uint8_t *const bval, double *dval, int doubleNum);

#endif
