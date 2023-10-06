
#include "commDataAnalysis.h"
#include "main.h"
#include "dataAcquisition/dataAcquisition.h"

void float2bytes(const float *const fval, uint8_t *bval, int floatNum)
{
	FLOAT32_UNION fu32;
	int j = 0;
	for (int i = 0; i < floatNum; i++)
	{
		fu32.fval = fval[i];
		bval[j++] = fu32.uval.u1;
		bval[j++] = fu32.uval.u2;
		bval[j++] = fu32.uval.u3;
		bval[j++] = fu32.uval.u4;
	}
}

void double2bytes(const double *const dval, uint8_t *bval, int doubleNum)
{
	DOUBLE64_UNION du64;
	int j = 0;
	for (int i = 0; i < doubleNum; i++)
	{
		du64.dval = dval[i];
		bval[j++] = du64.luval.u1;
		bval[j++] = du64.luval.u2;
		bval[j++] = du64.luval.u3;
		bval[j++] = du64.luval.u4;
		bval[j++] = du64.luval.u5;
		bval[j++] = du64.luval.u6;
		bval[j++] = du64.luval.u7;
		bval[j++] = du64.luval.u8;
	}
}

void bytes2float(const uint8_t *const bval, float *fval, int floatNum)
{
	FLOAT32_UNION fu32;
	int j = 0;
	for (int i = 0; i < floatNum; i++)
	{
		fu32.uval.u1 = bval[j++];
		fu32.uval.u2 = bval[j++];
		fu32.uval.u3 = bval[j++];
		fu32.uval.u4 = bval[j++];
		fval[i] = fu32.fval;
	}
}

void bytes2double(const uint8_t *const bval, double *dval, int doubleNum)
{
	DOUBLE64_UNION du64;
	int j = 0;
	for (int i = 0; i < doubleNum; i++)
	{
		du64.luval.u1 = bval[j++];
		du64.luval.u2 = bval[j++];
		du64.luval.u3 = bval[j++];
		du64.luval.u4 = bval[j++];
		du64.luval.u5 = bval[j++];
		du64.luval.u6 = bval[j++];
		du64.luval.u7 = bval[j++];
		du64.luval.u8 = bval[j++];
		dval[i] = du64.dval;
	}
}

CommDataAnalysis::CommDataAnalysis()
{
}

void CommDataAnalysis::dataAnalysis()
{
	switch (_rxFrame._cmdPtr[0])
	{
	case TEST_CMD: // connect
		data_connect();
		break;
	case STRING_CMD:
		rs232_data_string();
		break;
	case RESET_CMD:
		rs232_data_reset();
		break;
	case AQUISITION_CMD: // data acquisition
		data_aquisition();
		break;
	case CONTROL_CMD:
		rs232_data_control();
		break;
	default:
		break;
	}
}

void CommDataAnalysis::data_connect()
{
	char buf[256];
	sprintf(buf, "FMAV is running... \r\n");
	send(buf);
}

void CommDataAnalysis::rs232_data_reset(void)
{
	char buf[240];
	uint8_t cmd;
	uint8_t len;

	// 发送codeIndex序号
	cmd = RESET_CMD;
	len = 1;	  // 'b'
	buf[0] = 'r'; // reset, 代表系统复位
	send(cmd, (uint8_t *)buf, len);
	delay(100);
	NVIC_SystemReset();
}

void CommDataAnalysis::rs232_data_string(void)
{
}

void CommDataAnalysis::data_aquisition()
{
	dataAcq.receiveCmd(_rxFrame._dataPtr);
}

void CommDataAnalysis::rs232_data_control()
{
	//char buf[250];
	const uint8_t *const rdata = _rxFrame._dataPtr;
	//int length = _rxFrame._dataLen;

	switch (rdata[0])
	{
	default:
		break;
	}
}
