
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
	case READ_CMD: // read
		data_read();
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
	char buf[250];
	const uint8_t *const rdata = _rxFrame._dataPtr;
	int length = _rxFrame._dataLen;

	switch (rdata[0])
	{
	case 'i': // imu read and set
	{
		if (rdata[1] == 'r')
		{
			if (rdata[2] == 'a') // example
			{
				sprintf(buf, "IMU info: zitai[%0.0f, %0.0f, %0.0f], acc[%g, %g, %g]m/s^2\r\tmagnet[%0.0f, %0.0f, %0.0f], pressure[%0.0f]Pa, altitude[%0.3f]m, temperature[%0.0f]C\r\n",
						imuHandle.zitai[0], imuHandle.zitai[1], imuHandle.zitai[2],
						imuHandle.acc[0], imuHandle.acc[1], imuHandle.acc[2],
						imuHandle.magnet[0], imuHandle.magnet[1], imuHandle.magnet[2],
						imuHandle.pressure, imuHandle.altitude, imuHandle.temperature);
				send(buf);
			}
			else if (rdata[2] == 'a') // example
			{
				sprintf(buf, "zitai[%g, %g, %g]\r\n", imuHandle.zitai[0], imuHandle.zitai[1], imuHandle.zitai[2]);
				send(buf);
			}
		}
		break;
	}
	case 'w': // esp wifi network
	{
		if (rdata[1] == 's')
		{
		}
		else if (rdata[1] == 'r')
		{
		}
		else if (rdata[1] == 'd') // send data to
		{
			uart_esp_send((uint8_t *)&rdata[2], length - 2);
		}
		break;
	}
	default:
		break;
	}
}

void CommDataAnalysis::data_read()
{
	uint8_t buf[256];
	const uint8_t *const rdata = _rxFrame._dataPtr;

	switch (rdata[0])
	{
	case 'p':
	{
		buf[0] = 'p';
		for (int i = 0; i < 15; i++)
		{
			buf[i + 1] = {Device_Status_get(i)};
		}
		send((char)READ_CMD, buf, 16);
		break;
	}
	case 'v':
	{
		buf[0] = 'v';
		for (int j = 0; j < 16; j++)
		{
			buf[j + 1] = {Device_Statusf_get(j)};
		}
		send((char)READ_CMD, buf, 17);
		break;
	}
	default:
		break;
	}
}