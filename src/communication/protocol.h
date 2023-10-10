#ifndef protocol_h
#define protocol_h

#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_BUFFER_SIZE 256
#define FRAME_HEADER 0x55
#define PLOT_FIFO_SIZE (10 * 1024) // 10k bytes

class CommunicationFrame
{
public:
	CommunicationFrame();
	CommunicationFrame(uint8_t *buf, uint32_t headerlen = 2, uint32_t cmdlen = 1, uint32_t datanumlen = 1, uint32_t checklen = 1);
	void init(uint8_t *buf, uint32_t headerlen = 2, uint32_t cmdlen = 1, uint32_t datanumlen = 1, uint32_t checklen = 1);

public:
	uint8_t *_header;	 // Frame头的匹配字符地址
	uint32_t _headerLen; // Frame头长度
	uint8_t *_cmdPtr;
	uint32_t _cmdLen; // FrameCMD占用的字节数
	uint8_t *_dataNum;
	uint32_t _dataNumLen;	  // Frame数据长度占用的字节数
	uint8_t _dataNumMsbFirst; // 是否是先高字节后低字节
	uint8_t *_dataPtr;
	uint32_t _checkLen;		// 检验位占用的字节数
	uint8_t _checkMsbFirst; // 是否是先高字节后低字节
	uint32_t _headerEnd;	// 帧头结尾所在的索引位置
	uint32_t _cmdEnd;		//
	uint32_t _dataNumEnd;

	uint32_t _dataLen; // Frame数据占用的字节数
	uint32_t _index;   // 代表Frame长度的字节索引
	uint32_t _length;  // 代表Frame长度的字节数
	uint32_t _dataEnd;
	uint32_t _checkEnd;
};

class CommunicationProtocol
{
public:
	CommunicationProtocol();

public:
	virtual uint8_t send(uint8_t *buf, int length);
	virtual uint8_t send(char *str);
	virtual uint8_t send(uint8_t *cmd, uint8_t *data, uint32_t datalength);
	virtual uint8_t send(char cmd, uint8_t *data, int datalength);

protected:
	virtual void dataAnalysis();

protected:
	virtual int checkAnalysis(); // 校验函数
	virtual int checkGen();		 // 校验生成函数

public:
	// 接收数据的数据分析
	virtual int rxAnalysis(const uint8_t *const recvBuf, uint32_t length); // 数据处理函数
protected:
	virtual int rxAnalysis(char *str);
	// 用户发送数据的协议组成
	virtual int txConstruct(const uint8_t *const cmd, const uint8_t *const data, uint8_t datalength); // 帧生成
	virtual int txConstruct(const char *const str);
	virtual int txConstruct(const uint8_t *const buf, uint32_t len);

public:
	const uint8_t *const getTxBufPtr() { return _txBuffer; };
	const uint8_t *const getRxBufPtr() { return _rxBuffer; };
	int getMiniTxFrameLength() { return _txFrame._dataNumEnd + _txFrame._checkLen; }
	int getTxFrameLength() { return _txFrame._length; }

protected:
	CommunicationFrame _txFrame;
	CommunicationFrame _rxFrame;
	uint8_t _txBuffer[PROTOCOL_BUFFER_SIZE];
	uint8_t _rxBuffer[PROTOCOL_BUFFER_SIZE];
};

extern CommunicationProtocol *userComm;

#endif