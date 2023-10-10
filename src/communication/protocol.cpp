

#include "protocol.h"
#include "string.h"

CommunicationProtocol *userComm = NULL;

CommunicationFrame::CommunicationFrame()
{
    _header = NULL;
}

CommunicationFrame::CommunicationFrame(uint8_t *buf, uint32_t headerlen, uint32_t cmdlen, uint32_t datanumlen, uint32_t checklen)
{
    init(buf, headerlen, cmdlen, datanumlen, checklen);
}

void CommunicationFrame::init(uint8_t *buf, uint32_t headerlen, uint32_t cmdlen, uint32_t datanumlen, uint32_t checklen)
{
    _header = buf;
    _headerLen = headerlen;
    _cmdPtr = _header + _headerLen;
    _cmdLen = cmdlen;
    _dataNum = _cmdPtr + _cmdLen;
    _dataNumLen = datanumlen;
    _dataNumMsbFirst = 1;
    _dataPtr = _dataNum + _dataNumLen;
    _dataLen = 0;
    _checkLen = checklen;
    _checkMsbFirst = 1;
    _headerEnd = _headerLen;
    _cmdEnd = _headerEnd + _cmdLen;
    _dataNumEnd = _cmdEnd + _dataNumLen;
    _index = 0;
    _length = 0;

    for (uint32_t i = 0; i < _headerLen; i++)
    {
        _header[i] = FRAME_HEADER;
    }
}

//
CommunicationProtocol::CommunicationProtocol()
{
    _txFrame.init(_txBuffer);//init之后就会有确定的数据格式，因为已经设定：
    /*
    	void init(uint8_t *buf, uint32_t headerlen = 2, uint32_t cmdlen = 1, uint32_t datanumlen = 1, uint32_t checklen = 1);
    */
    _rxFrame.init(_rxBuffer);
}

// frame数据生成
int CommunicationProtocol::txConstruct(const uint8_t *const cmd, const uint8_t *const data, uint8_t datalength)
{
    // 命令
    for (uint32_t i = 0; i < _txFrame._cmdLen; i++)
    {
        _txFrame._cmdPtr[i] = cmd[i];
    }

    // 数据长度
    if (_txFrame._dataNumMsbFirst) // 高位在前
    {
        for (uint32_t i = 0; i < _txFrame._dataNumLen; i++)
        {
            _txFrame._dataNum[i] = (datalength >> ((_txFrame._dataNumLen - i - 1) * 8));
        }
    }
    else // 低位在前
    {
        for (uint32_t i = 0; i < _txFrame._dataNumLen; i++)
        {
            _txFrame._dataNum[i] = (datalength >> (i * 8));
        }
    }

    // 数据
    for (uint32_t i = 0; i < datalength; i++)
    {
        _txFrame._dataPtr[i] = data[i];
    }

    // 校验位
    _txFrame._dataLen = datalength;
    _txFrame._index = 0;
    _txFrame._dataEnd = _txFrame._dataNumEnd + _txFrame._dataLen;
    _txFrame._checkEnd = _txFrame._dataEnd + _txFrame._cmdLen;
    _txFrame._length = _txFrame._checkEnd;
    checkGen();
    return 0;
}

int CommunicationProtocol::txConstruct(const char *const str)
{
    uint8_t cmd = 's';
    uint8_t *data = (uint8_t *)(str);
    uint32_t datalength = strlen(str);
    return txConstruct(&cmd, data, datalength);
}

int CommunicationProtocol::txConstruct(const uint8_t *const buf, uint32_t len)
{
    uint8_t cmd = buf[0];
    const uint8_t *const data = &buf[1];
    if (len < 1)
        return -1;
    len--; // 减去cmd的长度
    return txConstruct(&cmd, data, len);
}

int CommunicationProtocol::rxAnalysis(char *str)
{
    const uint8_t *const recvBuf = (const uint8_t *const)str;
    uint32_t length = strlen(str);
    return rxAnalysis(recvBuf, length);
}

/**  采用 Frame头 + 包长度 的通讯协议
 **/
int CommunicationProtocol::rxAnalysis(const uint8_t *const recvBuf, uint32_t length)
{
    uint8_t dataTmp;
    uint32_t tmp;

    for (uint32_t i = 0; i < length; i++)
    {
        dataTmp = recvBuf[i];

        // 帧头 header
        if (_rxFrame._index < _rxFrame._headerEnd)
        { // 处于Frame头位置，需要判断Frame头是否正确
            if (dataTmp != _rxFrame._header[_rxFrame._index])
            { // 如果Frame头对应不上，从当前字节开始认为下一个Frame的起点
                if (dataTmp != _rxFrame._header[0])
                {
                    _rxFrame._index = 0;
                    continue;
                }
                else
                {
                    _rxFrame._header[0] = dataTmp;
                    _rxFrame._index++;
                    continue;
                }
            }
            else
            {
                _rxFrame._header[_rxFrame._index++] = dataTmp;
                continue;
            }
        }

        // 命令 cmd
        if (_rxFrame._index < _rxFrame._cmdEnd)
        {
            _rxFrame._header[_rxFrame._index++] = dataTmp;
            continue;
        }

        // 数据长度
        if (_rxFrame._index < _rxFrame._dataNumEnd)
        {
            _rxFrame._header[_rxFrame._index++] = dataTmp;
            /// 计算数据长度
            if (_rxFrame._index == _rxFrame._dataNumEnd) // 数据长度接收完整了，可以计算数据长度了
            {
                tmp = 0;
                if (_rxFrame._dataNumMsbFirst == 0) // 代表长度的数据数组中，高字节在后
                {
                    for (uint32_t j = 0; j < _rxFrame._dataNumLen; j++)
                    {
                        tmp += (_rxFrame._dataNum[j] << (j * 8));
                    }
                }
                else // 代表长度的数据数组中，高字节在前
                {
                    for (uint32_t j = 0; j < _rxFrame._dataNumLen; j++)
                    {
                        tmp += (_rxFrame._dataNum[j] << ((_rxFrame._dataNumLen - j - 1) * 8));
                    }
                }
                _rxFrame._dataLen = tmp;
                _rxFrame._dataEnd = _rxFrame._dataNumEnd + _rxFrame._dataLen;
                _rxFrame._checkEnd = _rxFrame._dataEnd + _rxFrame._checkLen;
                _rxFrame._length = _rxFrame._checkEnd;
            }
            continue;
        }

        // 数据
        if (_rxFrame._index < _rxFrame._dataEnd)
        {
            _rxFrame._header[_rxFrame._index++] = dataTmp;
            continue;
        }

        // 校验
        if (_rxFrame._index < _rxFrame._checkEnd)
        {
            _rxFrame._header[_rxFrame._index++] = dataTmp;
            if (_rxFrame._index == _rxFrame._length)
            {
                if (checkAnalysis() == 0) // 校验OK
                    dataAnalysis();
                _rxFrame._index = 0;
            }
            continue;
        }
    }

    return 0;
}

// 求和校验
int CommunicationProtocol::checkAnalysis()
{
    int len = _rxFrame._length - _rxFrame._checkLen;
    uint8_t *checkPtr = _rxFrame._header + _rxFrame._dataEnd;
    int check = 0;

    if (len >= (int)_rxFrame._dataNumEnd)
    {
        for (int i = 0; i < len; i++)
        {
            check += _rxFrame._header[i];
        }

        for (int i = 0; i < (int)_rxFrame._checkLen; i++)
        {
            if (_rxFrame._checkMsbFirst) // 高位在前
            {
                if (checkPtr[i] != ((check >> ((_rxFrame._checkLen - i - 1) * 8)) & 0xFF))
                {
                    return -1;
                }
            }
            else // 低位在前
            {
                if (checkPtr[i] != ((check >> ((i)*8)) & 0xFF))
                {
                    return -1;
                }
            }
        }

        return 0;
    }
    else
        return -1;
}

// 求和校验码生成
int CommunicationProtocol::checkGen()
{
    int check = 0;
    uint8_t *checkPtr = _txFrame._header + _txFrame._dataEnd;

    // 除去校验位，所有数据的和
    for (uint32_t i = 0; i < _txFrame._dataEnd; i++)
    {
        check += _txFrame._header[i];
    }

    if (_txFrame._checkMsbFirst) // 高位在前
    {
        for (uint32_t i = 0; i < _txFrame._checkLen; i++)
        {
            checkPtr[i] = (check >> ((_txFrame._checkLen - i - 1) * 8)) & 0xFF;
        }
    }
    else // 低位在前
    {
        for (uint32_t i = 0; i < _txFrame._checkLen; i++)
        {
            checkPtr[i] = (check >> (i * 8)) & 0xFF;
        }
    }

    return 0;
}

// virtual func part
uint8_t CommunicationProtocol::send(uint8_t *buf, int length)
{
    return 0;
}
uint8_t CommunicationProtocol::send(char *str)
{
    return 0;
}
uint8_t CommunicationProtocol::send(uint8_t *cmd, uint8_t *data, uint32_t datalength)
{
    return 0;
}
uint8_t CommunicationProtocol::send(char cmd, uint8_t *data, int datalength)
{
    return 0;
}
void CommunicationProtocol::dataAnalysis()
{
}
