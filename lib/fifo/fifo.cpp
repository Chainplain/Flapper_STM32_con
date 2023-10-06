#include "fifo.h"

FIFOClass::FIFOClass()
{
}

FIFOClass::FIFOClass(int bytesize, uint8_t *memory)
{
    //_buffer = new uint8_t[bytesize]; //不再使用动态申请内存了
    _buffer = memory;
    _size = bytesize;
    _writePtr = 0;
    _readPtr = 0;
    _occupy = 0;
    _lock = 0;
}

FIFOClass::~FIFOClass()
{
    // delete[] _buffer;
}

void FIFOClass::init(int bytesize, uint8_t *memory)
{
    //_buffer = new uint8_t[bytesize]; //不再使用动态申请内存了
    _buffer = memory;
    _size = bytesize;
    _writePtr = 0;
    _readPtr = 0;
    _occupy = 0;
    _lock = 0;
}

int FIFOClass::reset()
{
    if (_lock == 0)
    {
        _writePtr = 0;
        _readPtr = 0;
        _occupy = 0;
        return 1;
    }
    else
        return -1;
}

int FIFOClass::peek(uint8_t *data, int len)
{
    if (_buffer == NULL)
        return -1;

    uint8_t *des;
    uint8_t *src;

    int readptr = _readPtr;
    int occupy = _occupy;

    int i;

    des = data;
    src = &_buffer[readptr];
    int availableLen = (len <= occupy) ? len : occupy; // 可以操作的长度
    if ((availableLen + readptr) <= _size)
    {
        for (i = 0; i < availableLen; i++)
        {
            *des++ = *src++;
            occupy--;
            readptr++;
        }
    }
    else
    {
        int len1 = _size - readptr;
        for (i = 0; i < len1; i++)
        {
            *des++ = *src++;
            occupy--;
            readptr++;
        }
        readptr = 0;
        src = _buffer;
        len1 = availableLen - len1;
        for (i = 0; i < len1; i++)
        {
            *des++ = *src++;
            occupy--;
            readptr++;
        }
    }

    if (readptr >= _size)
    {
        readptr = 0;
    }
    return availableLen;
}

int FIFOClass::read(uint8_t *data, int len)
{
    if (_buffer == NULL)
        return -1;

    uint8_t *des;
    uint8_t *src;
    int i;

    if (_lock == 0)
    {
        _lock = 1;
        des = data;
        src = &_buffer[_readPtr];
        int availableLen = (len <= _occupy) ? len : _occupy; // 可以操作的长度
        if ((availableLen + _readPtr) <= _size)
        {
            for (i = 0; i < availableLen; i++)
            {
                *des++ = *src++;
                _occupy--;
                _readPtr++;
            }
        }
        else
        {
            int len1 = _size - _readPtr;
            for (i = 0; i < len1; i++)
            {
                *des++ = *src++;
                _occupy--;
                _readPtr++;
            }
            _readPtr = 0;
            src = _buffer;
            len1 = availableLen - len1;
            for (i = 0; i < len1; i++)
            {
                *des++ = *src++;
                _occupy--;
                _readPtr++;
            }
        }

        if (_readPtr >= _size)
        {
            _readPtr = 0;
        }
        _lock = 0;
        return availableLen;
    }
    else
    {
        return -1;
    }
}

int FIFOClass::write(uint8_t *data, int len)
{
    if (_buffer == NULL)
        return -1;

    uint8_t *des;
    uint8_t *src;
    int i;

    if (_lock == 0)
    {
        _lock = 1;
        src = data;
        des = &_buffer[_writePtr];
        int remain = _size - _occupy;
        int availableLen = (len <= remain) ? len : remain; // 可以操作的长度
        if ((availableLen + _writePtr) <= _size)
        {
            for (i = 0; i < availableLen; i++)
            {
                *des++ = *src++;
                _occupy++;
                _writePtr++;
            }
        }
        else
        {
            int len1 = _size - _writePtr;
            for (i = 0; i < len1; i++)
            {
                *des++ = *src++;
                _occupy++;
                _writePtr++;
            }
            _writePtr = 0;
            des = _buffer;
            len1 = availableLen - len1;
            for (i = 0; i < len1; i++)
            {
                *des++ = *src++;
                _occupy++;
                _writePtr++;
            }
        }

        if (_writePtr >= _size)
        {
            _writePtr = 0;
        }
        _lock = 0;
        return availableLen;
    }
    else
    {
        return -1;
    }
}

int FIFOClass::write(uint8_t data)
{
    if (_buffer == NULL)
        return -1;

    if (_lock == 0)
    {
        _lock = 1;
        int remain = _size - _occupy;
        if (remain > 0)
        {
            _buffer[_writePtr++] = data;
            if (_writePtr >= _size)
                _writePtr = 0;
        }
        _lock = 0;
        return 1;
    }
    else
    {
        return -1;
    }
}

int FIFOClass::pop(int len)
{
    if (len > _size)
        return -1;

    while (_lock)
    {
    }
    _lock = 1;
    if (_occupy < len)
    {
        _lock = 0;
        return -1;
    }
    else
    {
        _occupy -= len;
        _readPtr += len;
        while (_readPtr >= _size)
        {
            _readPtr -= _size;
        }
    }
    _lock = 0;
    return len;
}