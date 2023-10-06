#ifndef _FIFO_H
#define _FIFO_H

#include <stddef.h>
#include <stdint.h>

class FIFOClass
{
public:
    /**必须给出fifo内存地址memory和大小bytesize
     * 注意内存地址memory最好不要动态申请
     */
    FIFOClass(int bytesize, uint8_t *memory);
    FIFOClass();
    ~FIFOClass();
    /**必须给出fifo内存地址memory和大小bytesize
     * 注意内存地址memory最好不要动态申请
     */
    void init(int bytesize, uint8_t *memory);

public:
    // 剩余的大小
    int remainedSize(void)
    {
        return _size - _occupy;
    }

    // 使用了的大小
    int occupiedSize(void)
    {
        return _occupy;
    }

    // 总大小
    int totalSize(void)
    {
        return _size;
    }

    // 删除len长度的数据，和peek一起用，可以构成非阻塞式的read, 返回-1代表操作失败
    int pop(int len);
    // 读但不改变FIFO状态, 返回1，-1代表操作失败
    int peek(uint8_t *data, int len);
    // 读, 返回=实际操作长度，-1代表操作失败
    int read(uint8_t *data, int len);
    // 写, 返回=实际操作长度，-1代表操作失败
    int write(uint8_t *data, int len);
    // 写, 返回1，-1代表操作失败
    int write(uint8_t data);
    // 重置, 返回1，-1代表操作失败
    int reset();    

private:
    uint8_t *_buffer = NULL; // 此内存需要外部定义，这里只是一个指针
    int _readPtr;
    int _writePtr;
    int _size;
    int _occupy;

private:
    int _lock;
};

#endif