#include "userString.h"

// 查找匹配字符[串]
// 返回
// ---- 匹配字符串的索引号[第一个匹配字符]
// ---- -1 代表没有匹配
int UserString::IndexOf(const char *src, const char *match)
{
    int srcLen = strlen(src);
    int mLen = strlen(match);
    if (mLen > srcLen)
        return -1;

    int j;
    int itLen = srcLen - mLen + 1;
    char *psrc, *pmatch;

    for (int i = 0; i < itLen; i++)
    {
        if (mLen > (srcLen - i))
            return -1;

        psrc = (char *)src + i;
        pmatch = (char *)match;
        for (j = 0; j < mLen; j++)
        {
            //if (src[i + j] != match[j])
            if (*psrc++ == *pmatch++)
                continue;
            else
                break;
        }
        if (j == mLen)
            return i;
    }
    return -1;
}
int UserString::IndexOf(const char *src, const char match)
{
    int srcLen = strlen(src);
    if (srcLen < 1)
        return -1;

    char *psrc = (char *)src;

    for (int i = 0; i < srcLen; i++)
    {
        if (*psrc++ == match)
            return i;
    }
    return -1;
}
