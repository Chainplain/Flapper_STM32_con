#ifndef _userString_h_
#define _userString_h_

#include <stddef.h>
#include <stdint.h>
#include "string.h"

class UserString
{
public:
    static int IndexOf(const char *src, const char *match);
    static int IndexOf(const char *src, const char match);
};

#endif