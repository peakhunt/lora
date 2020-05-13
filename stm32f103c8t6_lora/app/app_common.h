#ifndef __APP_COMMON_DEF_H__
#define __APP_COMMON_DEF_H__

#include <stdint.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"

#define bool      uint8_t
#define true      1
#define false     0

////////////////////////////////////////////////////////////////////////////////
//
// system uptime. defined in stm32f1xx_callback.c
//
////////////////////////////////////////////////////////////////////////////////
extern volatile uint32_t     __uptime;

////////////////////////////////////////////////////////////////////////////////
//
// misc utilities
//
////////////////////////////////////////////////////////////////////////////////
#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})

#define min(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b;       \
})

#endif //!__APP_COMMON_DEF_H__
