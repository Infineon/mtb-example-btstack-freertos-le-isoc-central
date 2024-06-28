#ifndef  APP_TERMINAL_TRACE_H
#define APP_TERMINAL_TRACE_H

#include <stdio.h>
#include "stdarg.h"

#ifndef ENABLE_BT_SPY_LOG

#ifdef WICED_BT_TRACE
#undef WICED_BT_TRACE
#endif

#define WICED_BT_TRACE(fmt,...)    printf(fmt "\n", ## __VA_ARGS__);

#endif

#endif
