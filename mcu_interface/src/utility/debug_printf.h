#pragma once

#undef _DBG
#undef _DBG_
#undef _DBC
#undef _DBD
#undef _DBD16
#undef _DBD32
#undef _DBH
#undef _DBH16
#undef _DBH32

#ifdef ENABLE_DEBUG_UART
#include <cstdio>
#define _DBG_PRINTF(...) printf(__VA_ARGS__)
#define _DBG(x)	 	printf("%s", x)
#define _DBG_(x)	printf("%s", x)
#define _DBC(x)	 	printf("%c", x)
#define _DBD(x)	 	printf("%d", x)
#define _DBD16(x)	printf("%d", x)
#define _DBD32(x)	printf("%ld", x)
#define _DBH(x)	  printf("0x%02X", x)
#define _DBH16(x)	printf("0x%04X", x)
#define _DBH32(x)	printf("0x%08X", x)
#else
#define _DBG_PRINTF(fmt_str, args...)
#define _DBG(x)
#define _DBG_(x)
#define _DBC(x)
#define _DBD(x)
#define _DBD16(x)
#define _DBD32(x)
#define _DBH(x)
#define _DBH16(x)
#define _DBH32(x)
#endif
