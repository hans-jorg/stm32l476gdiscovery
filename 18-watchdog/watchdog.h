#ifndef WATCHDOG_H
#define WATCHDOG_H
/**
 * @file     watchdog.h
 * @brief    Interface to Watchdog
 * @version  V1.0
 * @date     09/11/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 * @note     Can be configured to use IWDG with 30 KHz clock or
 *           WWDG with system clock
 *
 *
 ******************************************************************************/

#define WWDG_DEVICE 1
#define IWDG_DEVICE 2

//#define WATCHDOG_DEVICE IWDG_DEVICE
//#define WATCHDOG_DEVICE WWDG_DEVICE


// Default watchdog
#ifndef WATCHDOG_DEVICE
#define WATCHDOG_DEVICE IWDG_DEVICE
#endif

#define WATCHDOG_RESETTED  ((RCC->CSR)&0x60000000)

#if WATCHDOG_DEVICE == WWDG_DEVICE
#define WATCHDOG_PRE_1   0x0
#define WATCHDOG_PRE_2   0x1
#define WATCHDOG_PRE_4   0x2
#define WATCHDOG_PRE_8   0x3
#endif


#if WATCHDOG_DEVICE == IWDG_DEVICE
#define WATCHDOG_PRE_4   0x0
#define WATCHDOG_PRE_8   0x1
#define WATCHDOG_PRE_16  0x2
#define WATCHDOG_PRE_32  0x3
#define WATCHDOG_PRE_64  0x4
#define WATCHDOG_PRE_128 0x5
#define WATCHDOG_PRE_256 0x6
#endif

void watchdog_init(uint32_t pre, uint32_t v);

void watchdog_feed(void);

#endif // WATCHDOG
