

/**
 * @file     watchdog.c
 * @brief    Hardware Abstraction Layer (HAL) for watchdoges
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 * @note     IWDG : clock 29.5-34 T=-40-125 Does not use Window
 *
 *
 ******************************************************************************/



#include <stdint.h>
#include "watchdog.h"
#include "stm32l476xx.h"

#define BIT(N) (1L<<(N))

#define IWDG_DEVICE_ENABLE_WORD 0x0000CCCC
#define IWDG_ACCESS_ENABLE_WORD 0x00005555
#define IWDG_REFRESH_WORD       0x0000AAAA

static void xdelay(volatile uint32_t v) {

    while( v-- ) {}

}


/**
 * @brief Watchdog Initialization
 *
 * @note Initializes specified LEDs
 *
 * When the window option it is not used, the IWDG can be configured as follows:
 * 1. Enable the IWDG by writing 0x0000 CCCC in the IWDG_KR register.
 * 2. Enable register access by writing 0x0000 5555 in the IWDG_KR register.
 * 3. Write the IWDG prescaler by programming IWDG_PR from 0 to 7.
 * 4. Write the reload register (IWDG_RLR).
 * 5. Wait for the registers to be updated (IWDG_SR = 0x0000 0000).
 * 6. Refresh the counter value with IWDG_RLR (IWDG_KR = 0x0000 AAAA)
 */
void watchdog_init(uint32_t pre, uint32_t v) {
#if WATCHDOG_DEVICE == IWDG_DEVICE
    IWDG->KR = IWDG_DEVICE_ENABLE_WORD;
    xdelay(10);
    IWDG->KR = IWDG_ACCESS_ENABLE_WORD;
    xdelay(10);

    IWDG->PR = pre&0x3;
    IWDG->RLR = v&0xFFF;
    while( IWDG->SR&0x2 ) {}

    IWDG->KR = IWDG_REFRESH_WORD;
#endif

#if WATCHDOG_DEVICE == WWDG_DEVICE
    WWDG->RL = BIT(6)|0x7F;
    WWDG->CR |= BIT(7)
#endif
}



void watchdog_feed(void) {

#if WATCHDOG_DEVICE == IWDG_DEVICE
    IWDG->KR = IWDG_REFRESH_WORD;
#endif
#if WATCHDOG_DEVICE == WWDG_DEVICE
    WWDG->RL = BIT(6)|0x7F;
#endif
}
