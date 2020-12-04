#ifndef SYSTEM_STM32L476_H
#define SYSTEM_STM32L476_H

/**
 * @file     system_stm32l476.h
 * @brief    utilities code according CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Provides CMSIS standard SystemInit and SystemCoreClockUpdate
 * @note     Provides non standard SystemCoreClockSet
 * @note     Define symbols for Clock Sources
 * @note     This code must be adapted for processor and compiler
 *
 ******************************************************************************/

/* CMSIS */
extern uint32_t SystemCoreClock;
void SystemInit(void);
void SystemCoreClockUpdate(void);

/**
 * @brief Clock frequencies
 */
//{
#ifndef HSE_CRYSTAL_FREQ
#define HSE_CRYSTAL_FREQ 8000000UL
#endif
#ifndef HSE_FREQ
#define HSE_FREQ  HSE_CRYSTAL_FREQ /* External crystal */
#endif

#ifndef LSE_CRYSTAL_FREQ
#define LSE_CRYSTAL_FREQ 32768UL
#endif
#ifndef LSE_FREQ
#define LSE_FREQ  LSE_CRYSTAL_FREQ /* External crystal */
#endif
//}

/**
 * @brief Frequency Parameters
 */
//{
#define HSI_FREQ        16000000UL /* Internal RC low precision (1%) */
#define MSI_DEFAULT_FREQ 4000000UL
#define LSI_FREQ           32000UL /* Internal RCC low precision [17..47 KHz]) */
//}


/**
 * @brief Codification of clock source
 */
//{
#define HSI16_CLOCKSRC       0x0
#define HSI_CLOCKSRC         HSI16_CLOCKSRC
#define HSE_CLOCKSRC         0x1
/* PLL based on HSI or HSE */
#define HSI16PLL_CLOCKSRC    0x2
#define HSEPLL_CLOCKSRC      0x3
#define HSIPLL_CLOCKSRC      HSI16PLL_CLOCKSRC
/* Clock source from MSI. They must be in this order */
#define MSI100K_CLOCKSRC     0x4
#define MSI200K_CLOCKSRC     0x5
#define MSI400K_CLOCKSRC     0x6
#define MSI800K_CLOCKSRC     0x7
#define MSI1M_CLOCKSRC       0x8
#define MSI2M_CLOCKSRC       0x9
#define MSI4M_CLOCKSRC       0xA
#define MSI8M_CLOCKSRC       0xB
#define MSI16M_CLOCKSRC      0xC
#define MSI24M_CLOCKSRC      0xD
#define MSI32M_CLOCKSRC      0xE
#define MSI48M_CLOCKSRC      0xF
/* PLL based on MSI. They must be in this order */
#define MSI4MPLL_CLOCKSRC    0x10
#define MSI8MPLL_CLOCKSRC    0x11
#define MSI16MPLL_CLOCKSRC   0x12
#define MSI24MPLL_CLOCKSRC   0x13
#define MSI32MPLL_CLOCKSRC   0x14
#define MSI48MPLL_CLOCKSRC   0x15


#define PLL_CLOCKSRC         MSI4M_PLL_CLOCKSRC
//}

/**
 * @brief PLL parameters
 */

typedef struct {
    uint32_t    M;
    uint32_t    N;
    uint32_t    P;              /* for PLLSAI3CLK */
    uint32_t    Q;              /* for PLL48M1CLK */
    uint32_t    R;              /* for SYSCLK     */
} PLL_Configuration;

/**
 * @brief Set System Core Clock
 *
 * @note
 * HSI_CLOCKSRC =>        HSI_FREQ/pre
 * HSE_CLOCKSRC =>        HSE_FREQ/pre
 * HSIPLL_CLOCKSRC =>    (HSI_FREQ/M)*N/R/pre
 * HSEPLL_CLOCKSRC =>    (HSE_FREQ/M)*N/R/pre
 * MSIxxxPLL_CLOCKSRC => (MSI_FREQ/M)*N/R/pre
 *
 * See RM00351 for constraints!!!!!
 * STM32L476VG6 maximum velocity = 80 MHz : MSI4M,M=2,N=80,P=2,pre=1
 *
 *
 * Collateral Effects
 *   Always set Voltage Scale Range to High Power (1)
 *
 *
 */
uint32_t
SystemCoreClockSet(uint32_t src, uint32_t pre, uint32_t lat, PLL_Configuration *pllconfig);

/*
 * Set APB Peripheral Clock
 * Set  clocks for APB1/AHB e AHB2/AHB Bridges
 * Derived from System Clock
 * APB2 : High speed AHB Clock (must be less than 90 MHz)
 * APB1 : Low speed AHB Clock (must be less than 45 MHz)
 */
uint32_t APBPeripheralClockSet(uint32_t div1, uint32_t div2);

#endif
