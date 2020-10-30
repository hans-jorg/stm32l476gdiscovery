
/**
 * @file     system_stm32l476.c
 * @brief    utilities code according CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Includes standard SystemInit
 * @note     Includes non standard SystemCoreClockSet
 * @note
 * @note     Calls SystemInit
 * @note     Calls _main (It provides one, but it is automatically redefined)
 * @note     Calls main
 * @note     This code must be adapted for processor and compiler
 *
 ******************************************************************************/

/**
 * @note
 *
 * To set clock frequency see:
 * DM00083560 RM0351  Reference Manual for STM32L4x6 advanced ARM-based
 *                    32-bit MCUs
 * DM00108832 DS10198 Datasheet for Ultra-low-power ARM ® Cortex-M4 32-bit
 *                    MCU+FPU, 100DMIPS, up to 1MB Flash, 128 KB SRAM,
 *                    USB OTG FS, LCD, analog, audio
 *
 * Clock source for SYSCLK
 *     HSE     4 - 48 MHz  Voltage scaling Range 1
 *             4 - 26 MHz  Voltage scaling Range 2
 *     HSI         16 MHz
 *     MSI   0.1 - 48 MHz  Voltage scaling Range 1
 *           0.1 - 26 MHz  Voltage scaling Range 2
 *     PLL   8 - 80 MHz Voltage scaling Range 1
 *           8 - 26 MHz Voltage scaling Range 2
 *
 * After reset: MSI with 4 MHz
 *
 * Crystal frequencies or external clock frequencies can be defined outside
 * this module by using compilation parameters. If not, a default is used
 *
 * Valid range for HSE frequency: 4 to 48 MHz Voltage scaling Range 1
 *                                4 to 26 MHz Voltage scaling Range 2
 * Valid range for HSE crystal:   4 to 48 MHz Voltage scaling Range 1
 *                                4 to 26 MHz Voltage scaling Range 2
 * Valid range for LSE frequency: 0 Hz to 1 MHz
 * Valid range for LSE crystal:   32768 Hz
 *
 * Symbol                  Default value
 * HSE_CRYSTAL_FREQ          8 MHz
 * HSE_FREQ                  HSE_CRYSTAL_FREQ
 * LSE_CRYSTAL_FREQ          32.768 Khz
 * LSE_FREQ                  LSE_CRYSTAL_FREQ
 *
 *
 * ATTENTION!!!!!!
 * There are dependencies between power supply, maximum frequency
 *   and flash latency
 *
 * From RM0351 5.1.6 (Voltage regulator)
 *      when VOS = 01, the maximum value of f_HCLK = 80 MHz  (1.2 V)
 *      when VOS = 10, the maximum value of f_HCLK = 26 MHz  (1.0 V)
 *
 *
 * From RM0351 3.3.3 (Read access latency) and Table 8 (Number of
 *      wait states according to CPU clock (HCLK) frequency)
 *
 *      Latencies for Flash memory in STM32FL476XX
 *              Voltage scaling                  Voltage scaling
 *             Range 1 (High Power)             Range 2 (Low Power)
 *   0 WS            <= 16                           <= 6
 *   1 WS            <= 32                           <= 12
 *   2 WS            <= 48                           <= 18
 *   3 WS            <= 64                           <= 26
 *   4 WS            <= 80                           <= 26
 *
 * MSI Configuration
 *   Frequencies:  0.1, 0.2, 0.4, 0.8, 1, 2, 4 (default), 8, 16, 24, 32, 48 MHz
 *   Can autocalibrate using LSE
 *
 * PLL Configuration
 *IN
 *               HSE or HSI or MSI
 *  PLL input = -------------------  (must be in range 4..16 MHz)
 *                      M            ( 1 <= M <= 8 coded as PLLM 0..7 )
 *
 *  VCO output = PLL input * N       (must be in range 64..344 MHz Range 1
 *                                  M                or 64..128 MHz Range 2)
 *                                   ( 8 <= N<= 86 )
 *
 *              VCO output
 *  SysClock = ------------          (must be in range  8..80 MHz Range 1
 *                 R                                or  8..26 MHz Range 2)
 *                                   (R=2, 4, 6, 8 coded as 0, 1, 2, 3)
 *
 *                SysClock
 *  HCLK =  -----------------
 *            AHB PrescalerDivisor
 *
 * AHB Prescaler Divisor  : 1   2   4   8  16  64 128 256 512  ( No 32!)
 * HPRE Prescaler Field   : 0   8   9  10  11  12  13  14  15
 *
 *  To generate 80 MHz SysClock from 4 MHz MSI clock calibrated by LSE
 *    Set voltage scaling to 01 (High Power)
 *    Set MSI to 4 MHz
 *    Enable LSE (you will need to unblock LSE configuration)
 *    Enable autocalibration MSIPLL
 *    Set PLLM =  0 (M   =  1 => PLLIN  =   4 MHz)
 *        PLLN = 40 (N   = 40 => PLLOUT = 160 MHz)
 *        PLLR =  0 (R   =  2 => SYSCLK =  80 MHz)
 *        HPRE =  0 (PRE =  1 => HCLK   =  80 MHz)
 *
 */


#include "stm32l476xx.h"
#include "system_stm32l476.h"


/* Timeout for clock set */
#if !defined  (CLOCK_TIMEOUT)
  #define CLOCK_TIMEOUT    0x05000
#endif /* CLOCK_TIMEOUT */

/**
 * @brief PLL Parameters
 *
 * @note
 * PLL_VCO = (HSE_FREQ or HSI_FREQ or MSI_FREQ / PLL_M) * PLL_N
 * SYSCLK = PLL_VCO / PLL_R
 * PLL48M1CLK = PLL_VCO / PLLQ
 * PLLSAI3CLK = PLL_VCO / PLLP
 * HCLK       = SYSCLK / HPRE  (SystemCoreClock)
 */


/// Global variable holding System Clock Frequency (HCLK)
uint32_t SystemCoreClock = MSI_DEFAULT_FREQ;

/// Table for AHB Prescaler actually the log2 of divisor
const uint8_t  AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

/// Table for MSI Frequencies
static const uint32_t MSIFreqTable[12] = {
    100000,         /* 100 KHz */
    200000,         /* 200 KHz */
    400000,         /* 400 KHz */
    800000,         /* 800 KHz */
   1000000,         /*   1 MHz */
   2000000,         /*   2 MHz */
   4000000,         /*   4 MHz */
   8000000,         /*   8 MHz */
  16000000,         /*  16 MHz */
  24000000,         /*  24 MHz */
  32000000,         /*  32 MHz */
  48000000          /*  48 MHz */
};

/// (internal) Small delay routine
static void delay(volatile uint32_t cnt) {
    while( cnt-- ) { __NOP(); } // to avoid optimization
}

/**
 * @brief Enable LSE Clock
 *
 * @note To modify LSE, it is necessary to access the Backup Domain Control
 *       Register (BDCR)
 * @note The register is write protected and a specific protocol is needed
 *       to get access.
 *
 */
static uint32_t LSEClock_Enable(void) {
uint32_t counter;

    if( RCC->BDCR&RCC_BDCR_LSERDY ) // Already set?
        return 0;

    // Enable APB Clock
    APBPeripheralClockSet(0, 0 );
    delay(10);

    // Enable access to PWR interface
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    delay(10);

    PWR->CR1 |= PWR_CR1_DBP;        // unprotect access to BDCR

    counter = CLOCK_TIMEOUT;        // wait until unprotected
    while( counter && ((PWR->CR1&PWR_CR1_DBP) == 0) ) counter--;
    if( counter == 0 )
        return 1;

    RCC->BDCR |= RCC_BDCR_LSEON;    // enable LSE

    counter = CLOCK_TIMEOUT;        // wait until ready
    while( counter && ((RCC->BDCR & RCC_BDCR_LSERDY) == 0) ) counter--;
    if( counter == 0 )
        return 2;

    // Done
    PWR->CR1 &= ~(PWR_CR1_DBP); // protect it
    return 0;
}


/**
 * @brief SystemInit
 *
 * @note Resets to default configuration for clock and disables all interrupts
 *
 * @note Replaces the one (dummy) containded in start_DEVICE.c
 *
 */

void
SystemInit(void) {

    /* Configure FPU */
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    /* set CP10 and CP11 Full Access */
    //SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
    #endif

    /* Set RCC clock configuration to the default */
    /* Set MSION bit */
    RCC->CR |= RCC_CR_MSION;

    /* Reset CFGR register */
    RCC->CFGR = 0x00000000;

    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= (uint32_t)0xEAF6FFFF;

    /* Reset PLLCFGR register */
    RCC->PLLCFGR = 0x00001000;

    /* Reset HSEBYP bit */
    RCC->CR &= (uint32_t)0xFFFBFFFF;

    /* Disable all interrupts */
    RCC->CIER = 0x00000000;

    /* Enable LSE (32768 Hz *) and set PLL Mode for MSI */

    /* Set SystemCoreClock */
    SystemCoreClockUpdate();

}

/**
 * @brief SystemCoreClockUpdate
 *
 * @note Updates the SystemCoreClock variable using information contained in the
 *       Clock Register Values (RCC)
 *
 * @note This function must be called to update SystemCoreClock variable every time
 *       the clock configuration is modified.
 */

void
SystemCoreClockUpdate(void) {
uint32_t rcc_cr, rcc_csr;
uint32_t src, pllvco_freq, pllr, pllsource, pllm;
uint32_t msi_freq, base_freq, prescaler;

    rcc_cr = RCC->CR;
    rcc_csr = RCC->CSR;

    /* Find frequency of MSI */
    if( (rcc_cr & RCC_CR_MSIRGSEL) == 0 ) {
        msi_freq = (rcc_csr&RCC_CSR_MSISRANGE)>>8;
    } else {
        msi_freq = (rcc_cr&RCC_CR_MSIRANGE)>>4;
    }
    /* Replace index with frequency */
    msi_freq = MSIFreqTable[msi_freq];

    /* Get source */
    src = RCC->CFGR & RCC_CFGR_SWS;
    switch (src) {
    case 0x00:  /* MSI used as system clock source */
        SystemCoreClock = msi_freq;
        break;
    case 0x04:  /* HSI used as system clock source */
        SystemCoreClock = HSI_FREQ;
        break;
    case 0x08:  /* HSE used as system clock source */
        SystemCoreClock = HSE_FREQ;
        break;
    case 0x0C:  /* PLL used as system clock source */
        /*
         * BASE_FREQ = HSE_FREQ or HSI_FREQ or MSI_FREQ
         * PLL_VCO = (BASE_FREQ / PLL_M) * PLL_N
         * SYSCLK = PLL_VCO / PLL_R
         */
        pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
        pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM)>>4)+1;

        switch (pllsource ) {
        case 0x02:
            base_freq = HSI_FREQ/pllm;
            break;
        case 0x03:
            base_freq = HSE_FREQ/pllm;
            break;
        default:
            base_freq = msi_freq/pllm;
            break;
        }

      pllvco_freq = base_freq * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 8);
      pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR)>>25) + 1 )*2;
      SystemCoreClock = pllvco_freq/pllr;
      break;
    default:
      SystemCoreClock = msi_freq;
      break;
    }

  /* HCLK = SYSCLK/AHB_Prescaler */

  /* Get HCLK prescaler */
  prescaler = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK frequency */
  SystemCoreClock >>= prescaler;
}


/**
 * @brief Configure PLL
 *
 * @note  It user PLL_Configuration structure
 * @note  src can be HSI_CLOCKSRC, MSIxxxxx_CLOCKSRC or HSE_CLOCKSRC
 * @note  prefactor can be 0 to 9 meaning 1 to 512 divisor
 * @note  lat is the latency for FLASH access (See RM00351 Table 8)
 * @note  pllconfig specifies M, N, R, P, Q
 */

static uint32_t PLLConfig(uint32_t src, uint32_t prefactor, uint32_t lat, PLL_Configuration *pllconfig) {
uint32_t counter,base_freq,index;
uint32_t pllsrc = 0;

    /* if PLL in use, set to use momentarily HSI */
    if( (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL ) {

        base_freq = HSI_FREQ;
        /* Start HSI */
        RCC->CR |= RCC_CR_HSION;

        /* Wait till HSI is ready and if time out is reached, exit */
        counter = CLOCK_TIMEOUT;
        while( counter && ((RCC->CR & RCC_CR_HSIRDY) == 0)) counter--;
        if( !counter ) return 0; /* HSI did not start */

        /* Do not use prescaler */
        RCC->CFGR = (RCC->CFGR&~RCC_CFGR_HPRE);

        /* Set Flash Latency (Wait States) */
        /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

        /* Set clock source to HSI */
        RCC->CFGR = (RCC->CFGR&~(RCC_CFGR_SW))|RCC_CFGR_SW_HSI;

        /* Verify if HSI is used as system clock source */
        counter = CLOCK_TIMEOUT;
        while (counter&&((RCC->CFGR & RCC_CFGR_SWS)!=RCC_CFGR_SWS_HSI)) counter--;
        if( !counter ) return 0; /* Could not use HSI as System Clock */

    };

    /* Turn off PLL */
    RCC->CR &= ~(RCC_CR_PLLON);

    /* Activate clock source */
    switch(src) {
    case HSI_CLOCKSRC:

        base_freq = HSI_FREQ;

        RCC->CR |= RCC_CR_HSION;
        /* Wait till HSI is ready and if time out is reached, exit */
        counter = CLOCK_TIMEOUT;
        while( counter && ((RCC->CR & RCC_CR_HSIRDY) == 0)) counter--;
        if( !counter ) return 0; /* HSI did not start */

        pllsrc = 2;
        break;

    case HSE_CLOCKSRC:

        base_freq = HSE_FREQ;

        RCC->CR |= RCC_CR_HSEON;
        /* Wait till HSE is ready and if time out is reached, exit */
        counter = CLOCK_TIMEOUT;
        while( counter && ((RCC->CR & RCC_CR_HSERDY) == 0)) counter--;
        if( !counter ) return 0; /* HSE did not start */

        pllsrc = 3;
        break;

    case MSI4M_CLOCKSRC:
    case MSI8M_CLOCKSRC:
    case MSI16M_CLOCKSRC:
    case MSI24M_CLOCKSRC:
    case MSI32M_CLOCKSRC:
    case MSI48M_CLOCKSRC:

        index = src-MSI100K_CLOCKSRC;
        base_freq = MSIFreqTable[index];

        /* Start MSI */
        RCC->CR |= RCC_CR_MSION;
        /* Wait till MSI is ready and if time out is reached, exit */
        counter = CLOCK_TIMEOUT;
        while( counter && ((RCC->CR & RCC_CR_MSIRDY) == 0)) counter--;
        if( !counter ) return 0; /* MSI did not start */

        /* Try to use PLL Mode with LSE */
        if( ((RCC->BDCR)&RCC_BDCR_LSEON) == 0 ) {
            RCC->BDCR |= RCC_BDCR_LSEON;
            counter = CLOCK_TIMEOUT;
            while( counter && (RCC->BDCR&RCC_BDCR_LSERDY) == 0 ) counter--;
            if( counter ) {/* LSE working */
                RCC->CR |= RCC_CR_MSIPLLEN;
            }
        }

        /* Set MSI range */
        RCC->CR = (RCC->CR&RCC_CR_MSIRANGE) | (index<<4);

        counter = CLOCK_TIMEOUT;
        while( counter && ((RCC->CR & RCC_CR_MSIRDY) == 0)) counter--;
        if( !counter ) return 0; /* MSI did not start */

        pllsrc = 1;
    }

    /* Configure Main PLL */
    /* Configure the main PLL */
    RCC->PLLCFGR = RCC_PLLCFGR_PLLREN | (pllconfig->M << 4) | (pllconfig->N << 8)
                  | pllsrc | (pllconfig->R << 25) | (pllconfig->Q << 21);

    /* Set prescaler */
    RCC->CFGR = (RCC->CFGR&~RCC_CFGR_HPRE) | (prefactor<<4);
    base_freq /= (1<<prefactor);

    /* Set Flash Latency (Wait States) */
    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | lat;

    /* Turn on PLL */
    RCC->CR |= RCC_CR_PLLON;    /* Configure PLL */

     /* Verify if PLL is ready */
    counter = CLOCK_TIMEOUT;
    while (counter&&((RCC->CFGR & RCC_CR_PLLRDY) == 0) ) counter--;
    if( !counter ) return 0; /* Could not use PLL as System Clock */

    /* Set clock source to PLL */
    RCC->CFGR = (RCC->CFGR&~(RCC_CFGR_SW))|RCC_CFGR_SW_PLL;

    /* Verify if PLL is used as system clock source */
    counter = CLOCK_TIMEOUT;
    while (counter&&((RCC->CFGR & RCC_CFGR_SWS)!=RCC_CFGR_SWS_PLL)) counter--;
    if( !counter ) return 0; /* Could not use PLL as System Clock */

    return base_freq;

}

/**
 * @brief Look for codification of prescaler divisor in AHBPrescTable
 *
 * @note Returns the codification for valid divisors (1,2,4,8,16,64,128,256,512)
 * @note There is NO 32 in the above list !!!
 * @note For divisors not in the above list returns the codification of the
 *       nearest highest divisor
 */

static uint32_t getAHBPrescalerEncoding(uint32_t prediv) {
int i;

    if( prediv <= 1 )
        return 0;

    for(i=0;i<sizeof(AHBPrescTable);i++) {
        if( AHBPrescTable[i]==0)
            continue;
        if( prediv == (1<<AHBPrescTable[i]) )
            return i;
        if( prediv < (1<<AHBPrescTable[i]) )
            break;
    }
    if( i==sizeof(AHBPrescTable) )
        return sizeof(AHBPrescTable)-1;

    return i;
}


/**
 * @brief Set System Core Clock
 *
 * @note  It uses PLL_Configuration structure
 * @note  src can be HSI16_CLOCKSRC, MSIxxxxx_CLOCKSRC or HSE_CLOCKSRC
 *        and to use PLL, HSI16PLL_CLOCKSRC, MSIxxxxPLL_CLOCKSRC,
 *        HSEPLL_CLOCKSRC
 * @note  prediv can be 0 to 9 meaning 1 to 512 divisor
 * @note  lat is the latency for FLASH access (See RM00351 Table 8)

 *
 *
 *
 * Collateral Effects
 *   Always set Voltage Scale to High Performance (Mode 1)
 *
 *
 * Flash wait states
 *
 *             Range 1 (High Power)             Range 2 (Low Power)
 *   0 WS            <= 16                           <= 6
 *   1 WS            <= 32                           <= 12
 *   2 WS            <= 48                           <= 18
 *   3 WS            <= 64                           <= 26
 *   4 WS            <= 80                           <= 26
 */
uint32_t
SystemCoreClockSet(uint32_t src, uint32_t prediv, uint32_t lat, PLL_Configuration *pllconfig) {
uint32_t base_freq,index,counter,prefactor;

    prefactor = getAHBPrescalerEncoding(prediv);

    /* Select regulator voltage output Scale 1 mode */
    // RCC->APB1ENR |= RCC_APB1ENR_PWREN; /* To enable Power Interface Clock */

    /* Set Voltage Scaling to High Performance */
    /* Voltage Scaling as Low Power can be used only under 26 MHZ */
    if( (PWR->CR1&PWR_CR1_VOS) != 1 ) {
        PWR->CR1 = (PWR->CR1&PWR_CR1_VOS)|1;             /* To enable 80 MHz */
        counter = CLOCK_TIMEOUT;
        while( counter && (PWR->SR2&PWR_SR2_VOSF) ) counter--;
        if( !counter ) return 1;
    }

    switch(src) {
    case HSI16_CLOCKSRC:        /* HSI as clock source */

        /* Set base frequency */
        base_freq = HSI_FREQ;

        /* Start HSI */
        RCC->CR |= RCC_CR_HSION;
        /* Wait till HSI is ready and if time out is reached, exit */
        counter = CLOCK_TIMEOUT;
        while( counter && ((RCC->CR & RCC_CR_HSIRDY) == 0)) counter--;
        if( !counter ) return 1; /* HSI did not start */

        /* Set prescaler */
        RCC->CFGR = (RCC->CFGR&~RCC_CFGR_HPRE) | (prefactor<<4);
        base_freq /= (1<<prefactor);

        /* Set Flash Latency (Wait States) */
        /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | lat;

        /* Set clock source to HSI */
        RCC->CFGR = (RCC->CFGR&~(RCC_CFGR_SW))|RCC_CFGR_SW_HSI;

        /* Verify if HSI is used as system clock source */
        counter = CLOCK_TIMEOUT;
        while (counter&&((RCC->CFGR & RCC_CFGR_SWS)!=RCC_CFGR_SWS_HSI)) counter--;
        if( !counter ) return 1; /* Could not use HSI as System Clock */

        base_freq = HSI_FREQ;
        break;

    case HSE_CLOCKSRC:          /* HSE as clock source */

        /* Set base frequency */
        base_freq = HSE_FREQ;

        /* Start HSE */
        RCC->CR |= RCC_CR_HSEON;
        /* Wait till HSE is ready and if time out is reached, exit */
        counter = CLOCK_TIMEOUT;
        while( counter && ((RCC->CR & RCC_CR_HSERDY) == 0)) counter--;
        if( !counter ) return 1; /* HSE did not start */

        /* Set prescaler */
        RCC->CFGR = (RCC->CFGR&~RCC_CFGR_HPRE) | (prefactor<<4);
        base_freq /= (1<<prefactor);

        /* Set Flash Latency (Wait States) */
        /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | lat;

        /* Set clock source to HSE */
        RCC->CFGR = (RCC->CFGR&~(RCC_CFGR_SW))|RCC_CFGR_SW_HSE;

        /* Verify if HSE is used as system clock source */
        counter = CLOCK_TIMEOUT;
        while (counter&&((RCC->CFGR & RCC_CFGR_SWS)!=RCC_CFGR_SWS_HSE)) counter--;
        if( !counter ) return 1; /* Could not use HSE as System Clock */

        base_freq = HSE_FREQ;
        break;

    case MSI100K_CLOCKSRC:      /* MSI as clock source */
    case MSI200K_CLOCKSRC:
    case MSI400K_CLOCKSRC:
    case MSI800K_CLOCKSRC:
    case MSI1M_CLOCKSRC:
    case MSI2M_CLOCKSRC:
    case MSI4M_CLOCKSRC:
    case MSI8M_CLOCKSRC:
    case MSI16M_CLOCKSRC:
    case MSI24M_CLOCKSRC:
    case MSI32M_CLOCKSRC:
    case MSI48M_CLOCKSRC:

        index = src-MSI100K_CLOCKSRC;
        base_freq = MSIFreqTable[index];

        /* Set prescaler */
        RCC->CFGR = (RCC->CFGR&~RCC_CFGR_HPRE) | (prefactor<<4);
        base_freq /= (1<<prefactor);

        /* Set Flash Latency (Wait States) */
        /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | lat;

        /* Set MSI range and configure MCU to use MSIRANGE */
        RCC->CR = (RCC->CR&~RCC_CR_MSIRANGE) | RCC_CR_MSIRGSEL | (index<<4);

        /* Try to use PLL Mode with LSE */
        if( ((RCC->BDCR)&RCC_BDCR_LSEON) == 0 ) {
            LSEClock_Enable();
        }
        // Enable autocalibration of MSI using LSE clock signal
        RCC->CR |= RCC_CR_MSIPLLEN;

        /* Start MSI */
        RCC->CR |= RCC_CR_MSION;
        /* Wait till MSI is ready and if time out is reached, exit */
        counter = CLOCK_TIMEOUT;
        while( counter && ((RCC->CR & RCC_CR_MSIRDY) == 0)) counter--;
        if( !counter ) return 1; /* MSI did not start */

        /* Set clock source to MSI */
        RCC->CFGR = (RCC->CFGR&~RCC_CFGR_SW)|RCC_CFGR_SW_MSI;

        /* Verify if MSI is used as system clock source */
        counter = CLOCK_TIMEOUT;
        while (counter&&((RCC->CFGR & RCC_CFGR_SWS)!=RCC_CFGR_SWS_MSI)) counter--;
        if( !counter ) return 1; /* Could not use MSI as System Clock */

        break;

    case HSI16PLL_CLOCKSRC:     /* PLL based on HSI */

        base_freq = PLLConfig(HSI_CLOCKSRC,prefactor,lat,pllconfig);
        break;

    case HSEPLL_CLOCKSRC:       /* PLL based on HSE */

        base_freq = PLLConfig(HSE_CLOCKSRC,prefactor,lat,pllconfig);
        break;

    case MSI4MPLL_CLOCKSRC:     /* PLL based on MSI */
    case MSI8MPLL_CLOCKSRC:
    case MSI16MPLL_CLOCKSRC:
    case MSI24MPLL_CLOCKSRC:
    case MSI32MPLL_CLOCKSRC:
    case MSI48MPLL_CLOCKSRC:
        /* Reindex to find clock source code */
        index = src-MSI4MPLL_CLOCKSRC+ MSI4M_CLOCKSRC;
        base_freq = PLLConfig(index,prefactor,lat,pllconfig);
        break;

    default:                    /* Error */
        return 1;

    };

    SystemCoreClock = base_freq;

    SystemCoreClockUpdate();

    return 0;
}


/**
 * @brief Find encoding for AHB Prescaler
 */
static uint32_t
getAPBPrescalerEncoding(uint32_t v) {

    if( v <= 1 ) {
        return 0;
    } else {
        uint32_t m = 2;
        uint32_t n = 0;
        while( n < 4 ) {
            if( m >= v )
                break;
            m <<= 1;
            n++;
        }
        if( n >= 4 ) n=3;
        return n+4;
    }

}

/**
 * @brief Set APB Peripheral Clock
 *
 * @note Set  clocks for APB1/AHB e AHB2/AHB Bridges
 *       APB2 : High speed AHB Clock (must be less than 90 MHz)
 *       APB1 : Low speed AHB Clock (muste be less than 45 MHz)
 */

uint32_t
APBPeripheralClockSet(uint32_t div1, uint32_t div2) {
uint32_t div1factor,div2factor;

    div1factor = getAPBPrescalerEncoding(div1);
    div2factor = getAPBPrescalerEncoding(div2);

    /* Set PCLK2 */
    RCC->CFGR = (RCC->CFGR&~RCC_CFGR_PPRE2)|(div2factor<<11);

    /* Set PCLK1 */
    RCC->CFGR = (RCC->CFGR&~RCC_CFGR_PPRE1)|(div1factor<<8);

    return 0;
}
