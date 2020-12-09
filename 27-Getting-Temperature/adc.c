
/*
 *  ADC HAL routines
 *
 * The STM32L476 has three ADC (ADC1, ADC2, ADC3)
 *
 * There are 24 external channels. some of them can be reached by ADCx exclusively,
 *   ADC1 or ADC2, ADC1 or ADC2 or ADC3.
 *
 * There are five internal channels: internal reference voltage, temperature sensor,
 *   VBAT/3, DAC1 and DAC2 outputs.
 *
 * The channel can be single ended or differential
 *
 * The temperature channel TS is on the channel 17 of ADC1 or ADC3. There are calibration
 *  factors at memory address 0x1FFF_75A8-0x1FFF_75A9 and 0x1FFF_75CA-0x1FFF_75CB.
 *
 * The internal reference V_REFINT is connected to the channel 0 of ADC1. There is a
 *   calibration factor at the memory address 0x1FFF_75AA-0x1FFF_75AB.
 *
 * The battery voltage (VBAT) is available on the channel 18 of ADC1 or ADC3. To
 *   keep the value in the range it is divided by 3.
 *
 */

/* Pin map
 *
 *  ADC123_IN1  PC0
 *  ADC123_IN2  PC1
 *  ADC123_IN3  PC2
 *  ADC123_IN4  PC3
 *  ADC12_IN5   PA0
 *  ADC12_IN6   PA1
 *  ADC12_IN7   PA2
 *  ADC12_IN8   PA3
 *  ADC12_IN9   PA4
 *  ADC12_IN10  PA5
 *  ADC12_IN11  PA6
 *  ADC12_IN12  PA7
 *  ADC12_IN13  PC4
 *  ADC12_IN14  PC5
 *  ADC12_IN15  PB0
 *  ADC12_IN16  PB1
 *
 * Not in LQFP100 version used in the Discovery board
 * ADC3_IN6     PF3
 * ADC3_IN7     PF4
 * ADC3_IN8     PF5
 * ADC3_IN9     PF6
 * ADC3_IN10    PF7
 * ADC3_IN11    PF8
 * ADC3_IN12    PF9
 * ADC3_IN13    PF10
*/

#include <stdint.h>
#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "adc.h"
#include "gpio2.h"
/*
 * Analog-Digital Interface Routines
 *
 * Only regular conversions
 *
 *
 */

#define BIT(N)                      (1UL<<(N))
#define BITMASK(M,N)                ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)               ((V)<<(N))
#define BITCLEAR(VAR,M)             ((VAR) &=~(M))
#define BITSET(VAR,M)               ((VAR) |=(M))
#define BITSETN(VAR,N)              (VAR)  |= (BIT(N))
#define BITCLEARN(VAR,N)            (VAR)  &= ~(BIT(N))
#define BITFIELDSET(VAR,MASK,VAL)   (VAR)  = ((VAR)&~(MASK))|(VAL)
#define BITFIELDCLEAR(VAR,MASK,VAL) (VAR)  = ((VAR)&~(MASK))


#include <stm32l4xx.h>
#include <stdio.h>
#include "gpio2.h"
#include "adc.h"


// See 3.15.1 in DM00108832
#define TS_CAL1 ( uint32_t) (* (uint16_t *) 0x1FFF75A8)
#define TS_CAL2 ( uint32_t) (* (uint16_t *) 0x1FFF75CA)
#define VREFINT_CAL ( uint32_t) (* (uint16_t *) 0x1FFF75AA)



static void udelay( uint32_t volatile cnt ) {

    while( cnt-- ) { __NOP(); }
}

/*
 * Information about pins used for ADC inputs
 *
 * For internal channels, gpiopin code the type of internal sensor/voltage
 *
 *    1:    Temperature sensor
 *    2:    V_REFINT
 *    3:    V_BAT
 *    4:    DAC1
 *    5:    DAC2
 */
typedef struct {
    uint32_t        adcchannel;         // ADC channel
    ADC_TypeDef     *adcport;               // ADC port
    GPIO_TypeDef    *gpioport;          // port in charge of pin
    uint16_t        gpiopin;            // pin at port
    uint16_t        index;              // index of distinct channels
} pinmap_t;


static const pinmap_t pinmap[] = {
/* V_REFINT can be reached thru ADC1 only */
    {   0,   ADC1,    0,         2,   0},
/* Channel 1 shares pin with ADC1, ADC2 and ADC 3 */
    {   1,   ADC1,    GPIOC,     0,   1},
    {   1,   ADC2,    GPIOC,     0,   1},
    {   1,   ADC3,    GPIOC,     0,   1},
/* Channel 2 shares pin with ADC1, ADC2 and ADC 3 */
    {   2,   ADC1,    GPIOC,     1,   2},
    {   2,   ADC2,    GPIOC,     1,   2},
    {   2,   ADC3,    GPIOC,     1,   2},
/* Channel 3 shares pin with ADC1, ADC2 and ADC 3 */
    {   3,   ADC1,    GPIOC,     2,   3},
    {   3,   ADC2,    GPIOC,     2,   3},
    {   3,   ADC3,    GPIOC,     2,   3},
/* Channel 4 shares pin with ADC1, ADC2 and ADC 3 */
    {   4,   ADC1,    GPIOC,     3,   4},
    {   4,   ADC2,    GPIOC,     3,   4},
    {   4,   ADC3,    GPIOC,     3,   4},
/* Channel 5 shares pin with ADC1 and ADC2  */
    {   5,   ADC1,    GPIOA,     0,   5},
    {   5,   ADC2,    GPIOA,     0,   5},
/* Channel 6 shares pin with ADC1 and ADC2, ADC3 has a separated pin  */
    {   6,   ADC1,    GPIOA,     1,   6},
    {   6,   ADC2,    GPIOA,     1,   6},
    {   6,   ADC3,    GPIOF,     3,   7},
/* Channel 6 shares pin with ADC1 and ADC2, ADC3 has a separated pin  */
    {   7,   ADC1,    GPIOA,     2,   8},
    {   7,   ADC2,    GPIOA,     2,   8},
    {   7,   ADC3,    GPIOF,     4,   9},
/* Channel 6 shares pin with ADC1 and ADC2, ADC3 has a separated pin  */
    {   8,   ADC1,    GPIOA,     3,  10},
    {   8,   ADC2,    GPIOA,     3,  10},
    {   8,   ADC3,    GPIOF,     5,  11},
/* Channel 6 shares pin with ADC1 and ADC2, ADC3 has a separated pin  */
    {   9,   ADC1,    GPIOA,     4,  12},
    {   9,   ADC2,    GPIOA,     4,  12},
    {   9,   ADC3,    GPIOF,     6,  13},
/* Channel 6 shares pin with ADC1 and ADC2, ADC3 has a separated pin  */
    {  10,   ADC1,    GPIOA,     5,  14},
    {  10,   ADC2,    GPIOA,     5,  14},
    {  10,   ADC3,    GPIOF,     7,  15},
/* Channel 6 shares pin with ADC1 and ADC2, ADC3 has a separated pin  */
    {  11,   ADC1,    GPIOA,     6,  16},
    {  11,   ADC2,    GPIOA,     6,  16},
    {  11,   ADC3,    GPIOF,     8,  17},
/* Channel 6 shares pin with ADC1 and ADC2, ADC3 has a separated pin  */
    {  12,   ADC1,    GPIOA,     7,  18},
    {  12,   ADC2,    GPIOA,     7,  18},
    {  12,   ADC3,    GPIOF,     9,  19},
/* Channel 6 shares pin with ADC1 and ADC2, ADC3 has a separeted pin */
    {  13,   ADC1,    GPIOC,     4,  20},
    {  13,   ADC2,    GPIOC,     4,  20},
    {  13,   ADC3,    GPIOF,    10,  21},
/* Channel 5 shares pin with ADC1 and ADC2, DAC1 thru ADC3 */
    {  14,   ADC1,    GPIOC,     5,  22},
    {  14,   ADC2,    GPIOC,     5,  22},
    {  14,   ADC2,    0,         4,  23},
/* Channel 5 shares pin with ADC1 and ADC2, DAC2 thru ADC3  */
    {  15,   ADC1,    GPIOB,     0,  24},
    {  15,   ADC2,    GPIOB,     0,  24},
    {  15,   ADC2,    0,         5,  25},
/* Channel 5 shares pin with ADC1 and ADC2  */
    {  16,   ADC1,    GPIOB,     1,  26},
    {  16,   ADC2,    GPIOB,     1,  27},
/* Temperature sensor can be accessed thru ADC1 or ADC3, DAC1 thru ADC2 */
    {  17,   ADC1,    0,         1,  28},
    {  17,   ADC2,    0,         4,  23},
    {  17,   ADC3,    0,         1,  28},
/* battery voltage can be read thru ADC1 or ADC3, DAC2 thru ADC2 */
    {  18,   ADC1,    0,         3,  29},
    {  18,   ADC2,    0,         5,  25},
    {  18,   ADC3,    0,         3,  29},

/* End of table */
    {   0,   0,       0,         0,   0}
};


static int PinMapSearch(ADC_TypeDef *adc, uint32_t channel) {
int k;

    for(k=0;k<sizeof(pinmap)/sizeof(pinmap_t);k++) {
        if( (channel == pinmap[k].adcchannel) && (adc == pinmap[k].adcport) ) {
            return k;
        }
    }
    return -1;
}

static uint32_t ADC_ConfigureChannel(ADC_TypeDef *adc, uint32_t ch, uint32_t config) {
uint32_t k;
GPIO_TypeDef *gpio;
int pin;
uint32_t samp;
int shift3;
int shift2;
int ind;
static uint32_t configured = 0;

    k = PinMapSearch(adc,ch);
    if( k < 0 )
        return -1;

    gpio = pinmap[k].gpioport;
    pin  = pinmap[k].gpiopin;
    ind  = pinmap[k].index;


    if( configured&(1U<<ind) )
        return -2;              // already configured

    if( gpio == NULL ) { // Internal channel
        if( ch == 17 ) {
            ADC123_COMMON->CCR |= ADC_CCR_TSEN;     // Enable Temp channel
        } else if( ch == 18 ) {
            ADC123_COMMON->CCR |= ADC_CCR_VBATEN;   // Enable VBAT channel
        } else if ( ch == 0 ) {
            ADC123_COMMON->CCR |= ADC_CCR_VREFEN;   // Enable VREF channel
        }
    } else {            // External channel
        shift2 = pin*2;
        // Enable clock for GPIO used
        GPIO_EnableClock(gpio);
        gpio->MODER = (gpio->MODER&~(3<<shift2)) | (3<<shift2);
        // Switch to analog
        gpio->ASCR |= BIT(pin);
    }
    samp = config&0xF;
    if( ch < 10 ) {
        shift3 = ch*3;
        adc->SMPR1 = (adc->SMPR1&~(7<<(shift3))) | (samp<<(shift3));
    } else {
        shift3 = (ch-10)*3;
        adc->SMPR2 = (adc->SMPR1&~(7<<(shift3))) | (samp<<(shift3));
    }
    configured |= (1U<<ind);
    if ( (config&ADC_MODE_DIFFERENTIAL)&&(ch<16)&&(ch>1)  ) {
        // Differential signals use next channel as negative input
        k = PinMapSearch(adc,ch+1);
        if( k < 0 )
            return -3;
        if( configured&(1U<<ind) )
            return -4;      // already configured
        gpio = pinmap[k].gpioport;
        pin  = pinmap[k].gpiopin;
        ind  = pinmap[k].index;
        adc->DIFSEL |= BIT(ch); /* differential inputs use next pin */
        GPIO_EnableClock(gpio);
        gpio->MODER = (gpio->MODER&~(3<<shift2)) | (3<<shift2);
        // Switch to analog
        gpio->ASCR |= BIT(pin);
        configured |= (1U<<ind);
    } else {
        adc->DIFSEL &= ~BIT(ch);
    }
    return 0;
}

int ADC_Init(ADC_TypeDef *adc) {
uint32_t t;
uint32_t timeout_cnt;

    // Enable SysConfig, Comp, etc.
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Enable Clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;


    // Disable (just in case)
    adc->CR &= ~(ADC_CR_ADEN);

    // Configure system clock as clock source for ADC
    RCC->CCIPR  =  (RCC->CCIPR&~RCC_CCIPR_ADCSEL_Msk) | (3<<RCC_CCIPR_ADCSEL_Pos);

    // Turn off DEEPPWD (default after reset))
    adc->CR     &= ~ADC_CR_DEEPPWD;

    // Turn on voltage regulator
    adc->CR     |= ADC_CR_ADVREGEN;

    // Enable the ADC clock, and GPIOA clk Depends on which ADC pins used
    RCC->AHB2ENR |= (RCC_AHB2ENR_ADCEN|RCC_AHB2ENR_GPIOAEN);

    /*
     * Boost enable for high VDDA
     *     0: I/O analog switches are supplied by V DDA voltage. This is the
     *        recommended configuration when using the ADC in high V DDA voltage
     *        operation.
     *     1: I/O analog switches are supplied by a dedicated voltage booster
     *        (supplied by V DD ). This is the recommended configuration when
     *        using the ADC in low V DDA voltage operation.
     */
    //SYSCFG->CFGR1 |= SYSCFG_CFGR1_BOOSTEN;

    int presc = 3;
    int ckmode = 0;

    if( ((RCC->CFGR&RCC_CFGR_HPRE)>>4) == 1 ) {
        presc = 1;
    } else {
        presc = 2;
    }

    ADC123_COMMON->CCR = (ADC123_COMMON->CCR
                                &~(ADC_CCR_PRESC_Msk|ADC_CCR_CKMODE_Msk))
                         |(presc<<ADC_CCR_PRESC_Pos)
                         |(ckmode<<ADC_CCR_CKMODE_Pos);

    // Calibration
 #if 0
    adc->CR |= ADC_CR_ADCAL;          // start calibration

    timeout_cnt = 4000;
    while( timeout_cnt && ((adc->CR&ADC_CR_ADCAL)==1) ) timeout_cnt--;
#endif

    adc->CR |= ADC_CR_ADEN;

    adc->ISR = ADC_ISR_ADRDY;

    timeout_cnt = 4000;
    while( timeout_cnt && ((adc->ISR&ADC_ISR_ADRDY)==0)) timeout_cnt--;
    if( timeout_cnt == 0 ) return -1;

    return 0;
}

int32_t ADC_Read(ADC_TypeDef *adc, uint32_t ch) {
uint32_t v=0;
uint32_t timeout_cnt;

    // Reset flags
    adc->ISR = ADC_ISR_EOC|ADC_ISR_OVR|ADC_ISR_JEOC|ADC_ISR_EOS|ADC_ISR_ADRDY;

    // Set Resolution
    adc->CFGR = (adc->CFGR&~(ADC_CFGR_RES_Msk))
                |(0<<ADC_CFGR_RES_Pos);

    // Specify channel
    adc->SQR1 = (ch<<6)|1; // only one channel is sequence

    // Single conversion and not hardware trigger
    adc->CFGR &= ~(ADC_CFGR_CONT|ADC_CFGR_EXTEN_Msk);

    //Clear the ADRDY bit in the ADCx_ISR register by writing ‘1’.
    ADC1->ISR    |= ADC_ISR_ADRDY;

    adc->CR = ADC_CR_ADSTART;


/*    timeout_cnt = 4000;*/
/*    while( timeout_cnt && ((adc->ISR&ADC_ISR_EOC)==0)) timeout_cnt--;*/
/*    if( timeout_cnt == 0 ) return -1;*/

    while( (adc->ISR&ADC_ISR_EOC)==0 ) {}
    v = adc->DR;

#if 0
    timeout_cnt = 4000;
    while( timeout_cnt && ((adc->ISR&ADC_ISR_EOS)==0)) timeout_cnt--;
    if( timeout_cnt == 0 ) return -2;
#endif
    return v;
}


int ADC_ReadMultiple(ADC_TypeDef *adc, uint32_t *ch, uint32_t *val, int n) {
uint32_t i,sqr1,sqr2,sqr3,sqr4;
uint32_t timeout_cnt;

    adc->CFGR &= ~ADC_CFGR_CONT;

    // Fill register with channels to be converted keeping the same order
    sqr1 = sqr2 = sqr3 = sqr4 = 0;

    for(i=0;i<n;i++) {
        if( i < 4 ) {                   /* From 0 to 3 : SQ1 to SQ4 */
            sqr1 |= (ch[i]<<((i)*6+6));
        } else if ( i < 10 ) {          /* From 4 to 8 : SQ5 to SQ9 */
            sqr2 |= (ch[i]<<((i-4)*6));
        } else if ( i < 15 ) {         /* From 9 to 13 : SQ10 to SQ14 */
            sqr3 |= (ch[i]<<((i-10)*6));
        } else  {                       /* From 14 to 15 : SQ15 to SQ16 */
            sqr4 |= (ch[i]<<((i-15)*6));
        }

    }
    adc->SQR1 = sqr1|n;         // Insert size of sequence length
    adc->SQR2 = sqr2;
    adc->SQR3 = sqr3;
    adc->SQR4 = sqr4;

    //adc->CFGR &= ~ADC_CFGR_EXTEN;

    // Start conversion
    adc->CR = ADC_CR_ADSTART;
    for(i=0;i<n;i++) {
        timeout_cnt = 4000;
        while( timeout_cnt && ((adc->ISR&ADC_ISR_EOC)==0)) timeout_cnt--;
        if( timeout_cnt == 0 ) return (uint32_t) -1;
        *val++ = adc->DR;
    }

    timeout_cnt = 4000;
    while( timeout_cnt && ((adc->ISR&ADC_ISR_EOS)==0)) timeout_cnt--;
    if( timeout_cnt == 0 ) return (uint32_t) -2;
    return 0;
}


/*
 * Read Temperature
 *
 *         110 - 30
 *  T = ----------------- (VMEASURED - TS_CAL1) + 30
 *        TS_CAL2-TS_CAL1
 *
 *
 */

int32_t
ADC_ReadTemperature(void) {
uint32_t v;

    v = ADC_Read(ADC1,ADC1_TEMP_CHANNEL);
    v = v - TS_CAL1;
    v = 90000/(TS_CAL2-TS_CAL1); // 3 decimal places
    v += 30000;

    return v;
}


/*
 * Read VBAT
 *
 *  VREF =  3*VMEASURED
 *
 * Result in mV
 */

int32_t
ADC_ReadVBat(void) {
uint32_t v;

    v = ADC_Read(ADC1,ADC1_VBAT_CHANNEL);
    v *= 3;
    v *= 3000;
    v >>= 12;

    return v;
}


/*
 * Read VREFINT
 *
 *               VREFINT
 *  VREF  = 3.0 -----------
 *               VMEASURED
 *
 * Result in mV
 */

int32_t
ADC_ReadVREF(void) {
uint32_t v;

    v = ADC_Read(ADC1,ADC1_VREF_CHANNEL);
    v = 3000*VREFINT_CAL/v;
    v >>= 12;

    return v;
}
