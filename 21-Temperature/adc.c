

#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "adc.h"
/*
 * Analog-Digital Interface Routines
 *
 * Only regular conversions
 *
 *
 */

#define BIT(N) (1UL<<(N))
#define BITMASK(M,N)  ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)  ((V)<<(N))
#define BITCLEAR(VAR,M)  ((VAR)&=~(M))
#define BITSET(VAR,M)    ((VAR)|=(M))
#define BITSETN(VAR,N) (VAR) |= (BIT(N))
#define BITCLEARN(VAR,N) (VAR) &= ~(BIT(N))
#define BITFIELDSET(VAR,MASK,VAL) (VAR) = ((VAR)&~(MASK))|(VAL)
#define BITFIELDCLEAR(VAR,MASK,VAL) (VAR) = ((VAR)&~(MASK))





static void delay( uint32_t cnt ) {

    while( cnt-- ) { __NOP(); }
}


typedef struct {
    uint32_t        adcchannel;
    uint32_t        adcs;
    GPIO_TypeDef *  gpio;
    uint32_t        gpiopin;
} pinmap_t;

#define SHADC1  (1)
#define SHADC2  (2)
#define SHADC3  (4)
#define SHADC12 (SHADC1|SHADC2)
#define SHADC123 (SHADC1|SHADC2|SHADC3)

static pinmap_t pinmap[] = {
    {   1,   SHADC123,    GPIOC,    0 },
    {   2,   SHADC123,    GPIOC,    1 },
    {   3,   SHADC123,    GPIOC,    2 },
    {   4,   SHADC123,    GPIOC,    3 },
    {   5,   SHADC12,     GPIOA,    0 },
    {   6,   SHADC12,     GPIOA,    1 },
    {   6,   SHADC3,      GPIOF,    3 },
    {   7,   SHADC12,     GPIOA,    2 },
    {   7,   SHADC3,      GPIOF,     4 },
    {   8,   SHADC12,     GPIOA,     3 },
    {   8,   SHADC3,      GPIOF,     5 },
    {   9,   SHADC12,     GPIOA,     4 },
    {   9,   SHADC3,      GPIOF,     6 },
    {  10,   SHADC12,     GPIOA,     5 },
    {  10,   SHADC3,      GPIOF,     7 },
    {  11,   SHADC12,     GPIOA,     6 },
    {  11,   SHADC3,      GPIOF,     8 },
    {  12,   SHADC12,     GPIOA,     7 },
    {  12,   SHADC3,      GPIOF,     9 },
    {  13,   SHADC12,     GPIOC,     4 },
    {  13,   SHADC3,      GPIOF,    10 },
    {  14,   SHADC12,     GPIOC,     5 },
    {  15,   SHADC12,     GPIOB,     0 },
    {  16,   SHADC12,     GPIOB,     1 }
};

static uint32_t lookforgpioinfo(ADC_TypeDef *adc, uint32_t channel, GPIO_TypeDef **pgpio, uint32_t *gpiopin) {
uint32_t k;
uint32_t a;
uint32_t found;

    if( adc == ADC1 )       a = SHADC1;
    else if ( adc == ADC2 ) a = SHADC2;
    else if ( adc == ADC3 ) a = SHADC3;
    else                    return 1; // ERROR

    found = 0;
    for(k=0;k<sizeof(pinmap)/sizeof(pinmap_t);k++) {
        if( (channel == pinmap[k].adcchannel) && (pinmap[k].adcs&a) ) {
            found = 1;
            break;
        }
    }
    if( found == 0 )
        return 2;

    *pgpio   = pinmap[k].gpio;
    *gpiopin = pinmap[k].gpiopin;
    return 0;
}

uint32_t ADC_Init(ADC_TypeDef *adc) {
uint32_t t;
uint32_t timeout_cnt;

    adc->CR &= ~(ADC_CR_ADEN); // disable (just in case)

    // Enable Clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    // Power the ADC
    adc->CR &= ~ADC_CR_DEEPPWD;
    adc->CR |= ADC_CR_ADVREGEN;
    delay(1000);

    // Use System Clock for ADC
    BITFIELDSET(RCC->CCIPR,RCC_CCIPR_ADCSEL,BITVALUE(3,28));

    // ?????
    //SYSCFG->CFGR1 |= SYSCFG_CFGR1_BOOSTEN; // Boost enable for high VDDA

    t = ADC123_COMMON->CCR;
    BITFIELDSET(t,ADC_CCR_CKMODE,BITVALUE(1,16)); // Set to highest frequency
#if 0
    // Only for injected ????
    if( (RCC->CFGR  & RCC_CFGR_HPRE)>>4) == 1 ) {
        presc = 1;
    } else {
        presc = 2;
    }
    BITFIELDSET(t,ADC_CCR_PRESC,BITVALUE(presc,16)); // Set to highest frequency
#endif
    ADC123_COMMON->CCR = t;



    adc->CR |= ADC_CR_ADCAL;          // start calibration

    timeout_cnt = 4000;
    while( timeout_cnt && ((adc->CR&ADC_CR_ADCAL)==1) ) timeout_cnt--;


    t = ADC123_COMMON->CCR;
    BITFIELDSET(t,ADC_CCR_PRESC, BITVALUE(1,18) );     // ADC Prescaler : ADCCLK/2
    BITFIELDSET(t,ADC_CCR_CKMODE, BITVALUE(2,16) );    // ADC Clock : HCLK/2
    t |= ADC_CCR_VBATEN;
    t |= ADC_CCR_TSEN;
    t |= ADC_CCR_VREFEN;
    ADC123_COMMON->CCR = t;

    adc->ISR |= ADC_ISR_ADRDY;
    adc->CR |= ADC_CR_ADEN;
    timeout_cnt = 4000;
    while( timeout_cnt && ((adc->ISR&ADC_ISR_ADRDY)==0)) timeout_cnt--;
    if( timeout_cnt == 0 ) return 1;
    adc->ISR &= ~ADC_ISR_ADRDY;

    return 0;
}

static uint32_t chused = 0;

uint32_t ADC_SetupPin(ADC_TypeDef *adc, uint32_t ch, uint32_t info) {
uint32_t samp=info&0x7;
uint32_t pin,chpos;
GPIO_TypeDef *gpio;

    if ( ((adc==ADC1)&&(ch==ADC1_TEMP_CHANNEL))
       ||((adc==ADC2)&&(ch==ADC2_TEMP_CHANNEL)) ) {
        ADC123_COMMON->CCR |= ADC_CCR_TSEN;     // Enable Temp channel
    } else if ( ((adc==ADC1)&&(ch==ADC1_VBAT_CHANNEL))
      ||((adc==ADC3)&&(ch==ADC3_VBAT_CHANNEL)) ) {
        ADC123_COMMON->CCR |= ADC_CCR_VBATEN;   // Enable VBAT channel
    } else if ( ((adc == ADC1) && (ch==ADC1_VREF_CHANNEL)) ) {
        ADC123_COMMON->CCR |= ADC_CCR_VREFEN;   // Enable VREF channel
    } else {
        chpos = lookforgpioinfo(adc,ch,&gpio,&pin);
        if( chused&BIT(chpos))
            return 1;
        gpio->ASCR = BIT(pin); //??
        BITFIELDSET(gpio->MODER,BITVALUE(3,pin*2),BITVALUE(3,pin*2));
        chused |= BIT(chpos);
    }
    if( ch < 10 ) {
        adc->SMPR1 |= (samp<<((ch-1)*3));
    } else {
        adc->SMPR2 |= (samp<<((ch-10)*3));
    }

    if ( (info&ADC_MODE_DIFFERENTIAL)&&(ch<16)  ) {
        adc->DIFSEL |= BIT(ch); /* differential inputs use next pin */
        chused |= BIT(ch+1);
    } else {
        adc->DIFSEL &= ~BIT(ch);
    }

    return 0;
}


uint32_t ADC_Read(ADC_TypeDef *adc, uint32_t ch) {
uint32_t v=0;
uint32_t timeout_cnt;

    // reset flags
    adc->ISR |= ADC_ISR_EOC|ADC_ISR_OVR|ADC_ISR_JEOC|ADC_ISR_EOS|ADC_ISR_ADRDY;

    BITFIELDSET(adc->CFGR,ADC_CFGR_RES,0);

    adc->SQR1 = (ch<<6)|1; // only one channel is sequence
    adc->CFGR &= ~ADC_CFGR_CONT;
    adc->CR |= ADC_CR_ADSTART;
    adc->CFGR &= ~ADC_CFGR_EXTEN;

    timeout_cnt = 4000;
    while( timeout_cnt && ((adc->ISR&ADC_ISR_EOC)==0)) timeout_cnt--;
    if( timeout_cnt == 0 ) return (uint32_t) -1;
    v = adc->DR;

    timeout_cnt = 4000;
    while( timeout_cnt && ((adc->ISR&ADC_ISR_EOS)==0)) timeout_cnt--;
    if( timeout_cnt == 0 ) return (uint32_t) -2;

    return v;
}


uint32_t ADC_ReadMultiple(ADC_TypeDef *adc, uint32_t *ch, uint32_t *val, int n) {
uint32_t i,sqr1,sqr2,sqr3,sqr4;
uint32_t timeout_cnt;

    adc->CFGR &= ~ADC_CFGR_CONT;

    sqr1 = sqr2 = sqr3 = sqr4 = 0;

    for(i=0;i<n;i++) {
        if( i <= 3 ) {                  /* From 0 to 3 : SQ1 to SQ4 */
            sqr1 |= (ch[i]<<((i)*6+6));
        } else if ( i <= 9 ) {          /* From 4 to 8 : SQ5 to SQ9 */
            sqr2 |= (ch[i]<<((i-4)*6));
        } else if ( i <= 14 ) {         /* From 9 to 13 : SQ10 to SQ14 */
            sqr3 |= (ch[i]<<((i-10)*6));
        } else  {                       /* From 14 to 15 : SQ15 to SQ16 */
            sqr4 |= (ch[i]<<((i-15)*6));
        }

    }
    adc->SQR1 = sqr1|n;
    adc->SQR2 = sqr2;
    adc->SQR3 = sqr3;
    adc->SQR4 = sqr4;

    adc->CR |= ADC_CR_ADSTART;
    adc->CFGR &= ~ADC_CFGR_EXTEN;

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

// See 3.15.1 in DM00108832
#define TS_CAL1 ( uint32_t) (* (uint16_t *) 0x1FFF75A8)
#define TS_CAL2 ( uint32_t) (* (uint16_t *) 0x1FFF75CA)

uint32_t
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

uint32_t
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

#define VREFINT_CAL ( uint32_t) (* (uint16_t *) 0x1FFF75AA)

uint32_t
ADC_ReadVREF(void) {
uint32_t v;

    v = ADC_Read(ADC1,ADC1_VREF_CHANNEL);
    v = 3000*VREFINT_CAL/v;
    //v >>= 12;

    return v;
}
