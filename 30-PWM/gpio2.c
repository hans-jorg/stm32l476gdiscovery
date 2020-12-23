

/**
 * @file  gpio.c
 * @brief GPIO Abstraction
 *
 * @note  To use an unified approach to address GPIO bit in 2 gpios, pins of different gpios
 *        are mapped to low and high bit
 * @note  a Read-Modify-Write cycle is used
 *
 *
 */



#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "gpio2.h"

#define BIT(N) (1UL<<(N))
#define BITMASK(M,N)  ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)  ((V)<<(N))
#define SETBITFIELD(VAR,MASK,VAL) (VAR) = ((VAR)&~(MASK))|(VAL)


/**
 * @brief  GPIO configuration
 *
 * MODEx Register : Mode register
 *           2 bits for each pin, MODE15..0
 *           00 : Input
 *           01 : Output
 *           10 : Alternate Function
 *           11 : Analog
 *
 * OSPEEDx Register : Output speed register
 *           2 bits for each pin, OSPEED15..0
 *           00 : Low speed
 *           01 : Medium speed
 *           10 : High speed
 *           11 : Very high speed
 *
 * PUPDRx Register : Pullup/Pulldown register
 *           2 bits for each pin, PUPD15..0
 *           00 : No pullup/pulldown
 *           01 : Pullup
 *           10 : Pulldown
 *           11 : Reserved
 *
 * ODRx Register : output data register
 *           1 bit for each pin, OD15..0
 */



/**
 * @brief Generates a 32 bit word with 2-bit fields set according to a 16 bit word with 1-bit field.
 *        Each 2-bit field is filled with the value v.
 *
 * @note  v must be 2 bit wide and not equal zero
 *
 */
static uint32_t mk2from1(uint16_t m, uint8_t f) {
uint32_t x;
uint32_t bv;

    if( !f )
        return 0;

    x = 0;
    bv = 1<<15;
    while( bv ) {
        x<<=2;
        if( m&bv ) {
            x |= f;
        }
        bv>>=1;
    }
    return x;
}

void GPIO_EnableClock(GPIO_TypeDef *gpio) {
uint32_t m;

    if      ( gpio == GPIOA ) m = RCC_AHB2ENR_GPIOAEN;
    else if ( gpio == GPIOB ) m = RCC_AHB2ENR_GPIOBEN;
    else if ( gpio == GPIOC ) m = RCC_AHB2ENR_GPIOCEN;
    else if ( gpio == GPIOD ) m = RCC_AHB2ENR_GPIODEN;
    else if ( gpio == GPIOE ) m = RCC_AHB2ENR_GPIOEEN;
    else if ( gpio == GPIOF ) m = RCC_AHB2ENR_GPIOFEN;
    else if ( gpio == GPIOG ) m = RCC_AHB2ENR_GPIOGEN;
    else if ( gpio == GPIOH ) m = RCC_AHB2ENR_GPIOHEN;
    else                      m = 0;

    RCC->AHB2ENR |= m;

    __DSB();

}

void GPIO_Init(GPIO_TypeDef *gpio, uint16_t input, uint16_t output) {
uint32_t v1,v2,v,m,pinin,pinout;

    GPIO_EnableClock(gpio);

    __DSB();

    GPIO_ConfigurePins(gpio,input,GPIO_MODE_IN);
    GPIO_ConfigurePins(gpio,output,GPIO_MODE_OUT);

}


void GPIO_ConfigurePins(GPIO_TypeDef *gpio, uint16_t pins, uint32_t conf) {
uint32_t v1,v2,m;
uint32_t mode,speed,pull;

    GPIO_EnableClock(gpio);

    __DSB();

    // mode is in lower 4 bits. if zero, do not modify.
    if( conf & 0xF ) {
        m =  mk2from1(pins,GPIO_MODE_MASK);
        mode = (conf&0xF)-1;
        v2 = mk2from1(pins,mode);
        gpio->MODER = ((gpio->MODER&~m))|v2;
    }

    // Set output speed
    if( conf & 0xF0 ) {
        m =  mk2from1(pins,GPIO_MODE_MASK);
        speed = ((conf&0xF0)>>4)-1;
        v2 = mk2from1(pins,speed);
        gpio->OSPEEDR = ((gpio->OSPEEDR&~m))|v2;
    }

    // Set pullup
    if( conf & 0xF00 ) {
        m =  mk2from1(pins,GPIO_MODE_MASK);
        pull = ((conf&0xF00)>>4)-1;
        v2 = mk2from1(pins,pull);
        gpio->PUPDR = ((gpio->PUPDR&~m))|v2;
    }

}

