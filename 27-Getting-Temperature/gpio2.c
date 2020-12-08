

/**
 * @file  gpio.c
 * @brief GPIO Abstraction
 *
 * @note  To use an unified approach to address GPIO bit in 2 ports, pins of different ports
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
///@{
#define GPIO_MODE_IN            0
#define GPIO_MODE_OUT           1
#define GPIO_MODE_ALT           2
#define GPIO_MODE_ANA           3
#define GPIO_MODE_MASK          3

#define GPIO_OSPEED_LOW         0
#define GPIO_OSPEED_MED         1
#define GPIO_OSPEED_HIGH        2
#define GPIO_OSPEED_VERYHIGH    3
#define GPIO_OSPEED_MASK        3

#define GPIO_PUPD_NONE          0
#define GPIO_PUPD_UP            1
#define GPIO_PUPD_DOWN          2
#define GPIO_PUPD_RES           3
#define GPIO_PUPD_MASK          3
///#}



/**
 * @brief Generates a word with 2-bit fields set according another word with 1-bit field.
 *        Each field is filled with the value v.
 *
 * @note  v must be 2 bit wide and not equal zero
 *
 */
static uint32_t mk2from1(uint32_t m, uint32_t v) {
uint32_t x;

    if( !v )
        return 0;

    x = 0;
    while( v ) {
        if( m& 1 ) {
            x |= v;
        }
        m>>=1;
        v<<=2;
    }
    return x;
}

void GPIO_EnableClock(GPIO_TypeDef *port) {
uint32_t m;

    if      ( port == GPIOA ) m = RCC_AHB2ENR_GPIOAEN;
    else if ( port == GPIOB ) m = RCC_AHB2ENR_GPIOBEN;
    else if ( port == GPIOC ) m = RCC_AHB2ENR_GPIOCEN;
    else if ( port == GPIOD ) m = RCC_AHB2ENR_GPIODEN;
    else if ( port == GPIOE ) m = RCC_AHB2ENR_GPIOEEN;
    else if ( port == GPIOF ) m = RCC_AHB2ENR_GPIOFEN;
    else if ( port == GPIOG ) m = RCC_AHB2ENR_GPIOGEN;
    else if ( port == GPIOH ) m = RCC_AHB2ENR_GPIOHEN;
    else                      m = 0;

    RCC->AHB2ENR |= m;

    __DSB();

}

void GPIO_Init(GPIO_TypeDef *port, uint16_t input, uint16_t output) {
uint32_t v1,v2,v,m,pinin,pinout;

    GPIO_EnableClock(port);

    __DSB();

    // Set mode
    m =  mk2from1(pinout|pinin,GPIO_MODE_MASK);
    v1 = mk2from1(pinout,GPIO_MODE_OUT);
    v2 = mk2from1(pinin,GPIO_MODE_IN);
    v = v1 | v2;
    SETBITFIELD(port->MODER,m,v);

    // Set output speed
    m = mk2from1(pinout,GPIO_OSPEED_MASK);
    v1 = mk2from1(pinout,GPIO_OSPEED_VERYHIGH);
    SETBITFIELD(port->OSPEEDR,m,v1);

    // Set pullup
    m =  mk2from1(pinout|pinin,GPIO_PUPD_MASK);
    v1 = mk2from1(pinout|pinin,GPIO_PUPD_UP);
    SETBITFIELD(port->PUPDR,m,v1);

}

