

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

#include "bitmanip.h"
#include "gpio.h"



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

static void GPIO_ConfigurePort(GPIO_TypeDef *port, uint32_t input, uint32_t output) {
uint32_t v1,v2,v,m,pinin,pinout;

    // Set mode
    m =  mk2from1(pinout|pinin,GPIO_MODE_MASK);
    v1 = mk2from1(pinout,GPIO_MODE_OUT);
    v2 = mk2from1(pinin,GPIO_MODE_IN);
    v = v1 | v2;
    BitFieldSet(port->MODER,m,v);

    // Set output speed
    m = mk2from1(pinout,GPIO_OSPEED_MASK);
    v1 = mk2from1(pinout,GPIO_OSPEED_VERYHIGH);
    BitFieldSet(port->OSPEEDR,m,v1);

    // Set pullup
    m =  mk2from1(pinout|pinin,GPIO_PUPD_MASK);
    v1 = mk2from1(pinout|pinin,GPIO_PUPD_UP);
    BitFieldSet(port->PUPDR,m,v1);

}

void GPIO_Init(uint32_t input, uint32_t output) {
uint32_t pinin,pinout;

    BitSet(RCC->AHB2ENR,RCC_AHB2ENR_GPIOEEN);        // Enable GPIO Port E
    BitSet(RCC->AHB2ENR,RCC_AHB2ENR_GPIOBEN);        // Enable GPIO Port B

    __DSB();

    // Configure GPIO Port E. Only pins in input and output parameters
    pinout = GPIO_GETPORTE(output);
    pinin  = GPIO_GETPORTE(input);

    GPIO_ConfigurePort(GPIOE,pinout,pinin);

    // Configure GPIO Port B. Only pins in input and output parameters
    pinout = GPIO_GETPORTB(output);
    pinin  = GPIO_GETPORTB(input);

    GPIO_ConfigurePort(GPIOB,pinout,pinin);

}
