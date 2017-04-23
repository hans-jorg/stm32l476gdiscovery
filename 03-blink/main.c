
/**
 * @file     main.c
 * @brief    Blink LEDs using interrupts and CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     The blinking frequency of the red LED depends on core frequency
 * @note     Direct access to registers
 * @note     No library used
 *
 *
 ******************************************************************************/

/** @mainpage 03-blink: Using interrutps
 *   @par Description: Blinks LEDs using interrupts and CMSIS
 *   @note The green LED blinks controlled by SysTickHandler
 *   @note The red LED blink using a counting delay
 */


#include "stm32l476xx.h"
#include "system_stm32l476.h"


/**
 * @brief Macros for bit and bitfields manipulation
 * @note  Small cases in macro names to improve readability
 *
 * BIT(N)                Creates a bit mask with only the bit N set
 * BITFIELD(V,N)         Creates a bit mask with value V at position N
 * BITFIELDMASK(M,N)     Creates a bit mask with bits between M and N (inclusive) set to 1

 * Operations
 * BitSet(V,M)           Set all bits in V with corresponding bits set in M
 * BitClear(V,M)         Clears all bits in V with corresponding bits set in M
 * BitToggle(V,M)        Toggles all bits in V with corresponding bits set in M
 *
 * Bitfield manipulation
 * BitFieldSet(V,M,X)    Clear bits in V which corresponds to bits set in M and set them according X
 * BitFieldClear(V,M)    Clear bits in V which corresponds to bits set in M
 */

///@{
#define BIT(N)                     (1UL<<(N))
#define BITFIELD(V,N)              ((V)<<(N))
#define BITFIELDMASK(M,N)          ((BIT((M)-(N)+1)-1)<<(N))

#define BitSet(V,M)                (V)|=(M)
#define BitClear(V,M)              (V)&=~(M)
#define BitToggle(V,M)             (V)^=(M)

#define BitFieldSet(VAR,MASK,VAL)  (VAR) = ((VAR)&~(MASK))|(VAL)
#define BitFieldClear(VAR,MASK)    (VAR) &= ~(MASK)

///@}


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
 * @brief Quick and dirty delay routine
 *
 * It gives approximately 1ms delay at 4 MHz (MSI)
 *
 */

void ms_delay(volatile int ms) {
   while (ms-- > 0) {
      volatile int x=7000;
      while (x-- > 0)
         __NOP();
   }
}

/**
 * @brief GPIO Abstraction
 *
 * @note  To use an unified approach to address GPIO bit in 2 ports, pins of different ports
 *        are mapped to low and high bit
 *
 */

#define MKWORD(PE,PB) ((PE)<<16)|((PB)&0xFFFF)
#define GETPORTB(X) ((X)&0xFFFF)
#define GETPORTE(X) ((X)>>16)

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
    pinout = GETPORTE(output);
    pinin  = GETPORTE(input);

    GPIO_ConfigurePort(GPIOE,pinout,pinin);

    // Configure GPIO Port B. Only pins in input and output parameters
    pinout = GETPORTB(output);
    pinin  = GETPORTB(input);

    GPIO_ConfigurePort(GPIOE,pinout,pinin);

}

static inline void GPIO_Write(uint32_t zeroes, uint32_t ones) {
uint32_t pinzeroes,pinones;

    if( GETPORTE(zeroes|ones) ) {
        pinzeroes = GETPORTE(zeroes);
        pinones   = GETPORTE(ones);
        GPIOE->ODR = (GPIOE->ODR&~(pinones|pinzeroes))|pinones;
    }
    if( GETPORTB(zeroes|ones) ) {
        pinzeroes = GETPORTB(zeroes);
        pinones   = GETPORTB(ones);
        GPIOE->ODR = (GPIOB->ODR&~(pinones|pinzeroes))|pinones;
    }
}

inline void GPIO_Toggle(uint32_t pins) {
uint32_t p;

    if( GETPORTE(pins) ) {
        p = GETPORTE(pins);
        GPIOE->ODR ^= p;
    }
    if( GETPORTB(pins) ) {
        p = GETPORTB(pins);
        GPIOB->ODR ^= p;
    }
}


/**
 * @brief LED Symbols
 *
 *     LEDs are in different ports.
 *     In order to avoid wrong use, the port identifier should be appended to the symbol.
 *
 *     It is necessary to have different symbols to configure the GPIO
 *     One to specify bit in the ODR register and another to specify a 2-bit wide field in
 *     MODER,OSPEER and PUPDR registers.
 *     To write on a 2-bit wide field it is necessary to erase all bits on it using a mask.
 *
 */
/* Bit numbers for LEDS
 * LED     GPIO      Pin
 * Green   GPIOE      8
 * Red     GPIOB      2
 */

#define LED_GREEN   MKWORD(BIT(8),0)
#define LED_RED     MKWORD(0,BIT(2))

//@}

int main(void) {

    //    SystemCoreClockSet(MSI48M_CLOCKSRC, 0, 3, 0);

    GPIO_Init(0,LED_GREEN|LED_RED);

    GPIO_Write(LED_GREEN,LED_RED);

    for (;;) {
       ms_delay(500);
       GPIO_Write(LED_RED,LED_GREEN);
       ms_delay(500);
       GPIO_Write(LED_GREEN,LED_RED);
    }
}
