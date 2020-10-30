
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
#define GPIO_MODE_INP           0
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

#define LED_GREEN_PE_PIN   (8)
#define LED_RED_PB_PIN     (2)

// Bit masks for ODR (data) register
#define LED_GREEN_PE            BIT(LED_GREEN_PE_PIN)
#define LED_RED_PB              BIT(LED_RED_PB_PIN)
// Bit masks for MODER, OSPEEDR and PUPDR register (2 bit fields)
// Green LED on Port E
#define LED_GREEN_PE_MODE       BITFIELD(GPIO_MODE_OUT,LED_GREEN_PE_PIN*2)
#define LED_GREEN_PE_MODE_M     BITFIELD(GPIO_MODE_MASK,LED_GREEN_PE_PIN*2)
#define LED_GREEN_PE_OSPEED     BITFIELD(GPIO_OSPEED_VERYHIGH,LED_GREEN_PE_PIN*2)
#define LED_GREEN_PE_OSPEED_M   BITFIELD(GPIO_OSPEED_MASK,LED_GREEN_PE_PIN*2)
#define LED_GREEN_PE_PUPD       BITFIELD(GPIO_PUPD_UP,LED_GREEN_PE_PIN*2)
#define LED_GREEN_PE_PUPD_M     BITFIELD(GPIO_PUPD_MASK,LED_GREEN_PE_PIN*2)

// RED LED on Port E
#define LED_RED_PB_MODE         BITFIELD(GPIO_MODE_OUT,LED_RED_PB_PIN*2)
#define LED_RED_PB_MODE_M       BITFIELD(GPIO_MODE_MASK,LED_RED_PB_PIN*2)
#define LED_RED_PB_OSPEED       BITFIELD(GPIO_OSPEED_VERYHIGH,LED_RED_PB_PIN*2)
#define LED_RED_PB_OSPEED_M     BITFIELD(GPIO_OSPEED_MASK,LED_RED_PB_PIN*2)
#define LED_RED_PB_PUPD         BITFIELD(GPIO_PUPD_UP,LED_RED_PB_PIN*2)
#define LED_RED_PB_PUPD_M       BITFIELD(GPIO_PUPD_MASK,LED_RED_PB_PIN*2)
//@}


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


int main(void) {

    //    SystemCoreClockSet(MSI48M_CLOCKSRC, 0, 3, 0);

    BitSet(RCC->AHB2ENR,RCC_AHB2ENR_GPIOEEN);        // Enable GPIO Port E
    BitSet(RCC->AHB2ENR,RCC_AHB2ENR_GPIOBEN);        // Enable GPIO Port B

    __DSB();

    // Configurate GPIO for Green LED
    BitFieldSet(GPIOE->MODER,LED_GREEN_PE_MODE_M,LED_GREEN_PE_MODE);            // Set to output
    BitFieldSet(GPIOE->OSPEEDR,LED_GREEN_PE_OSPEED_M,LED_GREEN_PE_OSPEED);      // Set to high speed
    BitFieldSet(GPIOE->PUPDR,LED_GREEN_PE_PUPD_M,LED_GREEN_PE_PUPD_M);          // Turn on pull up
    BitClear(GPIOE->ODR,LED_GREEN_PE);                                          // Turn off LED

    // Configurate GPIO for Red LED
    BitFieldSet(GPIOB->MODER,LED_RED_PB_MODE_M,LED_RED_PB_MODE);                // Set to output
    BitFieldSet(GPIOB->OSPEEDR,LED_RED_PB_OSPEED_M,LED_RED_PB_OSPEED);          // Set to high speed
    BitFieldSet(GPIOB->PUPDR,LED_RED_PB_PUPD_M,LED_RED_PB_PUPD_M);              // Turn on pull up
    BitSet(GPIOB->ODR,LED_RED_PB);                                              // Turn on LED





    for (;;) {
       ms_delay(500);
       BitToggle(GPIOB->ODR,LED_RED_PB);
       BitToggle(GPIOE->ODR,LED_GREEN_PE);
    }
}
