
/**
 ** @file     main.c
 ** @brief    Main program to send a hello message thru UART 1
 **
 ** @version  V1.0
 ** @date     23/01/2016
 **
 ** @note     Direct access to registers
 ** @note     No library except CMSIS is used
 ** @note     UART Driver for STM32L476VG
 ** @note     Uses UART2 (connected to STLINK) and a VCP (/dev/ttyACMx)
 ** @note     Solder bridges SB13 and SB16 must be ON (soldered) and
 **           SB15 and SB17 must be OFF (NOT connected)
 **
 ******************************************************************************/



#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "led.h"
#include "uart.h"

#define BIT(N)                      (1UL<<(N))
#define BITMASK(M,N)                ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)               ((V)<<(N))
#define SETBITFIELD(VAR,MASK,VAL)   (VAR) = ((VAR)&~(MASK))|(VAL)


volatile uint32_t tick = 0;

void SysTick_Handler(void) {
    tick++;
}

void Delay(uint32_t ms) {  // delay for ms milliseconds
uint32_t s = tick;

    while(  (tick-s) < ms ) {
        __NOP();
    }

}

#define UART UART_2

int main(void) {
char *msg = "Hello";

    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    SysTick_Config(SystemCoreClock/1000);

    APBPeripheralClockSet(0,0); /* Enable APBx */

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* Enable SysConfig, Comp, etc. */

    LED_Init(LED_ALL);

    LED_Toggle(LED_GREEN);

    UART_Init(UART,UART_NOPARITY|UART_8BITS|UART_2_STOP|UART_BAUD_9600|UART_OVER8);

    for (;;) {
        UART_WriteString(UART,msg);
        Delay(2000);
        LED_Toggle(LED_RED);
    }
}
