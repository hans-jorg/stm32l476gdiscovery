
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

#include "lcd.h"
#include "led.h"
#include "uart.h"

#define BIT(N)                      (1UL<<(N))
#define BITMASK(M,N)                ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)               ((V)<<(N))
#define SETBITFIELD(VAR,MASK,VAL)   (VAR) = ((VAR)&~(MASK))|(VAL)

static const uint32_t sorder[] = { SEGA, SEGB, SEGC, SEGD, SEGE, SEGF,
                             SEGG, SEGH, SEGJ, SEGK, SEGM, SEGN,
                             SEGP, SEGQ };

static int32_t result = -1;

uint32_t segs[6] = { 0,0,0,0,0,0 };

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

char *msg = "Hello\n";

#define UART UART2

int main(void) {
uint32_t t;
uint32_t sindex = 0;
int i;

    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    SysTick_Config(SystemCoreClock/1000);

    APBPeripheralClockSet(0,0); /* Enable APBx */

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* Enable SysConfig, Comp, etc. */

    t = LED_Init(LED_ALL);

    t = LCD_Init();

    result = t;

    LED_Toggle(LED_GREEN);

    LCD_WriteString("uart");
    Delay(10000);

    LCD_Clear();

    UART_Init(UART,UART_NOPARITY|UART_8BITS|UART_2_STOP|UART_BAUD_9600|UART_OVER8);

    for (;;) {
        UART_WriteString(UART,msg);
        Delay(2000);
        LED_Toggle(LED_RED);
    }
}
