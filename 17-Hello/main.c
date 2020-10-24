
/**
 ** @file     main.c
 ** @brief    Main program to display hello in the GH08172T LCD display.
 **           After that, it shows a pattern of revolving segments
 **
 ** @version  V1.0
 ** @date     23/01/2016
 **
 ** @note     Direct access to registers
 ** @note     No library except CMSIS is used
 ** @note     LCD Driver for GH08172 LCD Display in STM32L476 Discovery board
 ** @note     Uses LCD interface of STM32L476VG
 **
 ******************************************************************************/

#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "lcd.h"
#include "led.h"

#define BIT(N)                      (1UL<<(N))
#define BITMASK(M,N)                ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)               ((V)<<(N))
#define SETBITFIELD(VAR,MASK,VAL)   (VAR)=(((VAR)&~(MASK))|(VAL))

static const uint32_t sorder[] = { SEGA, SEGB, SEGC, SEGD, SEGE, SEGF,
                                   SEGG, SEGH, SEGJ, SEGK, SEGM, SEGN,
                                   SEGP, SEGQ };

static int32_t result = -1;

uint32_t segs[6] = { 0,0,0,0,0,0 };

uint32_t allsegs[6] = { 0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF };

uint32_t alldispsegs[8] = { 0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF };

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

    LCD_WriteString("hello");
    Delay(10000);

    LCD_WriteSegments(allsegs);
    Delay(10000);

    LCD_WriteToRAM(alldispsegs);
    Delay(10000);


    LCD_Clear();


    for (;;) {
        for(i=0;i<6;i++) {
            segs[i] = sorder[sindex];
        }
        LCD_WriteSegments(segs);
        Delay(2000);
        sindex++;
        if( sindex >= sizeof(sorder)/sizeof(uint32_t)) sindex = 0;
    }
}
