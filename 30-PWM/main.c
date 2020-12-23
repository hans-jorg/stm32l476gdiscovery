
/*
 * Blink
 * Blink a LED without using libraries
 *
 * Uses CMSIS definitions
 */


#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "lcd.h"
#include "led.h"
#include "quadrature.h"

#define BIT(N) (1UL<<(N))
#define BITMASK(M,N)  ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)  ((V)<<(N))
#define SETBITFIELD(VAR,MASK,VAL) (VAR) = ((VAR)&~(MASK))|(VAL)

static volatile int sec_tick = 0;
static volatile int ms_tick = 0;
void SysTick_Handler(void) {

    ms_tick++;  // overflow in 49 days!!!

    if( sec_tick > 0 ) {
        sec_tick--;
    } else {
        // Run every sec
        sec_tick = 999;
        LED_Toggle(LED_RED);
    }
}

void Delay(uint32_t ms) {  // delay for ms milliseconds
uint32_t s = ms_tick;

    while(  (ms_tick-s) < ms ) {
        __NOP();
    }

}


static void itoa(int v, char *s, int len) {
char *p = s+len-1;
int neg = 0;
int fill = 0;

    if( v < 0 ) {
        v = -v;
        neg = 1;
    }
    *p-- = 0;
    if( v == 0 ) {
        *p-- = '0';
    } else {
        while( v > 0 && p >= s ) {
            *p-- = (v%10)+'0';
            v /= 10;
        }
    }
    if( (v > 0) || (neg&&(p<s) ) ) {
        p = s+len-2;
        while( p >= s ) *p-- = '*';
        fill = 1;
    } else {
        if( neg ) *p-- = '-';
        while ( p >= s ) *p-- = ' ';
    }
}





int main(void) {
int temp;
char s[6];

    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    SysTick_Config(SystemCoreClock/1000);

    APBPeripheralClockSet(0,0); /* Enable APBx */

    LED_Init(LED_ALL);

    LCD_Init();

    LED_Write(LED_GREEN,0);

    LCD_WriteString("hello");

    Delay(2000);

    LCD_Clear();

    Quadrature_Init();

    for (;;) {
        Delay(1000);
        temp = Quadrature_GetPosition();
        itoa(temp,s,6);
        LCD_WriteString(s);
    }
}
