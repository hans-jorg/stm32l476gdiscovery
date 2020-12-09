
/*
 * Driver of USART modules in Asynchronous mode (UART)
 *
 * Version 1 : No interrupts
 * Oversampling = 8
 * Clock = SystemCoreClock
 * No Handshaking
 *
 */

#include "stm32l476xx.h"
#include "system_stm32l476.h"

#define BIT(N) (1UL<<(N))
#define BITMASK(M,N)  ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)  ((V)<<(N))
#define SETBITFIELD(VAR,MASK,VAL) (VAR) = ((VAR)&~(MASK))|(VAL)

#define UART_PARITY      0x3
#define UART_NOPARITY    0x0
#define UART_EVENPARITY  0x1
#define UART_ODDPARITY   0x2

#define UART_SIZE        0xC
#define UART_8BITS       0x4
#define UART_9BITS       0x8
#define UART_7BITS       0xC

#define UART_STOP        0x70
#define UART_1_STOP      0x10
#define UART_0_5_STOP    0x20
#define UART_2_STOP      0x30
#define UART_1_5_STOP    0x40

#define UART_OVER8       0x80
#define UART_OVER16      0x00

#define UART_BAUD          0xFFFFF00
#define UART_BAUD_150      BITVALUE(150,8)
#define UART_BAUD_300      BITVALUE(300,8)
#define UART_BAUD_600      BITVALUE(600,8)
#define UART_BAUD_1200     BITVALUE(1200,8)
#define UART_BAUD_2400     BITVALUE(2400,8)
#define UART_BAUD_4800     BITVALUE(4800,8)
#define UART_BAUD_9600     BITVALUE(9600,8)
#define UART_BAUD_19200    BITVALUE(19200,8)
#define UART_BAUD_38400    BITVALUE(38400,8)
#define UART_BAUD_57600    BITVALUE(57600,8)
#define UART_BAUD_115200   BITVALUE(115200,8)

static USART_TypeDef *uarttab[] = {
    LPUART1,
    USART1,
    USART2,
    USART3,
    UART4,
    UART5
};

static const int uarttabsize = sizeof(uarttab)/sizeof(USART_TypeDef *);

uint32_t
UART_Init(USART_TypeDef *uart, uint32_t info) {
uint32_t baudrate,div,t,over;
int uartn;

    // Enable Clock
    for(uartn=0;uartn<uarttabsize;uartn++) {
        if( uarttab[uartn] == uart )
            break;
    }
    if( uartn == uarttabsize )
        return 1;

    if ( uartn == 0 ) {         // LPUART1
        RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
    } else if ( uartn == 1 ) {  // USART1
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    } else {                      // USART2..5
        RCC->APB1ENR1 |= BIT(uartn+17-2); // RCC_APB1ENR1_USARTxEN
    }

    if( uartn == 0 ) uartn = uarttabsize;

    t = RCC->CCIPR&~BITVALUE(3,uartn*2-2);
    t |= BITVALUE(1,uartn*2-2); // SYSCLK as Clock Source
    RCC->CCIPR = t;

    // Configure
    t = uart->CR1;
    t &= ~(USART_CR1_M|USART_CR1_OVER8|USART_CR1_PCE|USART_CR1_PS|USART_CR1_UE);
    switch( info&UART_SIZE ) {
    case UART_8BITS:                  ; break;
    case UART_7BITS: t |= USART_CR1_M0; break;
    case UART_9BITS: t |= USART_CR1_M1; break;
    default:
        return 2;
    }
    t |= USART_CR1_TE|USART_CR1_RE;
    switch( info&UART_PARITY ) {
    case UART_NOPARITY:                                  ; break;
    case UART_ODDPARITY:  t |= USART_CR1_PCE|USART_CR1_PS; break;
    case UART_EVENPARITY: t |= USART_CR1_PCE;              break;
    }
    if( info&UART_OVER8 ) {
        t |= USART_CR1_OVER8;
        over = 8;
    } else {
        t &= ~USART_CR1_OVER8;
        over = 16;
    }
    uart->CR1 = t;

    t = uart->CR2&~UART_STOP;
    switch( info&UART_STOP ) {
    case UART_1_STOP:                ; break;
    case UART_0_5_STOP:  t |= 1;     ; break;
    case UART_2_STOP:    t |= 2;     ; break;
    case UART_1_5_STOP:  t |= 3;     ; break;
    default:
        return 3;
    }
    uart->CR2 = t;

    // Configure Baudrate
    baudrate = ((info&UART_BAUD)>>8);
    div      = SystemCoreClock/baudrate/over; // round it?

    uart->BRR = (div&~0xF)|((div&0xF)>>1);

    // Enable UART
    uart->CR1 |= USART_CR1_UE;
    return 0;
}

uint32_t
UART_WriteChar(USART_TypeDef *uart, uint32_t c) {


    while( (uart->ISR&USART_ISR_TEACK)==0 ) {}
    uart->TDR = c;
    return 0;
}

uint32_t
UART_WriteString(USART_TypeDef *uart, char s[]) {

    while(*s) {
        UART_WriteChar(uart,*s++);
    }
    return 0;
}
