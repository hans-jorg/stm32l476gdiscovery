
/**
 ** @file     uart.h
 ** @brief    Hardware Abstraction Layer (HAL) for UART
 ** @version  V1.0
 ** @date     23/01/2016
 **
 ** @note     Direct access to registers
 ** @note     No library except CMSIS is used
 ** @note     Only asynchronous communication
 **
 **/


#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "uart.h"

/**
 ** @brief Bit manipulation macros
 **/
//@{
#define BIT(N)                      (1UL<<(N))
#define BITMASK(M,N)                ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)               ((V)<<(N))
#define SETBITFIELD(VAR,MASK,VAL)   (VAR)=(((VAR)&~(MASK))|(VAL))
//@}

/**
 ** @brief Info and data area for UARTS
 **/
typedef struct {
    USART_TypeDef  *device;
    int             irqlevel;
    int             irqn;
    char            inbuffer;
    char            outbuffer;
} UART_Info;

/**
 ** @brief List of known UARTs
 **
 ** @note  Actually pointers to UARTs
 **/
//@{
static UART_Info uarttab[] = {
    { LPUART1,  6,  LPUART1_IRQn, 0, 0 },
    { USART1,   6,  USART1_IRQn,  0, 0 },
    { USART2,   6,  USART2_IRQn,  0, 0 },
    { USART3,   6,  USART3_IRQn,  0, 0 },
    { UART4,    6,  UART4_IRQn,   0, 0 },
    { UART5,    6,  UART5_IRQn,   0, 0 }
};
static const int uarttabsize = sizeof(uarttab)/sizeof(USART_TypeDef *);
//@}


/**
 ** @brief  Interrupt routines for USART, UART and LPUART
 **/
///@{

/// IRQ Handler for LPUART1
void LPUART1_IRQHandler(void) {
USART_TypeDef  *uart = LPUART1;

    if( uart->ISR & USART_ISR_RXNE  ) { // RX not empty
        uarttab[0].inbuffer = uart->RDR;
    }
    if( uart->ISR & USART_ISR_TXE  ) { // RX not empty
        if ( uarttab[0].outbuffer ) {
            uart->TDR = uarttab[0].outbuffer;
            uarttab[0].outbuffer = 0;
        }
    }
    uart->ICR = 0x00121BDF;     // Clear all pending interrupts
}

/// IRQ Handler for USART1
void USART1_IRQHandler(void) {
USART_TypeDef  *uart = USART1;

    if( uart->ISR & USART_ISR_RXNE  ) { // RX not empty
        uarttab[1].inbuffer = uart->RDR;
    }
    if( uart->ISR & USART_ISR_TXE  ) { // RX not empty
        if ( uarttab[1].outbuffer ) {
            uart->TDR = uarttab[1].outbuffer;
            uarttab[1].outbuffer = 0;
        }
    }
    uart->ICR = 0x00121BDF;     // Clear all pending interrupts
}

/// IRQ Handler for USART2
void USART2_IRQHandler(void) {
USART_TypeDef  *uart = USART2;

    if( uart->ISR & USART_ISR_RXNE  ) { // RX not empty
        uarttab[2].inbuffer = uart->RDR;
    }
    if( uart->ISR & USART_ISR_TXE  ) { // RX not empty
        if ( uarttab[2].outbuffer ) {
            uart->TDR = uarttab[2].outbuffer;
            uarttab[2].outbuffer = 0;
        }
    }
    uart->ICR = 0x00121BDF;     // Clear all pending interrupts
}

/// IRQ Handler for USART3
void USART3_IRQHandler(void) {
USART_TypeDef  *uart = USART3;

    if( uart->ISR & USART_ISR_RXNE  ) { // RX not empty
        uarttab[3].inbuffer = uart->RDR;
    }
    if( uart->ISR & USART_ISR_TXE  ) { // TX not empty
        if ( uarttab[3].outbuffer ) {
            uart->TDR = uarttab[3].outbuffer;
            uarttab[3].outbuffer = 0;
        }
    }
    uart->ICR = 0x00121BDF;     // Clear all pending interrupts
}

/// IRQ Handler for UART4
void UART4_IRQHandler(void) {
USART_TypeDef  *uart = UART4;

    if( uart->ISR & USART_ISR_RXNE  ) { // RX not empty
        uarttab[4].inbuffer = uart->RDR;
    }
    if( uart->ISR & USART_ISR_TXE  ) { // TX  empty
        if ( uarttab[4].outbuffer ) {
            uart->TDR = uarttab[4].outbuffer;
            uarttab[4].outbuffer = 0;
        }
    }
    uart->ICR = 0x00121BDF;     // Clear all pending interrupts
}

/// IRQ Handler for UART5
void UART5_IRQHandler(void) {
USART_TypeDef  *uart = UART5;

    if( uart->ISR & USART_ISR_RXNE  ) { // RX not empty
        uarttab[5].inbuffer = uart->RDR;
    }
    if( uart->ISR & USART_ISR_TXE  ) { // TX not empty
        if ( uarttab[5].outbuffer ) {
            uart->TDR = uarttab[5].outbuffer;
            uarttab[5].outbuffer = 0;
        }
    }
    uart->ICR = 0x00121BDF;     // Clear all pending interrupts
}
///@}


/**
 ** @brief UART Initialization
 **
 ** @note  Use defines in uart.h to configure the uart, or'ing the parameters
 **/
int
UART_Init(int uartn, uint32_t info) {
uint32_t baudrate,div,t,over;
USART_TypeDef * uart;

    uart = uarttab[uartn].device;

    // Enable Clock
    if ( uartn == 0 ) {         // LPUART1
        RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
    } else if ( uartn == 1 ) {  // USART1
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    } else {                      // USART2..5
        RCC->APB1ENR1 |= BIT(uartn+17-2); // RCC_APB1ENR1_USARTxEN
    }

    // Configuration of LPUART is after the last UART
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

    // enable interrupts (only TCIE and RX)

    uart->CR1 |= USART_CR1_RXNEIE;      // Enable interrupt when RX not empty
    uart->CR1 |= USART_CR1_TXEIE;       // Enable interrupt when TX is empty

    // Enable interrupts on NVIC
    NVIC_SetPriority(uarttab[uartn].irqn,uarttab[uartn].irqlevel);
    NVIC_ClearPendingIRQ(uarttab[uartn].irqn);
    NVIC_EnableIRQ(uarttab[uartn].irqn);

    // Enable UART
    uart->CR1 |= USART_CR1_UE;
    return 0;
}

/**
 ** @brief UART Send a character
 **
 **/
int
UART_WriteChar(int uartn, uint32_t c) {
USART_TypeDef *uart;


    uart = uarttab[uartn].device;

    while( (uart->ISR&USART_ISR_TEACK)==0 ) {}
    uart->TDR = c;

    return 0;
}

/**
 ** @brief UART Send a string
 **
 ** @note  It uses UART_WriteChar
 **
 **/
int
UART_WriteString(int uartn, char s[]) {

    while(*s) {
        UART_WriteChar(uartn,*s++);
    }
    return 0;
}

/**
 ** @brief Read a character from UART
 **
 ** @note  It blocks until a character is entered
 **
 **/
int
UART_ReadChar(int uartn) {
USART_TypeDef *uart;

    uart = uarttab[uartn].device;

    while( (uart->ISR & USART_ISR_RXNE) == 0 ) {}
    return uart->RDR;
}


/**
 ** @brief UART Send a string
 **
 ** @note  It block until "n" characters are entered or
 **        a newline is entered
 **
 ** @note  It uses UART_ReadChar
 **
 **/
int
UART_ReadString(int uartn, char *s, int n) {
int i;

    for(i=0;i<n-1;i++) {
        s[i] = UART_ReadChar(uartn);
        if( s[i] == '\n' || s[i] == '\r' )
            break;
    }
    s[i] = '\0';
    return i;
}

/**
 ** @brief UART Get Status
 **
 ** @note  Return status
 **
 **/
int
UART_GetStatus(int uartn) {
USART_TypeDef *uart;

    uart = uarttab[uartn].device;

    return uart->ISR;

}

