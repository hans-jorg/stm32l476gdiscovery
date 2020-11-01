
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

/**
 ** @brief UART Configuration
 **
 ** @note No interrupts are used
 ** Oversampling    : 8
 ** Clock source    : SystemCoreClock
 ** Handshaking     : No
 **/


#include "stm32l476xx.h"
#include "system_stm32l476.h"
#include "buffer.h"
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

/// Buffer size for input and output
#define INPUTBUFFERSIZE 100
#define OUTPUTBUFFERSIZE 100
/// Interrupt level
#define RXINTLEVEL 6
#define TXINTLEVEL 6

/**
 * @brief   Global variables
 *
 * @note    To avoid use of malloc, it uses a macro to define area at compile time
 */

DECLARE_BUFFER_AREA(inputbufferarea,INPUTBUFFERSIZE);
DECLARE_BUFFER_AREA(outputbufferarea,OUTPUTBUFFERSIZE);
buffer inputbuffer = 0;
buffer outputbuffer = 0;


/**
 ** @brief List of known UARTs
 **
 ** @note  Actually pointers to UARTs
 **/
//@{
static USART_TypeDef *uarttab[] = {
    LPUART1,
    USART1,
    USART2,
    USART3,
    UART4,
    UART5
};
static const int uarttabsize = sizeof(uarttab)/sizeof(USART_TypeDef *);
//@}

/**
 * Interrupt routine for USART2
 */
void
USART2_IRQHandler(void) {


}

/**
 ** @brief UART Initialization
 **
 ** @note  Use defines in uart.h to configure the uart, or'ing the parameters
 **/
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

    // Enable buffers
    inputbuffer  = buffer_init(inputbufferarea,INPUTBUFFERSIZE);
    outputbuffer = buffer_init(outputbufferarea,OUTPUTBUFFERSIZE);

    return 0;
}

/**
 * @brief   Resets UART
 */

void UART_Reset(USART_TypeDef *uart) {
int uartn;

    // Enable Clock
    for(uartn=0;uartn<uarttabsize;uartn++) {
        if( uarttab[uartn] == uart )
            break;
    }
    if( uartn == uarttabsize )
        return;

    // Disable UART
    uart->CR1 &= ~USART_CR1_UE;

    buffer_deinit(inputbuffer);
    buffer_deinit(outputbuffer);

}

/**
 ** @brief UART Send a character
 **
 **/
uint32_t
UART_WriteChar(USART_TypeDef *uart, uint32_t c) {


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
uint32_t
UART_WriteString(USART_TypeDef *uart, char s[]) {

    while(*s) {
        UART_WriteChar(uart,*s++);
    }
    return 0;
}

/**
 ** @brief UART Read a char
 **
 ** @note  It uses UART_WriteChar
 **
 **/
uint32_t
UART_ReadChar(USART_TypeDef *uart) {


    return 0;
}

/**
 * @brief   Flush buffer
 *
 * @note    Does block!!!!!
 */

int UART_Flush(USART_TypeDef *uart) {
int cnt;
int ch;

    // Clear input buffer
    buffer_clear(inputbuffer);

    // Clear output buffer
    cnt = 0;
    // Disable interrupts
    //UART0->IEN &= ~(UART_IEN_TXC|UART_IEN_RXDATAV);
    while( ! buffer_empty(outputbuffer) ) {
        // Wait until UART TX is free
        ch = buffer_remove(outputbuffer);
//        UART_PutCharPolling(ch);
        cnt++;
    }
    // Reenable interrupts
    //UART0->IEN |= UART_IEN_TXC|UART_IEN_RXDATAV;
    return cnt;
}

