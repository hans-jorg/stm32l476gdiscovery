
/**
 ** @file     uart.h
 ** @brief    Hardware Abstraction Layer (HAL) for UART
 ** @version  V1.1
 ** @date     01/11/2020
 **
 ** @note     Direct access to registers
 ** @note     No library except CMSIS is used
 ** @note     Only asynchronous communication
 **
 **/


#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "uart.h"
#include "buffer.h"

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
 * @brief   Define buffer for USART2
 */
///@{
#define BUFFERSIZE2     50
BUFFER_DEFINE_AREA(inarea2,BUFFERSIZE2);
BUFFER_DEFINE_AREA(outarea2,BUFFERSIZE2);
///@{

/// Define Interrupt level for USARTx, UARTx, LPUSARTx
#define INT_LEVEL           6



/**
 ** @brief Info and data area for UARTS
 **/
typedef struct {
    USART_TypeDef  *device;
    struct {
        unsigned    irqlevel:8;         // interrupt level
        unsigned    irqn:8;             // interrupt number
        unsigned    buffered:1;         // use circular buffer when 1, else use a byte
        unsigned    size:10;            // maximal 1023
    } info;
    union {
        char            byte;
        buffer_t        buffer;
    } in;
    union {
        char            byte;
        buffer_t        buffer;
    } out;
} UART_Info;

/**
 ** @brief List of known UARTs
 **
 ** @note  Actually pointers to UARTs
 **/
//@{
static UART_Info uarttab[] = {
    { LPUART1,  { INT_LEVEL,  LPUART1_IRQn, 0  },
                { 0 },
                { 0 }
    },
    { USART1,   { INT_LEVEL,  USART1_IRQn,  0  },
                { 0 },
                { 0 }
    },
    { USART2,   { INT_LEVEL,  USART2_IRQn,  1, BUFFERSIZE2 },
                { .buffer = (buffer_t) inarea2 },
                { .buffer = (buffer_t) outarea2}
    },
    { USART3,   { INT_LEVEL,  USART3_IRQn,  0  },
                { 0 },
                { 0 }
    },
    { UART4,    { INT_LEVEL,  UART4_IRQn,   0  },
                { 0 },
                { 0 }
    },
    { UART5,    { INT_LEVEL,  UART5_IRQn,   0  },
                { 0 },
                { 0 }
    }
};
static const int uarttabsize = sizeof(uarttab)/sizeof(UART_Info);
//@}

/**
 * @brief   Macros to simplify coding of the interrupt routines
 *
 * @note    The processing of UART interrupts are identical except for
 *          the pointer to USART_TypeDef structure and the index in the
 *          uarttab
 *
 * @note    They use the old hack of do { ;; } while(0) to avoid problems
 *          when they are used on then clauses.
 *          There is now ; after } in the then clause. It generates an error
 */

#define PROCESS_INTERRUPT_UNBUFFERED(UART,UARTN)                            \
    do {                                                                    \
        if( (UART)->ISR & USART_ISR_RXNE  ) {                               \
            uarttab[(UARTN)].in.byte = (UART)->RDR;                         \
        }                                                                   \
        if( (UART)->ISR & USART_ISR_TXE  ) {                                \
            if ( uarttab[UARTN].out.byte ) {                                \
                (UART)->TDR = uarttab[UARTN].out.byte;                      \
                uarttab[UARTN].out.byte = 0;                                \
            }                                                               \
        }                                                                   \
        /* Clear all pending interrupts */                                  \
        (UART)->ICR = 0x00121BDF;                                           \
    } while(0)

#define PROCESS_INTERRUPT_BUFFERED(UART,UARTN)                              \
    do {                                                                    \
        if( (UART)->ISR & USART_ISR_RXNE  ) {                               \
            buffer_insert(uarttab[(UARTN)].in.buffer,(UART)->RDR);          \
        }                                                                   \
        if( (UART)->ISR & USART_ISR_TXE  ) {                                \
            if ( uarttab[UARTN].out.byte ) {                                \
                (UART)->TDR = uarttab[UARTN].out.byte;                      \
                uarttab[UARTN].out.byte = 0;                                \
            }                                                               \
        }                                                                   \
    } while(0)

#define PROCESS_INTERRUPT(UART,UARTN)                                       \
    do {                                                                    \
        if( uarttab[(UARTN)].info.buffered ) {                              \
            PROCESS_INTERRUPT_BUFFERED(LPUART1,0);                          \
        } else {                                                            \
            PROCESS_INTERRUPT_UNBUFFERED(LPUART1,0);                        \
        }                                                                   \
        /* Clear all pending interrupts */                                  \
        (UART)->ICR = 0x00121BDF;                                           \
    } while(0)

/**
 ** @brief  Interrupt routines for USART, UART and LPUART
 **/
///@{

/// IRQ Handler for LPUART1
void LPUART1_IRQHandler(void) {

    PROCESS_INTERRUPT(LPUART1,0);
}

/// IRQ Handler for USART1
void USART1_IRQHandler(void) {

    PROCESS_INTERRUPT(USART1,1);

}

/// IRQ Handler for USART2
void USART2_IRQHandler(void) {

    PROCESS_INTERRUPT(USART2,2);

}

/// IRQ Handler for USART3
void USART3_IRQHandler(void) {

    PROCESS_INTERRUPT(USART3,3);

}

/// IRQ Handler for UART4
void UART4_IRQHandler(void) {

    PROCESS_INTERRUPT(UART4,4);

}

/// IRQ Handler for UART5
void UART5_IRQHandler(void) {

    PROCESS_INTERRUPT(UART5,5);

}
///@}


/**
 ** @brief UART Initialization
 **
 ** @note  Use defines in uart.h to configure the uart, or'ing the parameters
 **/
int
UART_Init(int uartn, unsigned info) {
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
    NVIC_SetPriority(uarttab[uartn].info.irqn,uarttab[uartn].info.irqlevel);
    NVIC_ClearPendingIRQ(uarttab[uartn].info.irqn);
    NVIC_EnableIRQ(uarttab[uartn].info.irqn);

    if( uarttab[uartn].info.buffered ) {
        uarttab[uartn].in.buffer  = buffer_init(
                                uarttab[uartn].in.buffer,
                                uarttab[uartn].info.size);
        uarttab[uartn].out.buffer = buffer_init(
                                uarttab[uartn].out.buffer,
                                uarttab[uartn].info.size);
    }
    // Enable UART
    uart->CR1 |= USART_CR1_UE;
    return 0;
}

/**
 ** @brief UART Send a character
 **
 ** @note   It blocks until char is sent
 **/
int
UART_WriteChar(int uartn, unsigned c) {
USART_TypeDef *uart;

    // wait until buffer free
    while ( uarttab[uartn].out.byte != 0 ) {}
    // send it
    uart->TDR = c;

    return 1;
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

    while( uarttab[uartn].in.byte == 0 ) {}

    return uarttab[uartn].in.byte;

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

/**
 ** @brief UART Flush buffers
 **
 **/
int
UART_Flush(int uartn) {
USART_TypeDef *uart;

    uart = uarttab[uartn].device;

    uarttab[uartn].in.byte  = 0;
    uarttab[uartn].out.byte = 0;

    return 0;

}

