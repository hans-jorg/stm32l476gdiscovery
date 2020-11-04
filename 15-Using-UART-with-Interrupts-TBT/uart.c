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
 * @brief   Pin configuration
 */
typedef struct {
    GPIO_TypeDef   *gpio;
    int             pin;
    int             af;
} PinConfiguration;

/**
 ** @brief Info and data area for UARTS
 **/
typedef struct {
    USART_TypeDef      *device;
    int                 irqlevel;
    int                 irqn;
    char                inbuffer;
    char                outbuffer;
    PinConfiguration    txpinconf;
    PinConfiguration    rxpinconf;
} UART_Info;

/**
 ** @brief List of known UARTs
 **
 ** @note  Actually pointers to UARTs
 **/
//@{
static UART_Info uarttab[] = {
    /* Device  IRQ    IRQn       Buffer     txconfig        rxconfig   */
    /*        level                       Port  Pin AF    Port  Pin AF */
    { LPUART1,  6,  LPUART1_IRQn, 0, 0, { GPIOB,11, 8 },{ GPIOB,10, 8 } },
    { USART1,   6,  USART1_IRQn,  0, 0, { GPIOG, 9, 7 },{ GPIOG,10, 7 } },
    { USART2,   6,  USART2_IRQn,  0, 0, { GPIOD, 5, 7 },{ GPIOD, 6, 7 } },
    { USART3,   6,  USART3_IRQn,  0, 0, { GPIOD, 8, 7 },{ GPIOD, 9, 7 } },
    { UART4,    6,  UART4_IRQn,   0, 0, { GPIOA, 0, 8 },{ GPIOD, 1, 8 } },
    { UART5,    6,  UART5_IRQn,   0, 0, { GPIOC,12, 8 },{ GPIOD, 2, 8 } }
};
static const int uarttabsize = sizeof(uarttab)/sizeof(UART_Info);
//@}

/**
 * @brief   Enable GPIO
 *
 * @note    Should be in gpio.[ch]
 *
 */
void GPIO_Enable(GPIO_TypeDef *gpio) {

    if( gpio == GPIOA ) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    } else if ( gpio == GPIOB ) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    } else if ( gpio == GPIOC ) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    } else if ( gpio == GPIOD ) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
    } else if ( gpio == GPIOE ) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
    } else if ( gpio == GPIOF ) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
    } else if ( gpio == GPIOG ) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
    } else if ( gpio == GPIOH ) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
    } else {
        return;
    }
}

/**
 * @brief   Configure Pin
 */
static void ConfigurePin(PinConfiguration *conf) {
GPIO_TypeDef *gpio;
int pos;

    gpio = conf->gpio;

    // Enable clock for GPIOx, if it is not already enabled
    GPIO_Enable(gpio);

    if( conf->pin > 7 ) {
        pos = (conf->pin - 7)*4;
        gpio->AFR[1] = (gpio->AFR[1]&~(0xF<<pos))|(conf->af<<pos);
    } else {
        pos = (conf->pin)*4;
        gpio->AFR[0] = (gpio->AFR[0]&~(0xF<<pos))|(conf->af<<pos);
    }
}
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
UART_Init(int uartn, unsigned info) {
uint32_t baudrate,div,t,over;
USART_TypeDef * uart;

    uart = uarttab[uartn].device;

    // Configure pin
    ConfigurePin(&uarttab[uartn].txpinconf);
    ConfigurePin(&uarttab[uartn].rxpinconf);

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
 ** @note   It blocks until char is sent
 **/
int
UART_WriteChar(int uartn, unsigned c) {
USART_TypeDef *uart;

    // wait until buffer free
    while ( uarttab[uartn].outbuffer != 0 ) {}
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

    while( uarttab[uartn].inbuffer == 0 ) {}

    return uarttab[uartn].inbuffer;

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

    uarttab[uartn].inbuffer  = 0;
    uarttab[uartn].outbuffer = 0;

    return 0;

}

