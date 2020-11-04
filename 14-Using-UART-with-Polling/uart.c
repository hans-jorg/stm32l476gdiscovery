
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
    PinConfiguration    txpinconf;
    PinConfiguration    rxpinconf;
} UART_Info;

/**
 ** @brief List of known UARTs
 **
 **/
//@{
static UART_Info uarttab[] = {
    /* Device        txconfig        rxconfig    */
    /*              Port  Pin AF    Port  Pin AF */
    { LPUART1,   { GPIOB,11, 8 },{ GPIOB,10, 8 } },
    { USART1,    { GPIOG, 9, 7 },{ GPIOG,10, 7 } },
    { USART2,    { GPIOD, 5, 7 },{ GPIOD, 6, 7 } },
    { USART3,    { GPIOD, 8, 7 },{ GPIOD, 9, 7 } },
    { UART4,     { GPIOA, 0, 8 },{ GPIOD, 1, 8 } },
    { UART5,     { GPIOC,12, 8 },{ GPIOD, 2, 8 } }
};
static const int uarttabsize = sizeof(uarttab)/sizeof(UART_Info *);
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
 * @brief   Enable UART
 */
void UART_Enable(USART_TypeDef *uart) {

    if ( uart == LPUART1 ) {
        RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
    } else if ( uart == USART1 ) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    } else if ( uart == USART2 ) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    } else if ( uart == USART3 ) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
    } else if ( uart == UART4 ) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
    } else if ( uart == UART5 ) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;
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
 ** @brief UART Initialization
 **
 ** @note  Use defines in uart.h to configure the uart, or'ing the parameters
 **/
int
UART_Init(int uartn, uint32_t info) {
uint32_t baudrate,div,t,over;
USART_TypeDef * uart;

    uart = uarttab[uartn].device;

    // Configure pins
    ConfigurePin(&uarttab[uartn].txpinconf);
    ConfigurePin(&uarttab[uartn].rxpinconf);

    // Configuration of LPUART is after the last UART
    if( uartn == 0 ) uartn = uarttabsize;

    // Select clock source
    t = RCC->CCIPR&~BITVALUE(3,uartn*2-2);
    t |= BITVALUE(1,uartn*2-2); // SYSCLK as Clock Source
    RCC->CCIPR = t;

    // Enable Clock
    UART_Enable(uart);

    // Configure UART
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

