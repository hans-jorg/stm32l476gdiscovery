#ifndef UART_H
#define USART_H
/**
 * @file     uart.h
 * @brief    Hardware Abstraction Layer (HAL) for UARTs
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 * @note     No support for synchronous communication
 * @note     No interrupts (for now)
 *
 ******************************************************************************/

/**
 * @brief LED Symbols
 *
 * @note Green LED is on GPIO Port E
 * @note Red LED is on GPIO Port B
 *
 * @note BIT rename LED_BIT to avoid name collision
 */
//@{

/**
 ** @brief  Parameters to configure UART
 **/
//@{
#define UART_PARITY         0x3
#define UART_NOPARITY       0x0
#define UART_EVENPARITY     0x1
#define UART_ODDPARITY      0x2

#define UART_SIZE           0xC
#define UART_8BITS          0x4
#define UART_9BITS          0x8
#define UART_7BITS          0xC

#define UART_STOP           0x70
#define UART_1_STOP         0x10
#define UART_0_5_STOP       0x20
#define UART_2_STOP         0x30
#define UART_1_5_STOP       0x40

#define UART_OVER8          0x80
#define UART_OVER16         0x00

#define UARTBITVALUE(V,P)   ((V)<<(P))
#define UART_BAUD           0xFFFFF00
#define UART_BAUD_150       UARTBITVALUE(150,8)
#define UART_BAUD_300       UARTBITVALUE(300,8)
#define UART_BAUD_600       UARTBITVALUE(600,8)
#define UART_BAUD_1200      UARTBITVALUE(1200,8)
#define UART_BAUD_2400      UARTBITVALUE(2400,8)
#define UART_BAUD_4800      UARTBITVALUE(4800,8)
#define UART_BAUD_9600      UARTBITVALUE(9600,8)
#define UART_BAUD_19200     UARTBITVALUE(19200,8)
#define UART_BAUD_38400     UARTBITVALUE(38400,8)
#define UART_BAUD_57600     UARTBITVALUE(57600,8)
#define UART_BAUD_115200    UARTBITVALUE(115200,8)
//@}

/**
 ** @brief Alias for USARTs
 **/
//@{
#define UART1   USART1
#define UART2   USART2
#define UART3   USART3
//@}

uint32_t UART_Init(USART_TypeDef *uart, uint32_t info);
uint32_t UART_WriteChar(USART_TypeDef *uart, uint32_t c);
uint32_t UART_WriteString(USART_TypeDef *uart, char s[]);

#endif // UART_H
