#ifndef UART_H
#define UART_H
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

#ifndef UART_BIT
#define UART_BIT(N) (1U<<(N))
#define UART_BITFIELD(V,P)   ((V)<<(P))
#endif

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


#define UART_BAUD           0xFFFFF00
#define UART_BAUD_150       UART_BITFIELD(150,8)
#define UART_BAUD_300       UART_BITFIELD(300,8)
#define UART_BAUD_600       UART_BITFIELD(600,8)
#define UART_BAUD_1200      UART_BITFIELD(1200,8)
#define UART_BAUD_2400      UART_BITFIELD(2400,8)
#define UART_BAUD_4800      UART_BITFIELD(4800,8)
#define UART_BAUD_9600      UART_BITFIELD(9600,8)
#define UART_BAUD_19200     UART_BITFIELD(19200,8)
#define UART_BAUD_38400     UART_BITFIELD(38400,8)
#define UART_BAUD_57600     UART_BITFIELD(57600,8)
#define UART_BAUD_115200    UART_BITFIELD(115200,8)
//@}

/**
 * @brief Id for USART/USARTs
 *
 *   Device     |   Id
 *   -----------|-----------------
 *   LPUART1    |   UART_0 or UART_LP
 *   USART1     |   UART_1
 *   USART2     |   UART_2
 *   USART3     |   UART_3
 *   UART4      |   UART_4
 *   UART5      |   UART_5
 */
//@{
#define UART_0   0
#define UART_LP  0
#define UART_1   1
#define UART_2   2
#define UART_3   3
#define UART_4   4
#define UART_5   5
//@}

/// Symbols returned by GetStatus
//@{
#define UART_TXCOMPLETE UART_BIT(6)
#define UART_RXNOTEMPTY UART_BIT(5)
#define UART_TXEMPTY    UART_BIT(7)
#define UART_RXBUSY     UART_BIT(16)
#define UART_RXFERROR   UART_BIT(1)
#define UART_RXPERROR   UART_BIT(0)
///@}

int UART_Init(int uartn, uint32_t info);
int UART_WriteChar(int uartn, uint32_t c);
int UART_WriteString(int uartn, char s[]);

int UART_ReadChar(int uartn);
int UART_ReadString(int uartn, char *s, int n);
int UART_GetStatus(int uartn);

#endif // UART_H
