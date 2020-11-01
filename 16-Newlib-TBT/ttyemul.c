/**
 * @file    ttyemul.c
 *
 * @note    TTY emulation for a UART
 */

#include "ttyemul.h"
#include "uart.h"

/// Backspace used in line buffered mode
#define TTY_BS          '\b'

static unsigned ttyconfig = TTY_IECHO|TTY_OCRLF|TTY_ICRLF|TTY_LINEBUFFERED;


/**
 *  #brief  UART unit to be used for all I/O
 */
#define UART_N  UART_2


/**
 *  @brief  tty_init
 */
int tty_init(int chn) {

    UART_Init(  UART_N,
                UART_NOPARITY|UART_8BITS|UART_2_STOP
               |UART_BAUD_9600|UART_OVER8);

    return 0;
}
/**
 *  @brief  tty_write
 */
int tty_write(int chn, char *ptr, int len) {
int cnt;
char ch;
int i;

    cnt = 0;
    for (i = 0; i < len; i++) {
        ch = *ptr++;
        if( (ch == '\n') && ttyconfig&TTY_OCRLF ) {
            UART_WriteChar(UART_N,'\r');
            cnt++;
        }
        UART_WriteChar(UART_N,ch);
        cnt++;
    }
    return cnt;
}

/**
 *  @brief  tty_read
 *
 *  @note   No timeout yet
 */
int tty_read(int chn, char *ptr, int len) {

    if( ttyconfig&TTY_LINEBUFFERED)
        return tty_read_lb(chn,ptr,len);
    else
        return tty_read_un(chn,ptr,len);
}

/**
 *  @brief  tty_read_un
 *  @note   unbuffered
 *  @note   No timeout yet
 */
int tty_read_un(int chn, char *ptr, int len) {
int cnt;
int ch;

    for(cnt=0;cnt < len;cnt++ ) {
        ch = UART_ReadChar(UART_N);
        if( ttyconfig&TTY_IECHO )
            UART_WriteChar(UART_N,ch);
        ptr[cnt] = ch;
    }

    return cnt;
}

/**
 *  @brief  tty_read_lb
 *  @note   line buffered
 *  @note   no timeout yet!!
 */
int tty_read_lb(int chn, char *ptr, int len) {
int cnt;
int ch;

    cnt = 0;
    UART_Flush(UART_N);
    while ( ((ch=UART_ReadChar(UART_N)) != '\n') && (ch!='\r') ) {
        if( ch == TTY_BS ) {
            if( cnt > 0 ) {
                cnt--;
                UART_WriteChar(UART_N,'\b');
                UART_WriteChar(UART_N,' ');
                UART_WriteChar(UART_N,'\b');
            }
        } else {
            if( ttyconfig&TTY_IECHO )
                UART_WriteChar(UART_N,ch);
            if( cnt < len )         // overflow characters not stored
                ptr[cnt++] = ch;
        }
    }

    if( cnt < len ) {
        ptr[cnt++] = '\n';
    }
    if(ttyconfig&TTY_ICRLF ) {
        UART_WriteChar(UART_N,'\r');
        UART_WriteChar(UART_N,'\n');
    }
    return cnt;
}

