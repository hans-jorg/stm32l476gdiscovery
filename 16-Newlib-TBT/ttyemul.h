#ifndef TTYEMUL_H
#define TTYEMUL_H
/**
 * @file    ttyemul.h
 *
 * @note    TTY emulation layer built upon a HAL for a UART
 */

int tty_init(int chn);
int tty_write(int chn, char *ptr, int len);
int tty_read(int chn, char *ptr, int len);
int tty_read_un(int chn, char *ptr, int len);
int tty_read_lb(int chn, char *ptr, int len);

/**
 *  @brief  TTY interface
 *
 *  @brief  TTY_write and TTY_READ
 *
 */
#define TTY_ICRLF           0x0001      ///< Map CR-LF to LF at input
#define TTY_OCRLF           0x0002      ///< Map LF to CR-LF at output
#define TTY_IECHO           0x0004      ///< Echo read char
#define TTY_LINEBUFFERED    0x0010      ///< Line buffered input


#endif
