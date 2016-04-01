#ifndef LCD_H
#define LCD_H

/**
 * @file     lcd.h
 * @brief    Hardware Abstraction Layer (HAL) for GH08172T LCD display
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 * @note     LCD Driver for GH08172 LCD Display in STM32L476 Discovery board
 * @note     Use LCD interface of STM32L476VG
 *
 ******************************************************************************/


#define LCDBIT(N) (1<<(N))

#define SEGA LCDBIT(0)
#define SEGB LCDBIT(1)
#define SEGC LCDBIT(2)
#define SEGD LCDBIT(3)
#define SEGE LCDBIT(4)
#define SEGF LCDBIT(5)
#define SEGG LCDBIT(6)
#define SEGH LCDBIT(7)
#define SEGJ LCDBIT(8)
#define SEGK LCDBIT(9)
#define SEGM LCDBIT(10)
#define SEGN LCDBIT(11)
#define SEGP LCDBIT(12)
#define SEGQ LCDBIT(13)


uint32_t LCD_Init(void);
uint32_t LCD_DeInit(void);

uint32_t LCD_WriteString(char *s);
uint32_t LCD_WriteSegments(uint32_t segs[6]);
uint32_t LCD_Clear(void);
uint32_t LCD_WriteBars(uint32_t bars);
uint32_t LCD_WriteToRAM(uint32_t area[8]);


#endif
