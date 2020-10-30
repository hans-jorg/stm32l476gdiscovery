
/**
 * @file     lcd.c
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

#include <stdint.h>
#include "stm32l476xx.h"
#include "system_stm32l476.h"
#include "lcd.h"

/**
 ** Structure to store segment encoding of characters
 **
 ** The LCD display has a 24 bit segment pins and a 4 common pin
 **
 ** To show a character in display, it is necessary to define which segments
 **    must be on on every phase (a common pin active)
 **
 **
 ** Segment pins assignment
 **
 **  Segment      Pin Display    Port MCU   Alternate Function       AF
 **   SEG0           1             PA7          LCD_SEG4             11
 **   SEG1           2             PC5          LCD_SEG23            11
 **   SEG2           3             PB1          LCD_SEG6             11
 **   SEG3           4             PB13         LCD_SEG13            11
 **   SEG4           5             PB15         LCD_SEG15            11
 **   SEG5           6             PD9          LCD_SEG29            11
 **   SEG6           7             PD11         LCD_SEG31            11
 **   SEG7           8             PD13         LCD_SEG33            11
 **   SEG8           9             PD15         LCD_SEG35            11
 **   SEG9          10             PC7          LCD_SEG25            11
 **   SEG10         11             PA15         LCD_SEG17            11
 **   SEG11         12             PB4          LCD_SEG8             11
 **   SEG12         17             PB5          LCD_SEG9             11
 **   SEG13         18             PC8          LCD_SEG26            11
 **   SEG14         19             PC6          LCD_SEG24            11
 **   SEG15         20             PD14         LCD_SEG34            11
 **   SEG16         21             PD12         LCD_SEG32            11
 **   SEG17         22             PD10         LCD_SEG30            11
 **   SEG18         23             PD8          LCD_SEG16            11
 **   SEG19         24             PB14         LCD_SEG14            11
 **   SEG20         25             PB12         LCD_SEG12            11
 **   SEG21         26             PB0          LCD_SEG5             11
 **   SEG22         27             PC4          LCD_SEG22            11
 **   SEG23         28             PA6          LCD_SEG3             11
 **
 ** Common Pins Assignment
 **
 **   Common
 **   COM0                         PA8          LCD_COM0             11
 **   COM1                         PA9          LCD_COM1             11
 **   COM2                         PA10         LCD_COM2             11
 **   COM3                         PB9          LCD_COM3             11
 **
 ** LCD Supply
 **    VLCD                        PC3 (with 1 uF capacitor)         11
 **
 **/

/**
 ** @brief MCU Segment Pins Multiplexed
 **
 ** @note 4 values (every cycle=a common pin active )
 ** @note 2 uint32_t are needed because MCU segment pins above 31 are used
 **
 **/
typedef unsigned mcusegpinmultiplexed[4][2];

/**
 ** @brief MCU Segment Pins From Segment
 **
 ** @note Indexed by segment and position
 **
 ** @parm segment from 0 to 13 (A...Q)
 **/
static const mcusegpinmultiplexed const tabmcusegfromseg[14][6];

/**
 ** @brief MCU Segment Pins From Char
 **
 ** @note Indexed by character and position and returns a structure with
 **       information about which segment must be on on a certain cycle
 **
 ** @note Character 0      : Blank
 **       Character 1..127 : ASCII
 **       Character 128    : Colon
 **       Character 129    : Bar
 **       Character 130    : Decimal position
 **
 **/
static const mcusegpinmultiplexed const tabmcusegfromchar[131][6];

#define BIT(N)                      (1UL<<(N))
#define BITMASK(M,N)                ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)               ((V)<<(N))
#define SETBITFIELD(VAR,MASK,VAL)   (VAR)=((VAR)&~(MASK))|(VAL)
#define SETBITFIELDMN(VAR,M,N,VAL)  (VAR)=((VAR)&~(BITMASK((M),(N)))|BITVALUE((VAL),(N)))

static void Delay(uint32_t v) {
    uint32_t cnt = v;
    while ( cnt -- ) { __NOP(); }
}


/**
 ** @brief    Set Pin to be used by LCD module
 **
 ** @note     Configure GPIO Pins to be used by LCD module
 **/

static void SetPinToLCD( GPIO_TypeDef *gpio, uint32_t bits ) {
uint32_t m = 1; // BIT(0)
uint32_t b = 0;

    while( (m != 0) && ( m <= bits ) ) {
        if( m & bits ) {
            // Configure Pin b to alternate function mode
            SETBITFIELD(gpio->MODER,BITVALUE(3,b*2),BITVALUE(2,b*2));
            // Configure Pin b to Alternate Function 11 (LCD)
            if( b < 8 ) { // Pins 0 to 7
                SETBITFIELD(gpio->AFR[0],BITVALUE(15,b*4),BITVALUE(11,b*4));
            }  else {     // Pins 8 to 15
                SETBITFIELD(gpio->AFR[1],BITVALUE(15,(b-8)*4),BITVALUE(11,(b-8)*4));
            }
            // Configure pin to high speed
            SETBITFIELD(gpio->OSPEEDR,BITVALUE(3,b*2),BITVALUE(3,b*2));
            // Configure pin without pull-up nor pull-down
            SETBITFIELD(gpio->PUPDR,BITVALUE(3,b*2),BITVALUE(0,b*2));

        }
        m <<= 1;
        b++;
    }

}

/**
 ** @brief  LCD Init
 **
 ** @note Initialize LCD to use 24 segments and 4 controls (1/4 duty cycle)
 **
 **  LSI is used as clock
 **  VLCD Pin must be connected to a 1 uF capacitor!!!
 **  1/2 bias
 **
 **
 **  Segments are mapped as above
 **
 **                ( LCDCLK_freq      )      1
 **  Frame_freq = -------------------- * ---------
 **                2^PS * ( 16 + DIV )     Duty#
 **
 **
 **  LCDCLK = LSE => LCDCLK_freq 32768 (Same as RTC)
 **  LCD_FCR -
 **      PS(3:0)   : prescaler
 **      DIV(3:0)  : divisor
 **
 **  For LSE = 32768 (using external source it can be as high as 1 MHz)
 **      PS = 1
 **      DIV = 31
 **      Frame rate = 33.03
 **
 **
 **/
#define TIMEOUT_LIM 5000

uint32_t
LCD_Init(void) {
uint32_t timeout_cnt;
uint32_t t;

    /*
     * Configure LSE as clock (= 32768 KHz)
     */

    /*
     * Enable Back Domain
     */
     /* Following instruction in Section 5.4.29 of RM035 */
    // Enable Power Interface Clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;

    // Disable Backup Domain Write Protection
    PWR->CR1 |= PWR_CR1_DBP;

    timeout_cnt = TIMEOUT_LIM;
    while( timeout_cnt&((PWR->CR1&PWR_CR1_DBP)==0) ) timeout_cnt--;
    if( timeout_cnt == 0 )  return 2; // error

#if 0
    // Reset Backup Domain
    RCC->BDCR |= RCC_BDCR_BDRST;
    Delay(100);
    RCC->BDCR &= ~RCC_BDCR_BDRST;
#endif

    // Reset LSE
    RCC->BDCR &= ~(RCC_BDCR_LSEON|RCC_BDCR_LSEBYP);

    // Wait until not ready
    timeout_cnt = TIMEOUT_LIM;
    while( timeout_cnt&((RCC->BDCR&RCC_BDCR_LSERDY)!=0) ) timeout_cnt--;
    if( timeout_cnt == 0 )  return 3; // error

    // Enable LSE
    RCC->BDCR |= RCC_BDCR_LSEON;

    // Wait until ready
    timeout_cnt = TIMEOUT_LIM;
    while( timeout_cnt&((RCC->BDCR&RCC_BDCR_LSERDY)==0) ) timeout_cnt--;
    if( timeout_cnt == 0 )  return 4; // error

/////
    // LCD use the same clock as RTC
    // But RTC clock can only be modified after a reset
    // but save and restore configuration
#if 0
    t = RCC->BDCR&~RCC_BDCR_RTCSEL;
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    RCC->BDCR = t;

    // Wait until LSE ready again
    timeout_cnt = TIMEOUT_LIM;
    while( timeout_cnt&((RCC->BDCR&RCC_BDCR_LSERDY)==0) ) timeout_cnt--;
    if( timeout_cnt == 0 )  return 6; // error
#endif
    SETBITFIELD(RCC->BDCR,RCC_BDCR_RTCSEL,BITVALUE(1,8)); // Set clock source to LSE
    RCC->BDCR |= RCC_BDCR_RTCEN; // Enable RTC Clock (just in case)

    __DSB();


    /*
     * Following flowchart in RM0351, Section 22.3.8
     *
     * 1) Enable GPIO Port Clocks
     * 2) Configure LCD Pins as alternate functions
     * 3) Configure LCD controller according to the display
     * 4) Load initial data into RAM
     * 5) Set UDR bit in LCD_SR
     * 6) Program the desired frame rate (PS, DIV) in LCD_FCR
     * 7) Program the constrast (CC) in LCD_FCR
     * 8) Enable LCDEN in LCD_SR
     */


    /*
     * Enable Clock for GPIOs A, B, C and D
     */
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOBEN|RCC_AHB2ENR_GPIOCEN
                     |RCC_AHB2ENR_GPIODEN);

    /*
     * Configure Pin VLCD to be the power supply
     * VLCD Pin must be connected to a 1 uF capacitor!!!
     *
     * Following instructions in RM0351, 22.3.5
     * 1) Configure the VLCD pin PC3 as alternate function LCD in the GPIO_AFR register.
     * 2) Wait for the external capacitor C EXT to be charged
     *   (C EXT connected to the VLCD pin,approximately 2 ms for C EXT = 1 μF)
     * 3) Set voltage source to internal source by resetting VSEL bit in the LCD_CR register
     * 4) Enable the LCD controller by setting LCDEN bit in the LCD_CR register (NOT NOW)
     */

    SetPinToLCD(GPIOC,BIT(3));

    Delay(100000);

    LCD->CR &= ~LCD_CR_VSEL; // Set to internal source

    // Verify status of step up converter
    timeout_cnt = TIMEOUT_LIM;
    while( timeout_cnt & ((LCD->SR&LCD_SR_RDY)==0) ) timeout_cnt--;
    if( timeout_cnt == 0 ) return 8;

    /*
     * GPIO Port A
     * Pins 6 (LCD_SEG23), 7 (LCD_SEG4),  8 (COM0), 9 (COM1)
     *     10 (COM2),     15 (LCD_SEG17)
     */
    SetPinToLCD( GPIOA,(BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(15)) );


    /*
     * GPIO Port B
     * Pins 0 (LCD_SEG5),  1 (LCD_SEG6),   4 (LCD_SEG8),   5 (LCD_SEG9),
     *      9 (COM3),     12 (LCD_SEG12), 13 (LCD_SEG13), 14 (LCD_SEG14),
     *     15 (LCD_SEG15)
     */
    SetPinToLCD( GPIOB,(BIT(0)|BIT(1)|BIT(4)|BIT(5)|BIT(9)|BIT(12)|BIT(13)
                       |BIT(14)|BIT(15)) );

    /*
     * GPIO Port C
     * Pins 3 (VLCD),      4 (LCD_SEG22), 5 (LCD_SEG23), 6 (LCD_SEG24)
     *      7 (LCD_SEG25), 8 (LCD_SEG26)
     *
     * Pin 3 already configured above as VLCD
     */
    SetPinToLCD( GPIOC,(BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)) );

    /*
     * GPIO Port D
     * Pins 8 (LCD_SEG16),  9 (LCD_SEG29), 10 (LCD_SEG30), 11 (LCD_SEG31)
     *     12 (LCD_SEG32), 13 (LCD_SEG33), 14 (LCD_SEG34), 15 (LCD_SEG35)
     */

    SetPinToLCD( GPIOD,(BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)
                       |BIT(13)|BIT(14)|BIT(15)) );

    /*
     * Enable LCD Clock
     */
    RCC->APB1ENR1 |= RCC_APB1ENR1_LCDEN;

    Delay(10000);

    /*
     * Disable LCD
     */
    LCD->CR &= ~LCD_CR_LCDEN;

    /*
     * Initialize RAM (All segments ON)
     */

    LCD->RAM[0] = 0xFFFFFFFF; /* cycle 0 : common 0 */
    LCD->RAM[1] = 0xFF;

    LCD->RAM[2] = 0x0; /* cycle 1 : common 1 */
    LCD->RAM[3] = 0x0;

    LCD->RAM[4] = 0xFFFFFFFF; /* cycle 2 : common 2 */
    LCD->RAM[5] = 0xFF;

    LCD->RAM[6] = 0xFFFFFFFF; /* cycle 3 : common 3 */
    LCD->RAM[7] = 0xFF;

    /* Set UDR */
    LCD->CR |= LCD_SR_UDR;

    /*
     * Set Frame rate
     */
    t = LCD->FCR;
    SETBITFIELD(t,LCD_FCR_PS,BITVALUE(0,22));        // PS=0 => 2^0=1
    SETBITFIELD(t,LCD_FCR_DIV,BITVALUE(15,18));      // DIV=15 => DIV=31
    SETBITFIELD(t,LCD_FCR_BLINK,BITVALUE(0,16));     // BLINK=0
    SETBITFIELD(t,LCD_FCR_BLINKF,BITVALUE(0,13));    // BLINKF=0
    SETBITFIELD(t,LCD_FCR_CC,BITVALUE(5,10));        // CC=5
    SETBITFIELD(t,LCD_FCR_DEAD,BITVALUE(4,7));       // DEADTIME=4
    SETBITFIELD(t,LCD_FCR_PON,BITVALUE(4,4));        // PON
    t &= ~(LCD_FCR_UDDIE|LCD_FCR_SOFIE|LCD_FCR_HD);  // No High Drive and interrupts
    LCD->FCR = t;

    /*
     * Duty cycle, bias, etc (write protected when LCD_SR_ENS=1)
     */
    t = LCD->CR;
    SETBITFIELD(t,LCD_CR_BIAS,BITVALUE(2,5)); // 2=>1/3
    SETBITFIELD(t,LCD_CR_DUTY,BITVALUE(3,2)); // 3=>1/4
    t &= ~LCD_CR_VSEL;
    LCD->CR = t;


/*
 * Finally enable LCD
 */
    LCD->CR |= LCD_CR_LCDEN;

    timeout_cnt = TIMEOUT_LIM;
    while( timeout_cnt & ((LCD->SR&LCD_SR_ENS)==0) ) timeout_cnt--;
    if( timeout_cnt == 0 ) return 10;

    timeout_cnt = TIMEOUT_LIM;
    while( timeout_cnt & ((LCD->SR&LCD_SR_RDY)==0) ) timeout_cnt--;
    if( timeout_cnt == 0 ) return 12;

    return 0;
}

/*
 * LCD Disable
 */
uint32_t
LCD_DeInit(void) {
uint32_t timeout_cnt;

    LCD->CR &= ~LCD_CR_LCDEN;
    timeout_cnt = TIMEOUT_LIM;
    while( timeout_cnt & (LCD->CR&LCD_SR_ENS)) timeout_cnt--;
    if( timeout_cnt == 0 ) return 1;

    return 0;
}

/**
 ** @brief LCD Write Buffer to RAM
 **
 ** @note Write a string to display
 **       For now, at most 6 characters
 **
 **/

uint32_t
LCD_WriteToRAM(uint32_t area[8]) {
uint32_t timeout_cnt;
int k;

    timeout_cnt = TIMEOUT_LIM;
    while(timeout_cnt & ((LCD->SR&LCD_SR_UDR)==0) ) timeout_cnt--;
    if( timeout_cnt == 0 ) return 1;

    /* Set LCD_RAM according text in s */

    for(k=0;k<8;k++)
        LCD->RAM[k] = area[k];
#if 0
    LCD->RAM[0] = 0xFFFFFFFF; /* cycle 0 : common 0 */
    LCD->RAM[1] = 0xFF;

    LCD->RAM[2] = 0xFFFFFFFF; /* cycle 1 : common 1 */
    LCD->RAM[3] = 0xFF;

    LCD->RAM[4] = 0xFFFFFFFF; /* cycle 2 : common 2 */
    LCD->RAM[5] = 0xFF;

    LCD->RAM[6] = 0xFFFFFFFF; /* cycle 3 : common 3 */
    LCD->RAM[7] = 0xFF;
#endif

    /* Set UDR */
    LCD->SR |= LCD_SR_UDR;
    return 0;
}

/**
 ** @brief LCD Write String
 **
 ** @note Write a string to LCD Display
 **       Only 6 characters
 **
 **/
uint32_t
LCD_WriteString(char *s) {
int i,k,p;
char ch;
uint32_t area[8];

    for(i=0;i<8;i++) area[i] = 0;

    for(p=0;(p<6)&&(s[p]);p++) {
        ch = s[p];
        /* Use ch as index to fill area */
        for(k=0;k<4;k++) {
            area[2*k  ] |= tabmcusegfromchar[(int)ch][p][k][0];
            area[2*k+1] |= tabmcusegfromchar[(int)ch][p][k][1];
        }
    }
    /* Transfer area to LCD RAM */
    LCD_WriteToRAM(area);

    return 0;
}
/**
 ** @brief LCD Write Segments to LCD
 **
 ** @note Write Segmentos to LCD Display
 **
 ** @parm segs is an 6 position array on uint32_t
 **       Every element defines which segment will be on in the corresponding
 **       position
 **
 **/
uint32_t
LCD_WriteSegments(uint32_t segs[6]) {
uint32_t area[8];
int i,k,p,b;
uint32_t s,m;

    for(i=0;i<8;i++) area[i] = 0;

    for(p=0;(p<6)&&(segs[p]);p++) {
        s = segs[p];
        m = 1;
        b = 0;
        while( m ) {
            if( s&m ) {
                for(k=0;k<4;k++) {
                    area[2*k  ] |= tabmcusegfromseg[b][p][k][0];
                    area[2*k+1] |= tabmcusegfromseg[b][p][k][1];
                }
            }
            m <<= 1;
            b++;
        }
    }
    /* Transfer area to LCD RAM */
    LCD_WriteToRAM(area);

    return 0;
}

/**
 ** @brief LCD Clear
 **
 ** @note  Clear display
 **
 **/
uint32_t
LCD_Clear(void) {
uint32_t area[8];
int i;


    for(i=0;i<8;i++) area[i] = 0;

    /* Transfer area to LCD RAM */
    LCD_WriteToRAM(area);

    return 0;

}

/**
 ** @brief LCD Clear
 **
 ** @note  Clear display
 **
 **/

/*
 * LCD Write Bars
 */
uint32_t
LCD_WriteBars(uint32_t bars) {


    return 0;
}


//////////////////////////////////////////////////////////////////////////////

static const mcusegpinmultiplexed const tabmcusegfromchar[][6] = {
    { /* Caracter   ('\x00')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0xC0000000, 0x00000003 },
        { 0x50000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x03000000, 0x0000000C },
        { 0x00001000, 0x00000004 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x01')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00800000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00002000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x20000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x00000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x02000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x02')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00400010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00001040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x10008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x80000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x01000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x00020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x03')*/
        /* Pos 0 :  */ {
        { 0x00C00000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x30000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000003 },
        { 0x80000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000300, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x04')*/
        /* Pos 0 :  */ {
        { 0x00C00008, 0x00000000 },
        { 0x00800008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003020, 0x00000000 },
        { 0x00002020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x30004000, 0x00000000 },
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000003 },
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x00000004 },
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000300, 0x00000000 },
        { 0x04000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x05')*/
        /* Pos 0 :  */ {
        { 0x00000008, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000020, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00004000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000000 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000004 },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x04000000, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x06')*/
        /* Pos 0 :  */ {
        { 0x00800018, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002060, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x2000C000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000002 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x0000000C },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020100, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x07')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00C00000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00003000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x30000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x00000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x03000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00000300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x08')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x09')*/
        /* Pos 0 :  */ {
        { 0x00800018, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002060, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x2000C000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000002 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x0000000C },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020100, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x0A')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00C00008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00003020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x30004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x40000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x03000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04000300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x0B')*/
        /* Pos 0 :  */ {
        { 0x00C00000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x30000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000003 },
        { 0x80000000, 0x00000003 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000300, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x0C')*/
        /* Pos 0 :  */ {
        { 0x00000010, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000040, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00008000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000000 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000008 },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020000, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x0D')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x80000000, 0x00000003 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x0E')*/
        /* Pos 0 :  */ {
        { 0x00800018, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002060, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x2000C000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000002 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x0000000C },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020100, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x0F')*/
        /* Pos 0 :  */ {
        { 0x00000018, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000060, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x0000C000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000000 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x0000000C },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020000, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x10')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x11')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x12')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x13')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x14')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x15')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x16')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x17')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x18')*/
        /* Pos 0 :  */ {
        { 0x00800008, 0x00000000 },
        { 0x00400010, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002020, 0x00000000 },
        { 0x00001040, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x20004000, 0x00000000 },
        { 0x10008000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000002 },
        { 0x80000000, 0x00000001 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000004 },
        { 0x01000000, 0x00000008 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000100, 0x00000000 },
        { 0x00020200, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x19')*/
        /* Pos 0 :  */ {
        { 0x00800008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000210, 0x00000000 },
        { 0x04000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000040, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000001 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x90000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000008 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00420000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        }
    },
    { /* Caracter   ('\x1A')*/
        /* Pos 0 :  */ {
        { 0x00800018, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002060, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x2000C000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000002 },
        { 0xC0000000, 0x00000001 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x0000000C },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020100, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x1B')*/
        /* Pos 0 :  */ {
        { 0x00C00008, 0x00000000 },
        { 0x00400008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003020, 0x00000000 },
        { 0x00001020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x30004000, 0x00000000 },
        { 0x10004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000003 },
        { 0x40000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x00000004 },
        { 0x01000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000300, 0x00000000 },
        { 0x04000200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x1C')*/
        /* Pos 0 :  */ {
        { 0x00000010, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x04000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000040, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00008000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x00000004 },
        { 0x00001000, 0x00000000 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        }
    },
    { /* Caracter   ('\x1D')*/
        /* Pos 0 :  */ {
        { 0x00800008, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002020, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x20004000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000002 },
        { 0x80000000, 0x00000000 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000100, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x1E')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000018, 0x00000000 },
        { 0x00000210, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000060, 0x00000000 },
        { 0x01000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x0000C000, 0x00000000 },
        { 0x00008000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0xC0000000, 0x00000000 },
        { 0x90000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x0000000C },
        { 0x00001000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04020000, 0x00000000 },
        { 0x00420000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x1F')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000210, 0x00000000 },
        { 0x04000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000040, 0x00000000 },
        { 0x01000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000001 },
        { 0x40000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x90000000, 0x00000000 },
        { 0x10004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000008 },
        { 0x00001020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00420000, 0x00000000 },
        { 0x00400008, 0x00000000 }
        }
    },
    { /* Caracter   ('\x20')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter ! ('\x21')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00800000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00002000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x20000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x00000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x02000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter " ('\x22')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter # ('\x23')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000218, 0x00000000 },
        { 0x04000210, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x01000060, 0x00000000 },
        { 0x01000040, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x0000C000, 0x00000001 },
        { 0x40008000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0xC0000000, 0x00000003 },
        { 0xD0000000, 0x00000000 },
        { 0x90004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x03000000, 0x0000000C },
        { 0x00001000, 0x0000000C },
        { 0x00001020, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x04420000, 0x00000000 },
        { 0x00420008, 0x00000000 }
        }
    },
    { /* Caracter $ ('\x24')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x80000000, 0x00000003 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter % ('\x25')*/
        /* Pos 0 :  */ {
        { 0x00800008, 0x00000000 },
        { 0x00800000, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x04000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002020, 0x00000000 },
        { 0x00002000, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000040, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x20004000, 0x00000000 },
        { 0x20000000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x40008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000002 },
        { 0x50000000, 0x00000000 },
        { 0x80004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000004 },
        { 0x02000000, 0x00000000 },
        { 0x00001000, 0x00000004 },
        { 0x00000020, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x04000100, 0x00000000 },
        { 0x00000100, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00020008, 0x00000000 }
        }
    },
    { /* Caracter & ('\x26')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x04000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000040, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x40008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000003 },
        { 0x50000000, 0x00000000 },
        { 0x80004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00001000, 0x00000004 },
        { 0x00000020, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00020008, 0x00000000 }
        }
    },
    { /* Caracter ' ('\x27')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter ( ('\x28')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter ) ('\x29')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x04000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000004 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        }
    },
    { /* Caracter * ('\x2A')*/
        /* Pos 0 :  */ {
        { 0x00800008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000218, 0x00000000 },
        { 0x04000210, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000060, 0x00000000 },
        { 0x01000040, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x0000C000, 0x00000001 },
        { 0x40008000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0xD0000000, 0x00000000 },
        { 0x90004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x0000000C },
        { 0x00001020, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x04000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04420000, 0x00000000 },
        { 0x00420008, 0x00000000 }
        }
    },
    { /* Caracter + ('\x2B')*/
        /* Pos 0 :  */ {
        { 0x00800008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter , ('\x2C')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000004 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter - ('\x2D')*/
        /* Pos 0 :  */ {
        { 0x00800008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter . ('\x2E')*/
        /* Pos 0 :  */ {
        { 0x00000018, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000018, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000060, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x00000060, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x0000C000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x0000C000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000000 },
        { 0x80000000, 0x00000000 },
        { 0xC0000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x0000000C },
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x0000000C },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020000, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x04020000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter / ('\x2F')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x50000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000004 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter 0 ('\x30')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0xC0000000, 0x00000003 },
        { 0x50000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x03000000, 0x0000000C },
        { 0x00001000, 0x00000004 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter 1 ('\x31')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00800000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00002000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x20000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x00000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x02000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter 2 ('\x32')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00400010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00001040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x10008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x80000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x01000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x00020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter 3 ('\x33')*/
        /* Pos 0 :  */ {
        { 0x00C00000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x30000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000003 },
        { 0x80000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000300, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter 4 ('\x34')*/
        /* Pos 0 :  */ {
        { 0x00C00008, 0x00000000 },
        { 0x00800008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003020, 0x00000000 },
        { 0x00002020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x30004000, 0x00000000 },
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000003 },
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x00000004 },
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000300, 0x00000000 },
        { 0x04000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter 5 ('\x35')*/
        /* Pos 0 :  */ {
        { 0x00000008, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000020, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00004000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000000 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000004 },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x04000000, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter 6 ('\x36')*/
        /* Pos 0 :  */ {
        { 0x00800018, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002060, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x2000C000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000002 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x0000000C },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020100, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter 7 ('\x37')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00C00000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00003000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x30000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x00000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x03000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00000300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter 8 ('\x38')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter 9 ('\x39')*/
        /* Pos 0 :  */ {
        { 0x00800018, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002060, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x2000C000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000002 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x0000000C },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020100, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter : ('\x3A')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter ; ('\x3B')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000004 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter < ('\x3C')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter = ('\x3D')*/
        /* Pos 0 :  */ {
        { 0x00800008, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002020, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x20004000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000002 },
        { 0x80000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000100, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter > ('\x3E')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x04000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000004 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        }
    },
    { /* Caracter ? ('\x3F')*/
        /* Pos 0 :  */ {
        { 0x00C00000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003000, 0x00000000 },
        { 0x00001000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x30000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000003 },
        { 0x00000000, 0x00000001 },
        { 0x80000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000300, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter @ ('\x40')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter A ('\x41')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00C00008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00003020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x30004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x40000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x03000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04000300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter B ('\x42')*/
        /* Pos 0 :  */ {
        { 0x00C00000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x30000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000003 },
        { 0x80000000, 0x00000003 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000300, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter C ('\x43')*/
        /* Pos 0 :  */ {
        { 0x00000010, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000040, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00008000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000000 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000008 },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020000, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter D ('\x44')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x80000000, 0x00000003 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter E ('\x45')*/
        /* Pos 0 :  */ {
        { 0x00800018, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002060, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x2000C000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000002 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x0000000C },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020100, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter F ('\x46')*/
        /* Pos 0 :  */ {
        { 0x00000018, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000060, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x0000C000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000000 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x0000000C },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020000, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter G ('\x47')*/
        /* Pos 0 :  */ {
        { 0x00800010, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002040, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x20008000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000002 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000008 },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020100, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter H ('\x48')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00800008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00002020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter I ('\x49')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter J ('\x4A')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00800010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00002040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x20008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0x80000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x02000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x00020100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter K ('\x4B')*/
        /* Pos 0 :  */ {
        { 0x00000018, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000060, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x0000C000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x0000000C },
        { 0x00000000, 0x00000004 },
        { 0x00001000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x04020000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter L ('\x4C')*/
        /* Pos 0 :  */ {
        { 0x00000010, 0x00000000 },
        { 0x00000018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000040, 0x00000000 },
        { 0x00000060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00008000, 0x00000000 },
        { 0x0000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000000 },
        { 0xC0000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020000, 0x00000000 },
        { 0x04020000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter M ('\x4D')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00800008, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x04000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00002020, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0x40000000, 0x00000002 },
        { 0x10000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x02000000, 0x00000004 },
        { 0x00001000, 0x00000000 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04000100, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        }
    },
    { /* Caracter N ('\x4E')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00800008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00002020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x40008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x80004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020008, 0x00000000 }
        }
    },
    { /* Caracter O ('\x4F')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter P ('\x50')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00400008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00001020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x10004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x40000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x01000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04000200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter Q ('\x51')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter R ('\x52')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00400008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00001020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x10004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x40000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x01000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04000200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter S ('\x53')*/
        /* Pos 0 :  */ {
        { 0x00800008, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002020, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x20004000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000002 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000004 },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000100, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter T ('\x54')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter U ('\x55')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter V ('\x56')*/
        /* Pos 0 :  */ {
        { 0x00000010, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000040, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00008000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x50000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x00000004 },
        { 0x00001000, 0x00000004 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter W ('\x57')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00800008, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00002020, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x20004000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0x40000000, 0x00000002 },
        { 0x40000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000004 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04000100, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter X ('\x58')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x04000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000040, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x40008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x50000000, 0x00000000 },
        { 0x80004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000004 },
        { 0x00000020, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00020008, 0x00000000 }
        }
    },
    { /* Caracter Y ('\x59')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000210, 0x00000000 },
        { 0x04000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000040, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000001 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x90000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000008 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00420000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        }
    },
    { /* Caracter Z ('\x5A')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00400010, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00001040, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x10008000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000001 },
        { 0x50000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000008 },
        { 0x00001000, 0x00000004 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00020200, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter [ ('\x5B')*/
        /* Pos 0 :  */ {
        { 0x00000010, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000040, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00008000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000000 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000008 },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020000, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter \ ('\x5C')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x40008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x80004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020008, 0x00000000 }
        }
    },
    { /* Caracter ] ('\x5D')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x80000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter ^ ('\x5E')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00001000, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x00000000, 0x00000001 },
        { 0x50000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00001000, 0x00000004 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter _ ('\x5F')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter ` ('\x60')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        }
    },
    { /* Caracter a ('\x61')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00C00008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00003020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x30004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x40000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x03000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04000300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter b ('\x62')*/
        /* Pos 0 :  */ {
        { 0x00C00000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x30000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000003 },
        { 0x80000000, 0x00000003 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000300, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter c ('\x63')*/
        /* Pos 0 :  */ {
        { 0x00000010, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000040, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00008000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000000 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000008 },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020000, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter d ('\x64')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00C00010, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00003040, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x30008000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x80000000, 0x00000003 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x03000000, 0x00000008 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00020300, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter e ('\x65')*/
        /* Pos 0 :  */ {
        { 0x00800018, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002060, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x2000C000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000002 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x0000000C },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020100, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter f ('\x66')*/
        /* Pos 0 :  */ {
        { 0x00000018, 0x00000000 },
        { 0x00400018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000060, 0x00000000 },
        { 0x00001060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x0000C000, 0x00000000 },
        { 0x1000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000000 },
        { 0xC0000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x0000000C },
        { 0x01000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020000, 0x00000000 },
        { 0x04020200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter g ('\x67')*/
        /* Pos 0 :  */ {
        { 0x00800010, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002040, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x20008000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000002 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000008 },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020100, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter h ('\x68')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00800008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00002020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter i ('\x69')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter j ('\x6A')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00800010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00002040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x20008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0x80000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x02000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x00020100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter k ('\x6B')*/
        /* Pos 0 :  */ {
        { 0x00000018, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000060, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x0000C000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x0000000C },
        { 0x00000000, 0x00000004 },
        { 0x00001000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x04020000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter l ('\x6C')*/
        /* Pos 0 :  */ {
        { 0x00000010, 0x00000000 },
        { 0x00000018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000040, 0x00000000 },
        { 0x00000060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00008000, 0x00000000 },
        { 0x0000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000000 },
        { 0xC0000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020000, 0x00000000 },
        { 0x04020000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter m ('\x6D')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00800008, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x04000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00002020, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0x40000000, 0x00000002 },
        { 0x10000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x02000000, 0x00000004 },
        { 0x00001000, 0x00000000 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04000100, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        }
    },
    { /* Caracter n ('\x6E')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00800008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00002020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x20004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x40008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0x40000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x80004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020008, 0x00000000 }
        }
    },
    { /* Caracter o ('\x6F')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter p ('\x70')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00400008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00001020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x10004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x40000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x01000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04000200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter q ('\x71')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter r ('\x72')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00400008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00001020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x10004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x40000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x01000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x04000200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter s ('\x73')*/
        /* Pos 0 :  */ {
        { 0x00800008, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002020, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x20004000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000002 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000004 },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000100, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter t ('\x74')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter u ('\x75')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter v ('\x76')*/
        /* Pos 0 :  */ {
        { 0x00000010, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000040, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00008000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x50000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x00000004 },
        { 0x00001000, 0x00000004 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter w ('\x77')*/
        /* Pos 0 :  */ {
        { 0x00400010, 0x00000000 },
        { 0x00800008, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001040, 0x00000000 },
        { 0x00002020, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10008000, 0x00000000 },
        { 0x20004000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000001 },
        { 0x40000000, 0x00000002 },
        { 0x40000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000008 },
        { 0x02000000, 0x00000004 },
        { 0x00000000, 0x00000004 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00020200, 0x00000000 },
        { 0x04000100, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter x ('\x78')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x04000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000040, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x40008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x50000000, 0x00000000 },
        { 0x80004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000004 },
        { 0x00000020, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00020008, 0x00000000 }
        }
    },
    { /* Caracter y ('\x79')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000210, 0x00000000 },
        { 0x04000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000040, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000001 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x90000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000008 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00420000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        }
    },
    { /* Caracter z ('\x7A')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00400010, 0x00000000 },
        { 0x00000208, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00001040, 0x00000000 },
        { 0x01000020, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x10008000, 0x00000000 },
        { 0x00004000, 0x00000001 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000001 },
        { 0x50000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000008 },
        { 0x00001000, 0x00000004 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00020200, 0x00000000 },
        { 0x04400000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter { ('\x7B')*/
        /* Pos 0 :  */ {
        { 0x00000008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x04000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Caracter | ('\x7C')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000200, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        }
    },
    { /* Caracter } ('\x7D')*/
        /* Pos 0 :  */ {
        { 0x00800000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x04000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 2 :  */ {
        { 0x20000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000004 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        }
    },
    { /* Caracter ~ ('\x7E')*/
        /* Pos 0 :  */ {
        { 0x00C00018, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00003060, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x3000C000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0xC0000000, 0x00000003 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x03000000, 0x0000000C },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04020300, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter   ('\x7F')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter colon ('\x80')*/
/*pos=0*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00800000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
/*pos=1*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00002000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
/*pos=2*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x20000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
/*pos=3*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000002 },
        { 0x00000000, 0x00000000 }
        },
/*pos=4*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
/*pos=5*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter bar ('\x81')*/
/*pos=0*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000100, 0x00000000 }
        },
/*pos=1*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000100, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
/*pos=2*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x02000000, 0x00000000 }
        },
/*pos=3*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x02000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
/*pos=4*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
/*pos=5*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Caracter decimal point ('\x82')*/
/*pos=0*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00800000, 0x00000000 }
        },
/*pos=1*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00002000, 0x00000000 }
        },
/*pos=2*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x20000000, 0x00000000 }
        },
/*pos=3*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000002 }
        },
/*pos=4*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
/*pos=5*/  {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    }
};
static const mcusegpinmultiplexed const tabmcusegfromseg[][6] = {
    { /* Segment A ('\x00')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Segment B ('\x01')*/
        /* Pos 0 :  */ {
        { 0x00400000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00001000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x10000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000001 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x01000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000200, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Segment C ('\x02')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00800000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00002000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x20000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x02000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Segment D ('\x03')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Segment E ('\x04')*/
        /* Pos 0 :  */ {
        { 0x00000010, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000040, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x80000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00020000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Segment F ('\x05')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Segment G ('\x06')*/
        /* Pos 0 :  */ {
        { 0x00000008, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000020, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00004000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x40000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000004 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x04000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Segment H ('\x07')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000008, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x40000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000004 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000000, 0x00000000 }
        }
    },
    { /* Segment J ('\x08')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 }
        }
    },
    { /* Segment K ('\x09')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00400000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00001000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x10000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000001 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x01000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000200, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Segment M ('\x0A')*/
        /* Pos 0 :  */ {
        { 0x00800000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00002000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x20000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000002 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x02000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000100, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Segment N ('\x0B')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 }
        }
    },
    { /* Segment P ('\x0C')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000010, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000040, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00008000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x80000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000008 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00020000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    },
    { /* Segment Q ('\x0D')*/
        /* Pos 0 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000008, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 1 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000020, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 2 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00004000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 3 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x40000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 4 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000004 },
        { 0x00000000, 0x00000000 }
        },
        /* Pos 5 :  */ {
        { 0x00000000, 0x00000000 },
        { 0x00000000, 0x00000000 },
        { 0x04000000, 0x00000000 },
        { 0x00000000, 0x00000000 }
        }
    }
};
