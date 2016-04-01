#ifndef LEDS_H
#define LEDS_H
/**
 * @file     led.h
 * @brief    Hardware Abstraction Layer (HAL) for LEDs
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 *
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

#define LED_GREEN_PIN   (8)
#define LED_RED_PIN     (2)
#define LED_BIT(N)      (1UL<<(N))
#define LED_GREEN       LED_BIT(LED_GREEN_PIN)
#define LED_RED         LED_BIT(LED_RED_PIN)
#define LED_ALL         (LED_GREEN|LED_RED)
//@}

/**
 * @brief LED API
 */
//@{
uint32_t LED_Init(uint32_t leds);
uint32_t LED_Write(uint32_t on, uint32_t off); // on and then off
uint32_t LED_Toggle(uint32_t leds);
//@}
#endif // LEDS_H
