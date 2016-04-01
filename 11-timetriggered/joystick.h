#ifndef JOYSTICK_H
#define JOYSTICK_H
/**
 * @file     joystick.h
 * @brief    Hardware Abstraction Layer (HAL) for joystick using polling
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 *
 *
 ******************************************************************************/

/**
 * @brief Joystick Symbols
 *
 * @note Joystick on GPIO Port A
 * @note BIT rename JOY_BIT to avoid name collision
 */
//@{
#define JOY_BIT(N) (1UL<<(N))
#define JOY_CENTER_PIN  (0)
#define JOY_DOWN_PIN    (5)
#define JOY_LEFT_PIN    (1)
#define JOY_RIGHT_PIN   (2)
#define JOY_UP_PIN      (3)

#define JOY_DOWN         JOY_BIT(JOY_DOWN_PIN)
#define JOY_LEFT         JOY_BIT(JOY_LEFT_PIN)
#define JOY_UP           JOY_BIT(JOY_UP_PIN)
#define JOY_RIGHT        JOY_BIT(JOY_RIGHT_PIN)
#define JOY_CENTER       JOY_BIT(JOY_CENTER_PIN)

#define JOY_ALL (JOY_DOWN|JOY_LEFT|JOY_UP|JOY_RIGHT|JOY_CENTER)
//@}
/**
 * @brief Joystick API
 *
 */
uint32_t JoyStick_Init(void);
uint32_t JoyStick_Read(void);
//@}
#endif
