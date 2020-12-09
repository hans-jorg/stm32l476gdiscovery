#ifndef GPIO2_H
#define GPIO2_H


#include "stm32l476xx.h"
#include "system_stm32l476.h"


/**
 * @brief  GPIO configuration
 *
 * MODEx Register : Mode register
 *           2 bits for each pin, MODE15..0
 *           00 : Input
 *           01 : Output
 *           10 : Alternate Function
 *           11 : Analog
 *
 * OSPEEDx Register : Output speed register
 *           2 bits for each pin, OSPEED15..0
 *           00 : Low speed
 *           01 : Medium speed
 *           10 : High speed
 *           11 : Very high speed
 *
 * PUPDRx Register : Pullup/Pulldown register
 *           2 bits for each pin, PUPD15..0
 *           00 : No pullup/pulldown
 *           01 : Pullup
 *           10 : Pulldown
 *           11 : Reserved
 *
 * ODRx Register : output data register
 *           1 bit for each pin, OD15..0
 */
///@{
#define GPIO_MODE_IN            (0)
#define GPIO_MODE_OUT           (1)
#define GPIO_MODE_ALT           (2)
#define GPIO_MODE_ANA           (3)
#define GPIO_MODE_MASK          (3)

#define GPIO_OSPEED_LOW         (0)
#define GPIO_OSPEED_MED         (1)
#define GPIO_OSPEED_HIGH        (2)
#define GPIO_OSPEED_VERYHIGH    (3)
#define GPIO_OSPEED_MASK        (3)

#define GPIO_PUPD_NONE          (0)
#define GPIO_PUPD_UP            (1)
#define GPIO_PUPD_DOWN          (2)
#define GPIO_PUPD_RES           (3)
#define GPIO_PUPD_MASK          (3)
///#}

/**
 * To use as conf parameter in GPIO_ConfigurePins
 *
 * Added 1 to permit a zero value means do not modify this parameter
 */
///@{
#define GPIO_CONF_IN           (GPIO_MODE_IN+1)
#define GPIO_CONF_OUT          (GPIO_MODE_OUT+1)
#define GPIO_CONF_ALT          (GPIO_MODE_ALT+1)
#define GPIO_CONF_ANA          (GPIO_MODE_ANA+1)
#define GPIO_CONF_LOW          ((GPIO_OSPEED_LOW+1)<<4)
#define GPIO_CONF_MED          ((GPIO_OSPEED_MED+1)<<4)
#define GPIO_CONF_HIGH         ((GPIO_OSPEED_HIGH+1)<<4)
#define GPIO_CONF_VERYHIGH     ((GPIO_OSPEED_VERYHIGH+1)<<4)
#define GPIO_CONF_NONE         ((GPIO_PUPD_NONE+1)<<8)
#define GPIO_CONF_UP           ((GPIO_PUPD_UP+1)<<8)
#define GPIO_CONF_DOWN         ((GPIO_PUPD_DOWN+1)<<8)
#define GPIO_CONF_RES          ((GPIO_PUPD_RES+1)<<8)
///#}


void     GPIO_Init(GPIO_TypeDef *gpio, uint16_t input, uint16_t output);
void     GPIO_EnableClock(GPIO_TypeDef *gpio);
void     GPIO_ConfigurePins(GPIO_TypeDef *gpio, uint16_t pins, uint32_t conf);


/*
 * Inline functions
 */


static inline void GPIO_Write(GPIO_TypeDef *gpio, uint16_t zeroes, uint16_t ones) {

#if 0
    gpio->ODR = ((gpio->ODR)&~zeroes)|ones;
#else
    gpio->BSRR = (((uint32_t) zeroes)<<16)| (uint32_t) ones;
#endif
}

static inline void GPIO_Toggle(GPIO_TypeDef *gpio, uint16_t bits) {

    gpio->ODR = (gpio->ODR)^bits;
}

static inline uint16_t GPIO_Read(GPIO_TypeDef *gpio) {

    return gpio->IDR;
}

static inline void GPIO_Set(GPIO_TypeDef *gpio, uint16_t bits) {

    gpio->BSRR = (uint32_t) bits;
}

static inline void GPIO_Clear(GPIO_TypeDef *gpio, uint16_t bits) {

    gpio->BSRR = ((uint32_t) bits)<<16;
}



#endif
