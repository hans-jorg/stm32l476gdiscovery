#ifndef GPIO2_H
#define GPIO2_H


#include "stm32l476xx.h"
#include "system_stm32l476.h"

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
