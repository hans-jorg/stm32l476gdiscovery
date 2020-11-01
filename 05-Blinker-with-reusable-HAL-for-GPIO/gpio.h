#ifndef GPIO_H
#define GPIO_H


#define GPIO_MKWORD(PE,PB) ((PE)<<16)|((PB)&0xFFFF)
#define GPIO_GETPORTB(X) ((X)&0xFFFF)
#define GPIO_GETPORTE(X) ((X)>>16)


void GPIO_Init(uint32_t input, uint32_t output);


static inline void GPIO_Write(uint32_t zeroes, uint32_t ones) {
uint32_t pinzeroes,pinones;

    if( GPIO_GETPORTE(zeroes|ones) ) {
        pinzeroes = GPIO_GETPORTE(zeroes);
        pinones   = GPIO_GETPORTE(ones);
        GPIOE->ODR = (GPIOE->ODR&~(pinones|pinzeroes))|pinones;
    }
    if( GPIO_GETPORTB(zeroes|ones) ) {
        pinzeroes = GPIO_GETPORTB(zeroes);
        pinones   = GPIO_GETPORTB(ones);
        GPIOE->ODR = (GPIOB->ODR&~(pinones|pinzeroes))|pinones;
    }
}

static inline void GPIO_Toggle(uint32_t pins) {
uint32_t p;

    if( GPIO_GETPORTE(pins) ) {
        p = GPIO_GETPORTE(pins);
        GPIOE->ODR ^= p;
    }
    if( GPIO_GETPORTB(pins) ) {
        p = GPIO_GETPORTB(pins);
        GPIOB->ODR ^= p;
    }
}


#endif
