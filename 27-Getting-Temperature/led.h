#ifndef LEDS_H
#define LEDS_H


// LED on GPIO Port E
#define LED_GREEN_PIN  (8)
// LED on GPIO Port B
#define LED_RED_PIN    (2)

#define LED_BIT(N) (1UL<<(N))

#define LED_GREEN    LED_BIT(LED_GREEN_PIN)
#define LED_RED      LED_BIT(LED_RED_PIN)

#define LED_ALL      (LED_GREEN|LED_RED)

uint32_t LED_Init(uint32_t leds);
uint32_t LED_Write(uint32_t on, uint32_t off); // on and then off
uint32_t LED_Toggle(uint32_t leds);

#endif // LEDS_H
