PWM on a STM32
==============

*** Only PWM with general purpose timer (TIM2..5) is implemented. ***

STM32 microcontroller have many timers of various types.
The STM32L476 has 13 timers, with different features, as shown below.

| Timer  |  Type       | Channels |  Size  |  Clock   |  Obs
|--------|-------------|----------|--------|----------|------------------ 
| TIM1   | ADVANCED    |     6    |    16  | APB2     | PWM, OC, IC, QE 
| TIM2   | GENERAL     |     4    |    32  | APB1/EN1 | PWM, OC, IC, QE
| TIM3   | GENERAL     |     4    |    16  | APB1/EN1 | PWM, OC, IC, QE 
| TIM4   | GENERAL     |     4    |    16  | APB1/EN1 | PWM, OC, IC, QE
| TIM5   | GENERAL     |     4    |    32  | APB1/EN1 | PWM, OC, IC, QE 
| TIM6   | BASIC       |     4    |    16  | APB1/EN1 |
| TIM7   | BASIC       |     4    |    16  | APB1/EN1 |
| TIM8   | ADVANCED    |     6    |    16  | ABP2     | PWM, OC, IC, QE
| TIM15  | SIMPLE      |     2    |    16  | APB2     | PWM, OC, IC
| TIM16  | SIMPLE      |     1    |    16  | APB2     | PWM, OC, IC
| TIM17  | SIMPLE      |     1    |    16  | APB2     | PWM, OC, IC
| LPTIM1 | LOW POWER   |     1    |    16  | APB1/EN2 | PWM, QE
| LPTIM2 | LOW POWER   |     1    |    16  | APB1/EN2 | PWM

The symbols are:
PWM: can generate PWM signals
OC:  can compare signals and generate an output signal
IC:  can measure input signal
QE:  can decode quadrature signals

For PWM generation, output compare and input capture, a timer channel must be connected to an
external pin.

| Timer     | Channel |  Port (AF) | Port (AF) | Port (AF) | Port (AF) |
|-----------|---------|------------|-----------|-----------|-----------|
| TIM1      |     1   |  PA8  (1)  |  PE9  (1) |           |           |
| TIM1      |     2   |  PA9  (1)  |  PE11 (1) |           |           |
| TIM1      |     3   |  PA10 (1)  |  PE13 (1) |           |           |
| TIM1      |     4   |  PA11 (1)  |  PE14 (1) |           |           |
| TIM2      |     1   |  PA0  (1)  |  PA5  (1) |  PA15 (1) |           |
| TIM2      |     2   |  PA1  (1)  |  PB3  (1) |           |           |
| TIM2      |     3   |  PA2  (1)  |  PB10 (1) |           |           |
| TIM2      |     4   |  PA3  (1)  |  PB11 (1) |           |           |
| TIM3      |     1   |  PA6  (2)  |  PB4  (2) |  PC6  (2) |  PE3  (2) |
| TIM3      |     2   |  PA7  (2)  |  PB5  (2) |  PC7  (2) |  PE4  (2) |
| TIM3      |     3   |  PB0  (2)  |  PC8  (2) |  PE5  (2) |           |
| TIM3      |     4   |  PB1  (2)  |  PC9  (2) |  PE6  (2) |           |
| TIM4      |     1   |  PB6  (2)  |  PD12 (2) |           |           |
| TIM4      |     2   |  PD7  (2)  |  PD12 (2) |           |           |
| TIM4      |     3   |  PB8  (2)  |  PD14 (2) |           |           |
| TIM4      |     4   |  PB9  (2)  |  PD15 (2) |           |           |
| TIM5      |     1   |  PA0  (2)  |  PF6  (2) |           |           |
| TIM5      |     2   |  PA1  (2)  |  PF7  (2) |           |           |
| TIM5      |     3   |  PA2  (2)  |  PF8  (2) |           |           |
| TIM5      |     4   |  PA3  (2)  |  PF9  (2) |           |           |
| TIM8      |     1   |  PC6  (3)  |           |           |           |
| TIM8      |     2   |  PC7  (3)  |           |           |           |
| TIM8      |     3   |  PC8  (3)  |           |           |           |
| TIM8      |     4   |  PC9  (3)  |           |           |           |
| TIM15     |     1   |  PA2 (14)  |  PB14(14) |  PF9 (14) |  PG10(14) |
| TIM15     |     2   |  PA3 (14)  |  PB15(14) |  PF10(14) |  PG11(14) |
| TIM16     |     1   |  PA6 (14)  |  PB8 (14) |  PE0 (14) |           |
| TIM17     |     1   |  PA7 (14)  |  PB9 (14) |  PE1 (14) |           |
| LPTIM1    |     1   |  PC1  (1)  |  PB2  (1) |           |           |
| LPTIM2    |     1   |  PC1  (1)  |  PA8 (14) |  PA4 (14) |  PD13(14) | 


For PWM generation, it is necessary a running counter and a register containing a 
reference value. The counter is incremented and compared to the reference value. If it is lower
the associated output is high. If it is higher, the ouput is low. When the counter reaches a 
specified value, it is then set to zero.

In many cases, the polarity can be inverted and the counter can count backwards.

General Purpose Timers
----------------------

TIM2, TIM3, TIM4 and TIM5 are general purpose timers and can be used to implement PWM.
TIM2 and TIM5 have 32 bit counters and the others, 16 bit counters.

The most important registers to configure PWM are:

* CNT: 16-bit (or 32 bit) Counter 
* PSC: Prescaler (1-65536)
* ARR: Auto reload register/Top value register
* CR1: Configuration 
* CCRx: Reference value

In the upcouting mode, the CNT counts from 0 to the value stored in ARR and the restarts from 0.
When the CNT is less than CCRX, the output is high. When it is greater than CCRx, the output is low.
But this can be inverted by setting bit CCxP bit of CCER.

There are two PWM modes;

* Mode 1: When upcounting, channel is active as long as CNT<CCR1 else inactive
* Mode 2: When upcounting, channel is inactive as long as CNT<CCR1 else active


To set PWM mode:

* Enable clock for gpio
* Configure gpio pin to alternate function
* Enable clock for timer
* Set OCxM field in CCMRx to 0110 (mode 1) or 0111 (mode 2)
* Set OCXPE bit in CCMRx to preload
* Set ARPE bit in CR1 to auto-reload CNT
* Clear DIR bit of CR1 to upcounting
* Set polarity by clearing or setting bit CCxP of CCER.
* Enable output pin by setting CCxE bit in CCER.
* Before starting the counter, set UG bit in EGR register (forced update)







