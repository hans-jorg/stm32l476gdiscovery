30 Quadrature version 3
=======================

Introduction
------------

There are two previous project that deals with quadrature signals. Both use a polling approach, that is CPU intensive and not adequate for use on larger projects.

Basically, a quadrature encoder generates two signals: A and B. Sometimes, there is an additional signal, index, the is linked to an absolute position. The direction can be detected by finding which signal (A or B) leads. The position can be found by counting the number of transitions, considering one direction as positive and other as negative.

               _________        _________
    A _________|       |________|       |____________
                   _________        ___________
    B______________|        |_______|         |__________
                            Figure 1- Clockwise rotation - Incrementing
                            
                   _________        ___________
    A______________|        |_______|         |__________
               _________        _________
    B _________|       |________|       |____________
                            Figure 2- Counterclockwise rotation - Decrementing
                            

There are counting modes.

* X2: When a specific signal changes, the other is accessed and it gives the direction.
* X4: At the transitions of either signal, a signal is generated by a state machine and it gives the direction.

For a disk with N slots, when using X2, a full rotation generates 2N counting pulses. When using X4, it generates 4N counting pulses.

The Full-Featured Timers, the Advanced-Control Timers and the Low-Power Timers on the STM32L476 can decode quadrature signals, because they have encoder modes. The 

The full featured timers are TIM2, TIM3, TIM4 and TIM5. TIM2 and TIM5 have 32-bit counters and TIM3 and TIM4, 16-bit counters.
The Advanced-Control Timers are TIM1 and TIM8.

LPTIM1 (but not LPTIM2) has an encoder mode.


The following pins can be used for A and B signals.

| Signal     |  Type    | Size  | Port/Pin  | Port/Pin  | Port/Pin  | Port/Pin  |
|------------|----------|-------|-----------|-----------|-----------|-----------|
| TIM1_CH1   | Advanced |   16  | PA8(AF1)  | PE9(AF1)  |           |           |
| TIM1_CH2   | Advanced |   16  | PA9(AF1)  | PE11(AF1) |           |           |
| TIM2_CH1   | Full     |   32  | PA0(AF1)  | PA5(AF1)  | PA15(AF1) |           |
| TIM2_CH2   | Full     |   32  | PA2(AF1)  | PB3(AF1)  |           |           |
| TIM3_CH1   | Full     |   16  | PA6(AF2)  | PB4(AF2)  | PC6(AF2)  | PE3(AF2)  |
| TIM3_CH2   | Full     |   16  | PA7(AF2)  | PB5(AF2)  | PC7(AF2)  | PE4(AF2)  |
| TIM4_CH1   | Full     |   16  | PB6(AF2)  | PD12(AF2) |           |           |
| TIM4_CH2   | Full     |   16  | PB7(AF2)  | PD13(AF2) |           |           |
| TIM5_CH1   | Full     |   32  | PA0(AF2)  | PF6(AF2)  |           |           |
| TIM5_CH2   | Full     |   32  | PA1(AF2)  | PF7(AF2)  |           |           |
| TIM8_CH1   | Advanced |   16  | PC6(AF3)  |           |           |           |
| TIM8_CH2   | Advanced |   16  | PC7(AF3)  |           |           |           |
| LPTIM1_IN1 | Low Power|   16  | PB5(AF1)  | PC0(AF1)  | PG10(AF1) |           |
| LPTIM1_IN2 | Low Power|   16  | PB7(AF1)  | PC2(AF1)  | PG11(AF1) |           |


The Full-Featured Timers and the Advanced-Timers share the same register layout, with minor differences. The Low Power Timer has a completely different register layout.


### Full-Featured Timers

The A and B signals must be connected to the CH1 and CH2 inputs of the timer. The pre-scaler must be set to one. 

There are three modes that can be used.

* mode1: X2 resolution on TI2 transitions
* mode2: X2 resolution on TI1 transitions
* mode3: X4 resolution on TI1 and TI2


To configure one of them, these steps must be followed.

1. Configure pin
2. Select and configure timer input.
2. Select the encoder mode.
3. Enable the timer counter.




To configure the timer TIMx (x=2,3,4,5),


    Set TIMx_CCMR1.CC1S = 0b001
    Set TIMx_CCMR1.CC2S = 0b001

    Set TIMx_CCER.CC1P = 0
    Set TIMx_CCER.CC1NP = 0
    Set TIMx_CCER.CC2P = 0
    Set TIMx_CCER.CC2NP = 0

    Set TIMx_SMCR.SMS = 0b0001 for X2 on TI2
    Set TIMx_SMCR.SMS = 0b0010 for X2 on TI1
    Set TIMx_SMCR.SMS = 0b0011 for X4 on TI1 and TI2
    
    Set TIMx.ARR =-

    Set TIMx_CR1.CEN = 1

To get the counting value, one must access TIMx_CNT register.

### Advanced Control Timers

The one-pulse mode can be used. 




To configure the timer TIMx (x=1,8),


    Set TIMx_CCMR1.CC1S = 01
    Set TIMx_CCMR1.CC2S = 01

    Set TIMx_CCER.CC1P = 0
    Set TIMx_CCER.CC1NP = 0
    Set TIMx_CCER.CC2P = 0
    Set TIMx_CCER.CC2NP = 0

    Set TIMx_SMCR.SMS = 0001 for X2 on TI2
    Set TIMx_SMCR.SMS = 0011 for X2 on TI1
    Set TIMx_SMCR.SMS = 0011 for X4 on TI1 and TI2

    Set TIMx_CR1.CEN = 1

To get the counting value, one must access TIMx_CNT register.

### Low Power Timer


STM32L476 Discovery Board
-------------------------

The MCU pins have multiple functions and most of the pins in the table above are used for other functions.

The PB6 and PB7 pins are on the P1 header, labeled as I2C1_SCL and I2C1_SDA.
The other pins conflict with either the LEDs or the Joystick.

References
----------

1. [STM32L476 Datasheet](https://www.st.com/resource/en/datasheet/stm32l476je.pdf)
2. [RM0351 Reference manual - STM32L47xxx, STM32L48xxx, STM32L49xxx and STM32L4Axxx
advanced Arm ® -based 32-bit MCUs](https://www.st.com/resource/en/reference_manual/rm0351-stm32l47xxx-stm32l48xxx-stm32l49xxx-and-stm32l4axxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
2. [AN4013 - STM32 cross-series timer overview (Application note)](https://www.st.com/resource/en/application_note/an4013-stm32-crossseries-timer-overview-stmicroelectronics.pdf)