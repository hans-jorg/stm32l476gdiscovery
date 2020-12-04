STM32L476DISCOVERY
====================

The STM32LDiscovery has a STM32L476VGT6 microcontroller and an embedded ST-LINK/V2-1 debugging interface.

The microcontroller has the following specifications:

* 1 MByte Flash memory
* 128 KByte of RAM
* I2C interfaces (3 units)
* SPI interface (3 units)
* UART interface (6 units)
* CAN interface
* 12 bit ADC
* 12 bit DAC
* LCD driver
* Touch sensing
* USB OTG Full Speed interface
* Flexible memory controller

Additionally, the board has:

* LCD with 24 segments
* LEDs (red and green)
* Pushbutton
* Joystick
* Audio DAC
* MEMS (Gyroscope/Accelerometer/Magnetometer)
* 128 MBit Quad SPI Flash Memory
* Embedded Ammeter to measure comsumption of the STM32L476 microcontroller

The board can be powered by one of the following

* ST_LINK V2-1
* USB FS Connector
* External 5 V
* CR2032 battery

### Energy supply

### Serial interface

There are two serial interface that can be used during debugging.]

* There is a serial connection between target MCU STM32L476 and the host MCU (a STM32F103CBT6).
* There is a virtual serial connection, that shares the debug connections.

The serial connection

Signal      | Connector |  Jumper   |   Pin     |  AF  |  Device       |
------------|-----------|-----------|-----------|------|---------------|
USART RX    |  JP4 RX   |  SB16     |   PD6     |  7   |   USART2      |   
USART TX    |  JP4 TX   |  SB13     |   PD5     |  7   |   USART2      |
 
