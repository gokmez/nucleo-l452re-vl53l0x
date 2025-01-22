# NUCLEO-L452RE VL53L0X APPLICATION

DESCRIPTION:
- VL53L0X Work Mode: Continuous Mode
- VL53L0X Interrupt Mode: When distance is lower than 150 mm, it generates interrupt signal on its GPIO pin
- MCU waits on low power (sleep) mode. When VL53L0X generates interrupt signal, it wakes up, reads measurement data and goes back to sleep.

NUCLEO-L452RE Pins:
- PA10: I2C1 SDA
- PA9 : I2C1 SCL
- PA7 : VL53L0X XSHUT
- PA15: VL53L0X INTERRUPT
- PA2 : USART2 TX
- PA3 : USART2 RX
