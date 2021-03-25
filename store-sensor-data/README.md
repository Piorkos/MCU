# Read from IMU and write to external Flash


This program reads data from gyroscope and accelometer (MPU9250) and saves it in external NOR Flash memory (W25Q128JV).
Communication:
* I2C for MCU <-> IMU 
* SPI for MCU <-> NOR Flash memory 

Additional functionality for testing:
* press BTN_1 - reads data from sensors, then sends it via UART and saves it in external flash memory
* press BTN_2 - select sample of data stored in external memory
* press BTN_3 - reads selected sample of data from stored memory and sends it via UART
