Arduino-Due-32Bit
=================

support:  Arduino Due

• Atmel SAM3X8E ARM Cortex-M3 CPU 32-bit a 84 MHz clock, ARM core microcontroller

• MPU6050C Gyro Accelerometer //i2c 400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2

• MS561101BA Barometer //i2c

• HMC5883L Magnetometer //i2c 400kHz

Quad-X
       
pin 6 FRONTL  M1CW                     M2CCW  FRONTR pin 8

                \         / 
                  \ --- /
                   |   |
                  / --- \
                /         \ 
                
pin 9 motor_BackL  M4 CCW             M3 CW  motor_BackR  pin 7 

----------rx-----------           
 A8 = CPPM
