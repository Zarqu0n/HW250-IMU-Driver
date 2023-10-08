# HW250-IMU-Driver
HW250/GY87 IMU Driver. For QMC5883L magnetometer.

Using MPU6050 Library
https://github.com/rfetick/MPU6050_light

Using BMP085 Library
https://github.com/adafruit/Adafruit-BMP085-Library

Using QMC5883L Library
https://github.com/mprograms/QMC5883LCompass

If you using MPU6050 Library's and test this IMU, you see magnetometer's output is constant value. So in this situation, yo need to  set `mpu.setI2CBypassEnabled(true)` for enable magnetometer I2C port.
