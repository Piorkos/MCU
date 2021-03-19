/*
 * imu.cpp
 *
 *  Created on: Mar 9, 2021
 *      Author: dominikpiorkowski
 */


#include "Imu.hpp"

/** Default constructor, uses default I2C address.
 * @see MPU9250_DEFAULT_ADDRESS
 */

uint8_t buffer[14];

Imu::Imu(I2C_HandleTypeDef* hi2c)
{
	this->hi2c = hi2c;
	devAddr = 0xD0;
	for (int i=0;i<15;i++){
		buffer[i] = 0x00;
	}
	ID = 0;
}

/** Specific address constructor.
 * @param address I2C address
 * @see MPU9250_DEFAULT_ADDRESS
 * @see MPU9250_ADDRESS_AD0_LOW
 * @see MPU9250_ADDRESS_AD0_HIGH
 */
/*MPU9250::MPU9250(uint8_t address,I2C_HandleTypeDef hi2c) {
    devAddr = address;
    this->hi2c = hi2c;
}*/

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void Imu::initialize() {

    //set clock source
    setClockSource(MPU9250_CLOCK_PLL_XGYRO);
    //set gyro output data rate to 1000hz
    setGyroDLPFMode(1);
    //set gyro range to 500dps.
    setFullScaleGyroRange(MPU9250_GYRO_FS_500);
    //set accel output data rate to 1000hz
    setAccDLPFMode(1);
    //set accel range to 2g
    setFullScaleAccelRange(MPU9250_ACCEL_FS_2);
    //setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
    //set i2c bypass enable pin to true to access magnetometer and configure interrupt
    setBypassEnableAndInterrupt();
    //enable interrupt
    //enableInterrupt();
    //set mag to continuous measurement mode
    setMagContMeasMode();
}

// CONFIG register

/** Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 * @see MPU9250_RA_CONFIG
 * @see MPU9250_CFG_DLPF_CFG_BIT
 * @see MPU9250_CFG_DLPF_CFG_LENGTH
 */
uint8_t Imu::getGyroDLPFMode() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    return buffer[0];
}
/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU9250_DLPF_BW_256
 * @see MPU9250_RA_CONFIG
 * @see MPU9250_CFG_DLPF_CFG_BIT
 * @see MPU9250_CFG_DLPF_CFG_LENGTH
 */
void Imu::setGyroDLPFMode(uint8_t mode) {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    uint8_t temp = (buffer[0] & 0xF8);
    temp = (temp | mode);
    HAL_I2C_Mem_Write(hi2c,devAddr, MPU9250_RA_CONFIG,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU9250_GYRO_FS_250
 * @see MPU9250_RA_GYRO_CONFIG
 * @see MPU9250_GCONFIG_FS_SEL_BIT
 * @see MPU9250_GCONFIG_FS_SEL_LENGTH
 */
uint8_t Imu::getFullScaleGyroRange() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    return buffer[0];
}
/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU9250_GYRO_FS_250
 * @see MPU9250_RA_GYRO_CONFIG
 * @see MPU9250_GCONFIG_FS_SEL_BIT
 * @see MPU9250_GCONFIG_FS_SEL_LENGTH
 */
void Imu::setFullScaleGyroRange(uint8_t range) {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    uint8_t temp = (buffer[0] & 0xE7);
    temp = (temp | (range<<3));
    //set fchoice_b to 00 as well
    temp = (temp & 0xFC);
    HAL_I2C_Mem_Write(hi2c,devAddr, MPU9250_RA_GYRO_CONFIG,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
}

// ACCEL_CONFIG register

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU9250_ACCEL_FS_2
 * @see MPU9250_RA_ACCEL_CONFIG
 * @see MPU9250_ACONFIG_AFS_SEL_BIT
 * @see MPU9250_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t Imu::getFullScaleAccelRange() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    return buffer[0];
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void Imu::setFullScaleAccelRange(uint8_t range) {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    uint8_t temp = (buffer[0] & 0xE7);
    temp = (temp | (range<<3));
    HAL_I2C_Mem_Write(hi2c,devAddr, MPU9250_RA_ACCEL_CONFIG,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
}

//ACCEL_CONFIG2 register
uint8_t Imu::getAccDLPFMode() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_ACCEL_CONFIG2, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    return buffer[0];
}

void Imu::setAccDLPFMode(uint8_t bandwidth) {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_ACCEL_CONFIG2, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    uint8_t temp = (buffer[0] & 0xF8);
    temp = (temp | bandwidth);
    //set fchoice_b to 0
    temp = (temp & 0xF7);
    HAL_I2C_Mem_Write(hi2c,devAddr, MPU9250_RA_ACCEL_CONFIG2,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
}

// ACCEL_*OUT_* registers

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * FUNCTION NOT FULLY IMPLEMENTED YET.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @param mx 16-bit signed integer container for magnetometer X-axis value
 * @param my 16-bit signed integer container for magnetometer Y-axis value
 * @param mz 16-bit signed integer container for magnetometer Z-axis value
 * @see getMotion6()
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
void Imu::getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz) {

  //get accel and gyro
  getMotion6(ax, ay, az, gx, gy, gz);

  //read mag
  uint8_t temp =0x02;
  HAL_I2C_Mem_Write(hi2c,devAddr, MPU9250_RA_INT_PIN_CFG,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
  //I2Cdev::writeByte(devAddr, MPU9250_RA_INT_PIN_CFG, 0x02); //set i2c bypass enable pin to true to access magnetometer
  HAL_Delay(10);
  temp = 0x01;
  HAL_I2C_Mem_Write(hi2c,MPU9250_RA_MAG_ADDRESS, 0x0A,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
  //I2Cdev::writeByte(MPU9250_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  HAL_Delay(10);
  HAL_I2C_Mem_Read(hi2c,MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_XOUT_L,I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,6,100);
  //I2Cdev::readBytes(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_XOUT_L, 6, buffer);
  *mx = (((int16_t)buffer[1]) << 8) | buffer[0];
    *my = (((int16_t)buffer[3]) << 8) | buffer[2];
    *mz = (((int16_t)buffer[5]) << 8) | buffer[4];
}
/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
void Imu::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_ACCEL_XOUT_H,I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,14,100);
    //I2Cdev::readBytes(devAddr, MPU9250_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU9250_RA_GYRO_XOUT_H
 */
void Imu::getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_ACCEL_XOUT_H,I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,6,100);
    //I2Cdev::readBytes(devAddr, MPU9250_RA_ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
int16_t Imu::getAccelerationX() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_ACCEL_XOUT_H,I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,2,100);
  //I2Cdev::readBytes(devAddr, MPU9250_RA_ACCEL_XOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_ACCEL_YOUT_H
 */
int16_t Imu::getAccelerationY() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_ACCEL_YOUT_H,I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,2,100);
    //I2Cdev::readBytes(devAddr, MPU9250_RA_ACCEL_YOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_ACCEL_ZOUT_H
 */
int16_t Imu::getAccelerationZ() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_ACCEL_ZOUT_H,I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,2,100);
    //I2Cdev::readBytes(devAddr, MPU9250_RA_ACCEL_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// TEMP_OUT_* registers

/** Get current internal temperature.
 * @return Temperature reading in 16-bit 2's complement format
 * @see MPU9250_RA_TEMP_OUT_H
 */
int16_t Imu::getTemperature() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_TEMP_OUT_H,I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,2,100);
    //I2Cdev::readBytes(devAddr, MPU9250_RA_TEMP_OUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_XOUT_H
 */
void Imu::getRotation(int16_t* x, int16_t* y, int16_t* z) {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,6,100);
    //I2Cdev::readBytes(devAddr, MPU9250_RA_GYRO_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_XOUT_H
 */
int16_t Imu::getRotationX() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,2,100);
    //I2Cdev::readBytes(devAddr, MPU9250_RA_GYRO_XOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_YOUT_H
 */
int16_t Imu::getRotationY() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_GYRO_YOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,2,100);
    //I2Cdev::readBytes(devAddr, MPU9250_RA_GYRO_YOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_ZOUT_H
 */
int16_t Imu::getRotationZ() {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,2,100);
    //I2Cdev::readBytes(devAddr, MPU9250_RA_GYRO_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// PWR_MGMT_1 register

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_SLEEP_BIT
 */
void Imu::setSleepEnabled(bool enabled) {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    uint8_t temp = (buffer[0] & 0xBF);
    temp = (temp | (enabled<<6));
    HAL_I2C_Mem_Write(hi2c,devAddr, MPU9250_RA_PWR_MGMT_1,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
    //I2Cdev::writeBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, enabled);
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CLKSEL_BIT
 * @see MPU9250_PWR1_CLKSEL_LENGTH
 */
void Imu::setClockSource(uint8_t source) {
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    uint8_t temp = ((buffer[0]) & (0xF8));
    temp = (temp | source);
    HAL_I2C_Mem_Write(hi2c,devAddr, MPU9250_RA_PWR_MGMT_1,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
    //I2Cdev::writeBits(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, source);
}

// WHO_AM_I register
/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100, 0x34).
 * @return Device ID (6 bits only! should be 0x34)
 * @see MPU9250_RA_WHO_AM_I
 * @see MPU9250_WHO_AM_I_BIT
 * @see MPU9250_WHO_AM_I_LENGTH
 */

uint8_t Imu::getDeviceID(){
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,&ID,1,100);
    return ID;
}

void Imu::setBypassEnableAndInterrupt(){
    uint8_t temp = 0x22;
    HAL_I2C_Mem_Write(hi2c,devAddr, MPU9250_RA_INT_PIN_CFG,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
    HAL_Delay(10);
    //I2Cdev::writeByte(0x68, MPU9250_RA_INT_PIN_CFG, 0x02);
}

void Imu::enableInterrupt(){
    HAL_I2C_Mem_Read(hi2c,devAddr, MPU9250_RA_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,1,100);
    uint8_t temp = ((buffer[0]) & (0xA6)); // 0xA6 = 10100110 -> clear bits 6,4,3,0
    temp = (temp | 0x01); //set last bit to enable data ready interrupt
    HAL_I2C_Mem_Write(hi2c,devAddr, MPU9250_RA_INT_ENABLE,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
}

void Imu::setMagContMeasMode(){
    uint8_t temp = 0x06;
    HAL_I2C_Mem_Write(hi2c,MPU9250_RA_MAG_ADDRESS, 0x0A,I2C_MEMADD_SIZE_8BIT, &temp,1,100);
    //I2C_M.writeByte(MPU9250_RA_MAG_ADDRESS, 0x0A, 0x06);
}

void Imu::getMagData(int16_t* mx,int16_t* my, int16_t* mz){
    HAL_I2C_Mem_Read(hi2c,MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_XOUT_L, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,6,100);
    //I2C_M.readBytes(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_XOUT_L, 6, buffer_m);

    /*read ST2 register as required by magnetometer.Otherwise the data is protected and won't be updated.*/
    uint8_t temp;
    HAL_I2C_Mem_Read(hi2c,MPU9250_RA_MAG_ADDRESS, 0x09, I2C_MEMADD_SIZE_8BIT, &temp,1,100);
    //I2C_M.readByte(MPU9250_RA_MAG_ADDRESS, 0x09, &buffer_);

    *mx = ((int16_t)(buffer[1]) << 8) | buffer[0] ;
    *my = ((int16_t)(buffer[3]) << 8) | buffer[2] ;
    *mz = ((int16_t)(buffer[5]) << 8) | buffer[4] ;
}

uint8_t Imu::getCompassDataReady(){
   uint8_t temp;
   HAL_I2C_Mem_Read(hi2c,MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_ST1, I2C_MEMADD_SIZE_8BIT, &temp,1,100);
   //I2C_M.readByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_ST1, &buffer_);
   temp = (temp & 0x01);//remove the front 7 bits.
   return temp;
}

void Imu::readIntStatus(){
   HAL_I2C_Mem_Read(hi2c,devAddr,MPU9250_RA_DMP_INT_STATUS,I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer,6,100);
}

