/* MPU6050_light library for Arduino
 * 
 * Authors: Romain JL. Fétick (github.com/rfetick)
 *              simplifications and corrections
 *          Tockn (github.com/tockn)
 *              initial author (v1.5.2)
 */

#include "MPU6050_light.h"
#include "Arduino.h"

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}

/* INIT and BASIC FUNCTIONS */

MPU6050::MPU6050(TwoWire &w){
  wire = &w;
  setFilterGyroCoef(DEFAULT_GYRO_COEFF);
  setGyroOffsets(0,0,0);
  setAccOffsets(0,0,0);
}

byte MPU6050::init(mpu6050_dps_t gyro_fs, mpu6050_range_t acc_range){
  // changed calling register sequence [https://github.com/rfetick/MPU6050_light/issues/1] -> thanks to augustosc
  byte status = writeData(MPU6050_PWR_MGMT_1_REGISTER, 0x01); // check only the first connection with status
  writeData(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeData(MPU6050_CONFIG_REGISTER, 0x00);
  setGyroConfig(gyro_fs);
  setAccConfig(acc_range);
  
  this->update();
  angleX = this->getAccAngleX();
  angleY = this->getAccAngleY();
  preInterval = millis(); // may cause lack of angular accuracy if begin() is much before the first update()
  return status;
}

byte MPU6050::writeData(byte reg, byte data){
  wire->beginTransmission(address);
  wire->write(reg);
  wire->write(data);
  byte status = wire->endTransmission();
  return status; // 0 if success
}

// This method is not used internaly, maybe by user...
byte MPU6050::readData(byte reg) {
  wire->beginTransmission(address);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(address,(uint8_t) 1);
  byte data =  wire->read();
  return data;
}

bool MPU6050::readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readData(reg);
    return ((value >> pos) & 1);
}

// Write register bit
void MPU6050::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readData(reg);

    if (state)
    {
        value |= (1 << pos);
    } else 
    {
        value &= ~(1 << pos);
    }

    writeData(reg, value);
}

/* SETTER */

void MPU6050::setGyroConfig(mpu6050_dps_t gyro_fs){
  setScale(gyro_fs);
  setGyroOffsets(0,0,0);
}

void MPU6050::setScale(mpu6050_dps_t scale){
    uint8_t value;

    switch (scale)
    {
	case MPU6050_SCALE_250DPS:
	    dpsPerDigit = .007633f;
	    break;
	case MPU6050_SCALE_500DPS:
	    dpsPerDigit = .015267f;
	    break;
	case MPU6050_SCALE_1000DPS:
	    dpsPerDigit = .030487f;
	    break;
	case MPU6050_SCALE_2000DPS:
	    dpsPerDigit = .060975f;
	    break;
	default:
	    break;
    }

    value = readData(MPU6050_GYRO_CONFIG_REGISTER);
    value &= 0b11100111;
    value |= (scale << 3);
    writeData(MPU6050_GYRO_CONFIG_REGISTER, value);
}


void MPU6050::setAccConfig(mpu6050_range_t acc_range){
  setRange(acc_range);
  setAccOffsets(0,0,0);
}

void MPU6050::setRange(mpu6050_range_t range)
{
    uint8_t value;

    switch (range)
    {
	case MPU6050_RANGE_2G:
	    rangePerDigit = .000061f;
	    break;
	case MPU6050_RANGE_4G:
	    rangePerDigit = .000122f;
	    break;
	case MPU6050_RANGE_8G:
	    rangePerDigit = .000244f;
	    break;
	case MPU6050_RANGE_16G:
	    rangePerDigit = .0004882f;
	    break;
	default:
	    break;
    }

    value = readData(MPU6050_ACCEL_CONFIG_REGISTER);
    value &= 0b11100111;
    value |= (range << 3);
    writeData(MPU6050_ACCEL_CONFIG_REGISTER, value);
}

mpu6050_dps_t MPU6050::getScale(void)
{
    uint8_t value;
    value = readData(MPU6050_GYRO_CONFIG_REGISTER);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_dps_t)value;
}

mpu6050_range_t MPU6050::getRange(void)
{
    uint8_t value;
    value = readData(MPU6050_ACCEL_CONFIG_REGISTER);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_range_t)value;
}

void MPU6050::setDHPFMode(mpu6050_dhpf_t dhpf)
{
    uint8_t value;
    value = readData(MPU6050_ACCEL_CONFIG_REGISTER);
    value &= 0b11111000;
    value |= dhpf;
    writeData(MPU6050_ACCEL_CONFIG_REGISTER, value);
}

void MPU6050::setDLPFMode(mpu6050_dlpf_t dlpf)
{
    uint8_t value;
    value = readData(MPU6050_REG_CONFIG);
    value &= 0b11111000;
    value |= dlpf;
    writeData(MPU6050_REG_CONFIG, value);
}

void MPU6050::setI2CBypassEnabled(bool state)
{
    return writeRegisterBit(MPU6050_REG_INT_PIN_CFG, 1, state);
}

bool MPU6050::getI2CBypassEnabled(void)
{
    return readRegisterBit(MPU6050_REG_INT_PIN_CFG, 1);
}

void MPU6050::setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::setAccOffsets(float x, float y, float z){
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void MPU6050::setFilterGyroCoef(float gyro_coeff){
  if ((gyro_coeff<0) or (gyro_coeff>1)){ gyro_coeff = DEFAULT_GYRO_COEFF; } // prevent bad gyro coeff, should throw an error...
  filterGyroCoef = gyro_coeff;
}

void MPU6050::setFilterAccCoef(float acc_coeff){
  setFilterGyroCoef(1.0-acc_coeff);
}

/* GETTER */

Vector MPU6050::getGyroOffset(){
  return Vector(gyroXoffset,gyroYoffset,gyroZoffset);
}

Vector MPU6050::getAccOffset(){
  return Vector(accXoffset,accYoffset,accZoffset);
}

Vector MPU6050::getAccRaw(){
  return Vector(accX,accY,accZ);
}

Vector MPU6050::getGyroRaw(){
  return Vector(gyroX,gyroY,gyroZ);
}

Vector MPU6050::getGyroAngle(){
  return  Vector(angleX,angleY,angleZ);
}

/* CALC OFFSET */

void MPU6050::calcOffsets(bool is_calc_gyro, bool is_calc_acc){
  if(is_calc_gyro){ setGyroOffsets(0,0,0); }
  if(is_calc_acc){ setAccOffsets(0,0,0); }
  float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro
  
  for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
    this->fetchData();
	ag[0] += accX;
	ag[1] += accY;
	ag[2] += (accZ-1.0);
	ag[3] += gyroX;
	ag[4] += gyroY;
	ag[5] += gyroZ;
	delay(1); // wait a little bit between 2 measurements
  }
  
  if(is_calc_acc){
    accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
    accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
    accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
  }
  
  if(is_calc_gyro){
    gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
  }
}

/* UPDATE */

void MPU6050::fetchData(){
  wire->beginTransmission(address);
  wire->write(MPU6050_ACCEL_OUT_REGISTER);
  wire->endTransmission(false);
  wire->requestFrom(address,(uint8_t) 14);

  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

  for(int i=0;i<7;i++){
	rawData[i]  = wire->read() << 8;
    rawData[i] |= wire->read();
  }

  accX = ((float)rawData[0]) * rangePerDigit * G - accXoffset;
  accY = ((float)rawData[1]) * rangePerDigit * G - accYoffset;
  accZ = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) * rangePerDigit * G - accZoffset;
  temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawData[4]) * dpsPerDigit * DEG_2_RAD - gyroXoffset;
  gyroY = ((float)rawData[5]) * dpsPerDigit * DEG_2_RAD - gyroYoffset;
  gyroZ = ((float)rawData[6]) * dpsPerDigit * DEG_2_RAD - gyroZoffset;
}

void MPU6050::update(){
  // retrieve raw data
  this->fetchData();
  
  // estimate tilt angles: this is an approximation for small angles!
  float sgZ = accZ<0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
  angleAccX =   atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)); // [-180,+180] deg
  angleAccY = - atan2(accX,     sqrt(accZ*accZ + accY*accY)); // [- 90,+ 90] deg

  unsigned long Tnew = millis();
  float dt = (Tnew - preInterval) * 1e-3;
  preInterval = Tnew;

  // Correctly wrap X and Y angles (special thanks to Edgar Bonet!)
  // https://github.com/gabriel-milan/TinyMPU6050/issues/6
  angleX = wrap(filterGyroCoef*(angleAccX + wrap(angleX +     gyroX*dt - angleAccX,M_PI)) + (1.0-filterGyroCoef)*angleAccX,M_PI);
  angleY = wrap(filterGyroCoef*(angleAccY + wrap(angleY + sgZ*gyroY*dt - angleAccY, M_PI_2)) + (1.0-filterGyroCoef)*angleAccY, M_PI_2);
  angleZ += gyroZ*dt; // not wrapped

}
