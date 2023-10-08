/* The register map is provided at
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 * Mapping of the different gyro and accelero configurations:
 *
 * GYRO_CONFIG_[0,1,2,3] range = +- [250, 500,1000,2000] deg/s
 *                       sensi =    [131,65.5,32.8,16.4] bit/(deg/s)
 *
 * ACC_CONFIG_[0,1,2,3] range = +- [    2,   4,   8,  16] times the gravity (9.81 m/s^2)
 *                      sensi =    [16384,8192,4096,2048] bit/gravity
*/

#ifndef MPU6050_LIGHT_H
#define MPU6050_LIGHT_H

#include "Arduino.h"
#include "Wire.h"

#define MPU6050_ADDR                  0x68
#define MPU6050_SMPLRT_DIV_REGISTER   0x19
#define MPU6050_CONFIG_REGISTER       0x1a
#define MPU6050_GYRO_CONFIG_REGISTER  0x1b
#define MPU6050_ACCEL_CONFIG_REGISTER 0x1c
#define MPU6050_PWR_MGMT_1_REGISTER   0x6b

#define MPU6050_GYRO_OUT_REGISTER     0x43
#define MPU6050_ACCEL_OUT_REGISTER    0x3B
#define MPU6050_REG_INT_PIN_CFG       0x37 // INT Pin. Bypass Enable Configuration
#define RAD_2_DEG             57.29578 // [deg/rad]
#define DEG_2_RAD             0.017453 // [rad/deg]
#define G                     9.81     // [m/s^2]
#define M_PI                  3.14159265358979323846
#define M_PI_2                1.57079632679489661923
#define CALIB_OFFSET_NB_MES   500
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0

#define DEFAULT_GYRO_COEFF    0.98

struct Vector
{
    float x;
    float y;
    float z;

    Vector(float& x, float& y, float& z) : x(x), y(y), z(z) {}
};


typedef enum
{
    MPU6050_SCALE_2000DPS         = 0b11,
    MPU6050_SCALE_1000DPS         = 0b10,
    MPU6050_SCALE_500DPS          = 0b01,
    MPU6050_SCALE_250DPS          = 0b00
} mpu6050_dps_t;

typedef enum
{
    MPU6050_RANGE_16G             = 0b11,
    MPU6050_RANGE_8G              = 0b10,
    MPU6050_RANGE_4G              = 0b01,
    MPU6050_RANGE_2G              = 0b00,
} mpu6050_range_t;

typedef enum
{
    MPU6050_DELAY_3MS             = 0b11,
    MPU6050_DELAY_2MS             = 0b10,
    MPU6050_DELAY_1MS             = 0b01,
    MPU6050_NO_DELAY              = 0b00,
} mpu6050_onDelay_t;

typedef enum
{
    MPU6050_DHPF_HOLD             = 0b111,
    MPU6050_DHPF_0_63HZ           = 0b100,
    MPU6050_DHPF_1_25HZ           = 0b011,
    MPU6050_DHPF_2_5HZ            = 0b010,
    MPU6050_DHPF_5HZ              = 0b001,
    MPU6050_DHPF_RESET            = 0b000,
} mpu6050_dhpf_t;

typedef enum
{
    MPU6050_DLPF_6                = 0b110,
    MPU6050_DLPF_5                = 0b101,
    MPU6050_DLPF_4                = 0b100,
    MPU6050_DLPF_3                = 0b011,
    MPU6050_DLPF_2                = 0b010,
    MPU6050_DLPF_1                = 0b001,
    MPU6050_DLPF_0                = 0b000,
} mpu6050_dlpf_t;

  public:
    // INIT and BASIC FUNCTIONS
	MPU6050(TwoWire &w);
    byte begin(int gyro_config_num=1, int acc_config_num=0);
	
	byte writeData(byte reg, byte data);
    byte readData(byte reg);
	
	void calcOffsets(bool is_calc_gyro=true, bool is_calc_acc=true);
	void calcGyroOffsets(){ calcOffsets(true,false); }; // retro-compatibility with v1.0.0
	void calcAccOffsets(){ calcOffsets(false,true); }; // retro-compatibility with v1.0.0
	
	void setAddress(uint8_t addr){ address = addr; };
	uint8_t getAddress(){ return address; };
	
	// MPU CONFIG SETTER
	byte setGyroConfig(int config_num);
	byte setAccConfig(int config_num);
	
    void setGyroOffsets(float x, float y, float z);
	void setAccOffsets(float x, float y, float z);
	
	void setFilterGyroCoef(float gyro_coeff);
	void setFilterAccCoef(float acc_coeff);

	// MPU CONFIG GETTER
	float getGyroXoffset(){ return gyroXoffset; };
    float getGyroYoffset(){ return gyroYoffset; };
    float getGyroZoffset(){ return gyroZoffset; };
	
	float getAccXoffset(){ return accXoffset; };
	float getAccYoffset(){ return accYoffset; };
	float getAccZoffset(){ return accZoffset; };
	
	float getFilterGyroCoef(){ return filterGyroCoef; };
	float getFilterAccCoef(){ return 1.0-filterGyroCoef; };
	
	// DATA GETTER
    float getTemp(){ return temp; };

    float getAccX(){ return accX; };
    float getAccY(){ return accY; };
    float getAccZ(){ return accZ; };

    float getGyroX(){ return gyroX; };
    float getGyroY(){ return gyroY; };
    float getGyroZ(){ return gyroZ; };
	
	float getAccAngleX(){ return angleAccX; };
    float getAccAngleY(){ return angleAccY; };

    float getAngleX(){ return angleX; };
    float getAngleY(){ return angleY; };
    float getAngleZ(){ return angleZ; };

	// INLOOP UPDATE
	void fetchData(); // user should better call 'update' that includes 'fetchData'
    void update();
	
	// UPSIDE DOWN MOUNTING
	bool upsideDownMounting = false;


  private:
    TwoWire *wire;
	uint8_t address = MPU6050_ADDR; // 0x68 or 0x69
	float gyro_lsb_to_degsec, acc_lsb_to_g;
    float gyroXoffset, gyroYoffset, gyroZoffset;
	float accXoffset, accYoffset, accZoffset;
    float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
    float angleAccX, angleAccY;
    float angleX, angleY, angleZ;
    unsigned long preInterval;
    float filterGyroCoef; // complementary filter coefficient to balance gyro vs accelero data to get angle
};

#endif
