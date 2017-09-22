#include <stdint.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include <MPU6050.h>

//Interrupt activity status
#define THERE_IS_NO_ACTIVE_READING 0x00u
#define THERE_IS_ACTIVE_READING 0x01u

typedef struct
{
	uint8_t data_counter;
	uint8_t data_size;
	uint8_t active_reading;
}imu_it_handler_t;

//public variables
I2C_HandleTypeDef *imu_i2c_p;
imu_data_raw_t data_buffer_raw;
imu_data_t data_buffer;
uint8_t imu_sensor_values[14];
imu_it_handler_t imu_it_handler;
float g_imu_rate_s;
extern IWDG_HandleTypeDef hiwdg;

float P[3][2][2];

float bias[3] = {0, 0, 0};
float angle[3] = {0, 0, 0};
float O_G[3] = {0, 0, 0};
float O_A[3] = {0, 0, 0};


//static function prototypes
static uint8_t read_register(uint8_t addr);
static void write_register(uint8_t addr, uint8_t data);
static imu_error_t read_register_it(uint8_t addr, uint8_t *data);
static void calculate_data(void);
static float kalman_filter(uint8_t index, int16_t gyro_raw, int16_t acc1_raw, int16_t acc2_raw);
void imu_calculate_data(void);

static void magnet_calibration(void);

//static functions
static uint8_t read_register(uint8_t addr)
{
	uint8_t regval;
	HAL_I2C_Mem_Read(imu_i2c_p, IMU_SLAVE_ADD, addr, IMU_MEMORY_SIZE, &regval, IMU_MEMORY_SIZE, IMU_TIMEOUT);
	return regval;
}

static void write_register(uint8_t addr, uint8_t data)
{
	HAL_I2C_Mem_Write(imu_i2c_p, IMU_SLAVE_ADD, addr, IMU_MEMORY_SIZE, &data, IMU_MEMORY_SIZE, IMU_TIMEOUT);
}
 
static imu_error_t read_register_it(uint8_t addr, uint8_t *data)
{
	HAL_I2C_Mem_Read_IT(imu_i2c_p, IMU_SLAVE_ADD, addr, IMU_MEMORY_SIZE, data, IMU_DATA_SIZE);
	return IMU_ERROR_SUCCES;
}	

//SENSOR DATA
imu_error_t imu_read_data(void)
{
//	imu_it_handler.data_counter = 0;
//	imu_it_handler.data_size = IMU_DATA_REGISTER_COUNTER;
//	imu_it_handler.active_reading = THERE_IS_ACTIVE_READING;
//	HAL_I2C_MemRxCpltCallback(imu_i2c_p);
	
	int condition = HAL_I2C_Mem_Read(imu_i2c_p, IMU_SLAVE_ADD, IMU_ACCEL_XOUT_H, 1, imu_sensor_values, 14, 10);
	if(condition != 0x00)
	{
		HAL_IWDG_Start(&hiwdg);
		while(1){};
	}
	imu_calculate_data();
	
	return IMU_ERROR_SUCCES;
}

static void imu_calculate_data(void)
{
	data_buffer_raw.imu_data_accX = imu_sensor_values[0] << 8u | imu_sensor_values[1];
	data_buffer_raw.imu_data_accY = imu_sensor_values[2] << 8u | imu_sensor_values[3];
	data_buffer_raw.imu_data_accZ = (-1) * (imu_sensor_values[4] << 8u | imu_sensor_values[5]);
	data_buffer_raw.imu_data_gyroX = (-1) * (imu_sensor_values[8] << 8u | imu_sensor_values[9]);
	data_buffer_raw.imu_data_gyroY = imu_sensor_values[10] << 8u | imu_sensor_values[11];
	data_buffer_raw.imu_data_gyroZ = imu_sensor_values[12] << 8u | imu_sensor_values[13];
	data_buffer_raw.imu_data_temp = imu_sensor_values[6] << 8u | imu_sensor_values[7];
	
	data_buffer.imu_data_X = kalman_filter(0, data_buffer_raw.imu_data_gyroX, data_buffer_raw.imu_data_accY, data_buffer_raw.imu_data_accZ);
	data_buffer.imu_data_Y = kalman_filter(1, data_buffer_raw.imu_data_gyroY, data_buffer_raw.imu_data_accX, data_buffer_raw.imu_data_accZ);
}

void imu_rx_cplt_callback(void)
{
	if(imu_it_handler.active_reading == THERE_IS_ACTIVE_READING)
	{
		if(imu_it_handler.data_counter < imu_it_handler.data_size)
		{
			read_register_it(IMU_ACCEL_XOUT_H + imu_it_handler.data_counter, &imu_sensor_values[imu_it_handler.data_counter]);
			imu_it_handler.data_counter++;
		}
		else
		{
			imu_it_handler.active_reading = THERE_IS_NO_ACTIVE_READING;
			imu_data_ready_callback(data_buffer);
		}	
	}	
}
//END OF SENSOR DATA



//INIT
imu_error_t imu_begin(I2C_HandleTypeDef *imu_i2c, uint32_t rate_ms, imu_gyro_range_t gyro_range, imu_acc_range_t acc_range)
{
	imu_i2c_p = imu_i2c;
	uint8_t regval;
	
	g_imu_rate_s = (float)rate_ms / 1000;
	
	//Checks communication 
	if(read_register(IMU_WHO_AM_I) != (IMU_SLAVE_ADD >> 1u))
	{
		return IMU_ERROR_COMM;
	}
	
	//set up gyro range
	regval = read_register(IMU_GYRO_CONFIG);
	regval |= gyro_range << IMU_MASK_GYRO_CONFIG_RATE;
	write_register(IMU_GYRO_CONFIG, regval);
	
	//set up accelerometer range
 	regval = read_register(IMU_ACCEL_CONFIG);
	regval |= acc_range << IMU_MASK_ACCEL_CONFIG_RATE;
	write_register(IMU_ACCEL_CONFIG, regval);
	
	//power on the imu
	write_register(IMU_PWR_MGMT_1, 0x00u);
	
	magnet_calibration();
	
	return IMU_ERROR_SUCCES;
}
//END OF INIT

static float kalman_filter(uint8_t index, int16_t gyro_raw, int16_t acc1_raw, int16_t acc2_raw)
{
	float dt = g_imu_rate_s;
	float Q_angle = 0.001;
	float Q_gyroBias = 0.003;
	float R_measure = 0.03;
	float K[2], S, rate, y;
		
	O_G[index] += (((gyro_raw + 200) * 250.0) / 32768) * g_imu_rate_s;
	if(index == 2)
	{
		O_A[index] = data_buffer.imu_data_Z;
	}
	else
	{
		O_A[index] = atan2(acc1_raw, acc2_raw) * 180 / 3.14;
	}
	
	rate	= (gyro_raw * 250.0 / 32768) - bias[index];
	angle[index] += dt * rate;
	
	P[index][0][0] += dt * (dt*P[index][1][1] - P[index][0][1] - P[index][1][0] + Q_angle);
	P[index][0][1] -= dt * P[index][1][1];
	P[index][1][0] -= dt * P[index][1][1];
	P[index][1][1] += Q_gyroBias * dt;
	
	y = O_A[index] - angle[index];

	S = P[index][0][0] + R_measure;

	K[0] = P[index][0][0] / S;
	K[1] = P[index][1][0] / S;

	angle[index] += K[0] * y;
	bias[index] += K[1] * y;

	float P00_temp = P[index][0][0];
	float P01_temp = P[index][0][1];

	P[index][0][0] -= K[0] * P00_temp;
	P[index][0][1] -= K[0] * P01_temp;
	P[index][1][0] -= K[1] * P00_temp;
	P[index][1][1] -= K[1] * P01_temp;
	
	return angle[index];
}

#define MAG_CONF_A_ENABLE_TEMP	0x80u		//default disabled

#define MAG_CONF_A_1_AVERAGE 		0x00u		//default
#define MAG_CONF_A_2_AVERAGE 		0x20u
#define MAG_CONF_A_4_AVERAGE 		0x40u
#define MAG_CONF_A_8_AVERAGE 		0x60u

#define MAG_CONF_A_0d75_RATE		0x00u
#define MAG_CONF_A_1d5_RATE			0x04u
#define MAG_CONF_A_3_RATE				0x08u
#define MAG_CONF_A_7d5_RATE			0x0Cu	
#define MAG_CONF_A_15_RATE			0x10u		//default
#define MAG_CONF_A_30_RATE			0x14u
#define MAG_CONF_A_75_RATE			0x18u
#define MAG_CONF_A_220_RATE			0x1Cu

#define MAG_CONF_B_0d88_GAIN 		0x00u
#define MAG_CONF_B_1d3_GAIN 		0x20u		//default
#define MAG_CONF_B_1d9_GAIN 		0x40u
#define MAG_CONF_B_2d5_GAIN			0x60u
#define MAG_CONF_B_4_GAIN 			0x80u
#define MAG_CONF_B_4d7_GAIN			0xA0u
#define MAG_CONF_B_5d6_GAIN			0xC0u
#define MAG_CONF_B_8d1_GAIN			0xE0u

#define MAG_MOD_CONTINUOS				0x00u
#define MAG_MOD_SINGLE					0x01u		//default
#define MAG_MOD_IDLE1						0x02u
#define MAG_MOD_IDLE2						0x03u

#define MAG_SLAVE_ADD						0x3C

#define MAG_REG_CONF_A					0x00u
#define MAG_REG_CONF_B					0x01u
#define MAG_REG_MODE						0x02u
#define MAG_REG_FD							0x03u

float calibBufferXX = 0;
float calibBufferZZ = 0;

static void magnet_calibration(void)
{
	uint8_t conf_A_reg = 0x10u;	//as default
	uint8_t conf_B_reg = 0x20u;	//as default	
	uint8_t mode_reg = 0x00u;		//as default
	
//	conf_A_reg |= (MAG_CONF_A_ENABLE_TEMP | MAG_CONF_A_4_AVERAGE | MAG_CONF_A_220_RATE);
//	conf_B_reg |= (MAG_CONF_B_1d3_GAIN);
	mode_reg |= (MAG_MOD_CONTINUOS);
	
	//Checks communication 
	if(HAL_I2C_IsDeviceReady(imu_i2c_p, MAG_SLAVE_ADD, 5, 200) != 0x00)
	{
		while(1){};	//com error
	}	
	HAL_I2C_Mem_Write(imu_i2c_p, MAG_SLAVE_ADD, MAG_REG_CONF_A, 1, &conf_A_reg, 1, IMU_TIMEOUT);
	HAL_I2C_Mem_Write(imu_i2c_p, MAG_SLAVE_ADD, MAG_REG_CONF_B, 1, &conf_B_reg, 1, IMU_TIMEOUT);
	HAL_I2C_Mem_Write(imu_i2c_p, MAG_SLAVE_ADD, MAG_REG_MODE, 1, &mode_reg, 1, IMU_TIMEOUT);	
}

void magnet_read_data(void)
{
	uint8_t mag_buffer[6];
	HAL_I2C_Mem_Read(imu_i2c_p, MAG_SLAVE_ADD, MAG_REG_FD, 1, mag_buffer, 6, 100);
	data_buffer_raw.imu_data_magX = mag_buffer[0] << 8u | mag_buffer[1];
	data_buffer_raw.imu_data_magY = mag_buffer[2] << 8u | mag_buffer[3];
	data_buffer_raw.imu_data_magZ = mag_buffer[4] << 8u | mag_buffer[5];
	
	calibBufferXX = (((float)data_buffer_raw.imu_data_magX + 188) / 290) - 1;
	calibBufferZZ = (((float)data_buffer_raw.imu_data_magZ + 417) / 290) - 1;
	
	data_buffer.imu_data_Z = atan2(calibBufferZZ, calibBufferXX)  * (-57.296) + 270;
	if(data_buffer.imu_data_Z > 360)
	{
		data_buffer.imu_data_Z = data_buffer.imu_data_Z - 360;
	}
//	data_buffer.imu_data_Z = atan2((data_buffer_raw.imu_data_magY * cos(data_buffer.imu_data_Y / (57.296))
//																+ data_buffer_raw.imu_data_magZ * sin(data_buffer.imu_data_Y / (57.296)))
//																,(data_buffer_raw.imu_data_magX * cos(data_buffer.imu_data_X / (57.296)) 
//																+ data_buffer_raw.imu_data_magZ * sin(data_buffer.imu_data_X / (57.296)))) * (57.296);	//tilt companzation
	data_buffer.imu_data_heading = kalman_filter(2, data_buffer_raw.imu_data_gyroZ, 0, 0);
}



