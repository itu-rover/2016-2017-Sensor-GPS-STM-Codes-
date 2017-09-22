
#ifndef BMP180_H_
#define BMP180_H_

#include <CONFIG.h>

#ifdef CONF_IMU 

#ifdef __cplusplus
extern "C" {
#endif
	
//BMP slave address for AD0 = gnd
//Slave address must be 8 bits 
//First bit is R/W sign and must leave 0	
#define BMP_SLAVE_ADD 0xEE
	
	

#ifdef __cplusplus
}
#endif

#endif	//CONF_IMU
#endif	//MPU6050_H_
