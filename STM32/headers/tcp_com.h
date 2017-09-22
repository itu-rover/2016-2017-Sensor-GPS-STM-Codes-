
#ifndef TCP_COM_H_
#define TCP_COM_H_

#include <MPU6050.h>
#include <CONFIG.h>

#ifdef CONF_TCP

#ifdef __cplusplus
extern "C" {
#endif
	
void tcp_rx_data_callback(void);
void tcp_begin(UART_HandleTypeDef *tcp_uart, imu_data_raw_t *imu_data);

	


#ifdef __cplusplus
}
#endif

#endif	//CONF_TCP
#endif	//TCP_COM_H_
