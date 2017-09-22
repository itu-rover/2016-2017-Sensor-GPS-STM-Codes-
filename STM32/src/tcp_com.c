#include <stdint.h>
#include "stm32f1xx_hal.h"
#include <MPU6050.h>
#include <tcp_com.h>

UART_HandleTypeDef *tcp_uart_p;
imu_data_raw_t *tcp_imu_data_p;

void tcp_rx_data_callback(void)
{

}

void tcp_begin(UART_HandleTypeDef *tcp_uart, imu_data_raw_t *imu_data)
{
	tcp_uart_p = tcp_uart;
	tcp_imu_data_p = imu_data;
}







