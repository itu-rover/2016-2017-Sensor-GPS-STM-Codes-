
#define CONF_IMU (1u)
#ifdef CONF_IMU
	#define IMU_TIMEOUT 0x100
	
	#define IMU_MEMORY_SIZE 0x01u
	#define IMU_DATA_SIZE 0x01u
	
	#define IMU_DATA_REGISTER_COUNTER 14
	
	#define GYROX_BIAS 0
	#define GYROY_BIAS 0
	#define GYROZ_BIAS 0
#endif

#define CONF_TCP (1u)
#ifdef CONF_TCP
#endif

#define CONF_GPS (1u)
#ifdef CONF_GPS
	#define GPS_DISABLE_GPGGA	(0u)
	#define GPS_DISABLE_GPRMC (0u)
	#define GPS_DISABLE_GPGSV	(0u)
	#define GPS_DISABLE_GPGSA	(0u)
#endif

