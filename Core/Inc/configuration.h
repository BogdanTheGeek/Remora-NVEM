#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define PRU_BASEFREQ    	40000 //24000   // PRU Base thread ISR update frequency (hz)
#define PRU_SERVOFREQ       1000            // PRU Servo thread ISR update freqency (hz)
//#define OVERSAMPLE          3
//#define SWBAUDRATE          19200           // Software serial baud rate
//#define PRU_COMMSFREQ       (SWBAUDRATE * OVERSAMPLE)

#define STEPBIT     		22            	// bit location in DDS accum
#define STEP_MASK   		  (1L<<STEPBIT)

#define JSON_BUFF_SIZE	    10000			// Jason dynamic buffer size

#define JOINTS			    8				// Number of joints - set this the same as LinuxCNC HAL compenent. Max 8 joints
#define VARIABLES           6             	// Number of command values - set this the same as the LinuxCNC HAL compenent

#define PRU_DATA			0x64617461 	// "data" payload
#define PRU_READ          	0x72656164  // "read" payload
#define PRU_WRITE         	0x77726974  // "writ" payload
#define PRU_ACKNOWLEDGE		0x61636b6e	// "ackn" payload
#define PRU_ERR		        0x6572726f	// "erro" payload
#define PRU_ESTOP           0x65737470  // "estp" payload
#define PRU_NVMPG			0x6D706764

#define BUFFER_SIZE			68

#define DATA_ERR_MAX		5

// Serial configuration
//#define TXD0                P0_2            // MBED pin number
//#define RXD0                P0_3
//#define PC_BAUD             115200          // UART baudrate



#endif
