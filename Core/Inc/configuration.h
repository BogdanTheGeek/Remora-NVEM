#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define PRU_BASEFREQ    	40000    		// PRU Base thread ISR update frequency (hz)
#define PRU_SERVOFREQ       1000            // PRU Servo thread ISR update freqency (hz)

#define STEPBIT     		22            	// bit location in DDS accum
#define STEP_MASK   		  (1L<<STEPBIT)

#define JSON_BUFF_SIZE	    10000			// Jason dynamic buffer size

#define JOINTS			    8				// Number of joints - set this the same as LinuxCNC HAL compenent. Max 8 joints
#define VARIABLES           6             	// Number of command values - set this the same as the LinuxCNC HAL compenent

#define PRU_DATA			0x64617461 		// "data" payload
#define PRU_READ          	0x72656164  	// "read" payload
#define PRU_WRITE         	0x77726974  	// "writ" payload
#define PRU_ACKNOWLEDGE		0x61636b6e		// "ackn" payload
#define PRU_ERR		        0x6572726f		// "erro" payload
#define PRU_ESTOP           0x65737470  	// "estp" payload
#define PRU_NVMPG			0x6D706764

#define BUFFER_SIZE			68

#define DATA_ERR_MAX		5


// Location for storage of JSON config file in Flash
#define JSON_UPLOAD_ADDRESS				0x080D0000
#define JSON_STORAGE_ADDRESS 			0x080E0000
#define USER_FLASH_LAST_PAGE_ADDRESS  	0x080E0000
#define USER_FLASH_END_ADDRESS        	0x080FFFFF

#define DEFAULT_CONFIG {0x7B, 0x0A, 0x09, 0x22, 0x42, 0x6F, 0x61, 0x72, 0x64, 0x22, 0x3A, 0x20, 0x22, 0x4E, 0x56, 0x45, 0x4D, 0x22, 0x2C, 0x0A, 0x09, 0x22, 0x4D, 0x6F, 0x64, 0x75, 0x6C, 0x65, 0x73, 0x22, 0x3A, 0x5B, 0x0A, 0x09, 0x7B, 0x0A, 0x09, 0x22, 0x54, 0x68, 0x72, 0x65, 0x61, 0x64, 0x22, 0x3A, 0x20, 0x22, 0x42, 0x61, 0x73, 0x65, 0x22, 0x2C, 0x0A, 0x09, 0x22, 0x54, 0x79, 0x70, 0x65, 0x22, 0x3A, 0x20, 0x22, 0x53, 0x74, 0x65, 0x70, 0x67, 0x65, 0x6E, 0x22, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x43, 0x6F, 0x6D, 0x6D, 0x65, 0x6E, 0x74, 0x22, 0x3A, 0x09, 0x09, 0x09, 0x22, 0x58, 0x20, 0x2D, 0x20, 0x4A, 0x6F, 0x69, 0x6E, 0x74, 0x20, 0x30, 0x20, 0x73, 0x74, 0x65, 0x70, 0x20, 0x67, 0x65, 0x6E, 0x65, 0x72, 0x61, 0x74, 0x6F, 0x72, 0x22, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x4A, 0x6F, 0x69, 0x6E, 0x74, 0x20, 0x4E, 0x75, 0x6D, 0x62, 0x65, 0x72, 0x22, 0x3A, 0x09, 0x09, 0x30, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x53, 0x74, 0x65, 0x70, 0x20, 0x50, 0x69, 0x6E, 0x22, 0x3A, 0x20, 0x09, 0x09, 0x22, 0x50, 0x45, 0x5F, 0x31, 0x35, 0x22, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x44, 0x69, 0x72, 0x65, 0x63, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x50, 0x69, 0x6E, 0x22, 0x3A, 0x20, 0x09, 0x22, 0x50, 0x45, 0x5F, 0x31, 0x34, 0x22, 0x0A, 0x09, 0x7D, 0x2C, 0x0A, 0x09, 0x7B, 0x0A, 0x09, 0x22, 0x54, 0x68, 0x72, 0x65, 0x61, 0x64, 0x22, 0x3A, 0x20, 0x22, 0x42, 0x61, 0x73, 0x65, 0x22, 0x2C, 0x0A, 0x09, 0x22, 0x54, 0x79, 0x70, 0x65, 0x22, 0x3A, 0x20, 0x22, 0x53, 0x74, 0x65, 0x70, 0x67, 0x65, 0x6E, 0x22, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x43, 0x6F, 0x6D, 0x6D, 0x65, 0x6E, 0x74, 0x22, 0x3A, 0x09, 0x09, 0x09, 0x22, 0x59, 0x20, 0x2D, 0x20, 0x4A, 0x6F, 0x69, 0x6E, 0x74, 0x20, 0x31, 0x20, 0x73, 0x74, 0x65, 0x70, 0x20, 0x67, 0x65, 0x6E, 0x65, 0x72, 0x61, 0x74, 0x6F, 0x72, 0x22, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x4A, 0x6F, 0x69, 0x6E, 0x74, 0x20, 0x4E, 0x75, 0x6D, 0x62, 0x65, 0x72, 0x22, 0x3A, 0x09, 0x09, 0x31, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x53, 0x74, 0x65, 0x70, 0x20, 0x50, 0x69, 0x6E, 0x22, 0x3A, 0x20, 0x09, 0x09, 0x22, 0x50, 0x45, 0x5F, 0x31, 0x33, 0x22, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x44, 0x69, 0x72, 0x65, 0x63, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x50, 0x69, 0x6E, 0x22, 0x3A, 0x20, 0x09, 0x22, 0x50, 0x45, 0x5F, 0x31, 0x32, 0x22, 0x0A, 0x09, 0x7D, 0x2C, 0x0A, 0x09, 0x7B, 0x0A, 0x09, 0x22, 0x54, 0x68, 0x72, 0x65, 0x61, 0x64, 0x22, 0x3A, 0x20, 0x22, 0x42, 0x61, 0x73, 0x65, 0x22, 0x2C, 0x0A, 0x09, 0x22, 0x54, 0x79, 0x70, 0x65, 0x22, 0x3A, 0x20, 0x22, 0x53, 0x74, 0x65, 0x70, 0x67, 0x65, 0x6E, 0x22, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x43, 0x6F, 0x6D, 0x6D, 0x65, 0x6E, 0x74, 0x22, 0x3A, 0x09, 0x09, 0x09, 0x22, 0x5A, 0x20, 0x2D, 0x20, 0x4A, 0x6F, 0x69, 0x6E, 0x74, 0x20, 0x32, 0x20, 0x73, 0x74, 0x65, 0x70, 0x20, 0x67, 0x65, 0x6E, 0x65, 0x72, 0x61, 0x74, 0x6F, 0x72, 0x22, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x4A, 0x6F, 0x69, 0x6E, 0x74, 0x20, 0x4E, 0x75, 0x6D, 0x62, 0x65, 0x72, 0x22, 0x3A, 0x09, 0x09, 0x32, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x53, 0x74, 0x65, 0x70, 0x20, 0x50, 0x69, 0x6E, 0x22, 0x3A, 0x20, 0x09, 0x09, 0x22, 0x50, 0x45, 0x5F, 0x31, 0x31, 0x22, 0x2C, 0x0A, 0x09, 0x09, 0x22, 0x44, 0x69, 0x72, 0x65, 0x63, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x50, 0x69, 0x6E, 0x22, 0x3A, 0x20, 0x09, 0x22, 0x50, 0x45, 0x5F, 0x31, 0x30, 0x22, 0x0A, 0x09, 0x7D, 0x0A, 0x09, 0x5D, 0x0A, 0x7D, 0x0A, 0x0A};


#endif
