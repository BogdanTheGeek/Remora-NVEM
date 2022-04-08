/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <inttypes.h>
#include <cstring>

#include "configuration.h"
#include "remora.h"

// Flash storage
#include "flash_if.h"

// Ethernet
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "tftpserver.h"

// libraries
#include <sys/errno.h>
#include "lib/ArduinoJson6/ArduinoJson.h"

// drivers
#include "drivers/pin/pin.h"

// interrupts
#include "interrupt/irqHandlers.h"
#include "interrupt/interrupt.h"

// threads
#include "thread/pruThread.h"
#include "thread/createThreads.h"

// modules
#include "modules/module.h"
#include "modules/blink/blink.h"
#include "modules/comms/RemoraComms.h"
#include "modules/debug/debug.h"
#include "modules/pwm/spindlePWM.h"
#include "modules/pwm/softPWM.h"
#include "modules/stepgen/stepgen.h"
#include "modules/digitalPin/digitalPin.h"
#include "modules/nvmpg/nvmpg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_USART2_UART_Init(void);

void udpServer_init(void);
void udp_data_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udp_mpg_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// re-target printf to UART2 by redeclaring week function in syscalls.c
extern "C" {
	int __io_putchar(int ch)
	{
	  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	  return ch;
	}
}

// network interface
struct netif gnetif;

// state machine
enum State {
    ST_SETUP = 0,
    ST_START,
    ST_IDLE,
    ST_RUNNING,
    ST_STOP,
    ST_RESET,
    ST_WDRESET
};

uint8_t resetCnt;
//uint32_t base_freq = PRU_BASEFREQ;
//uint32_t servo_freq = PRU_SERVOFREQ;

// boolean
volatile bool PRUreset;
bool configError = false;
bool threadsRunning = false;

// pointers to objects with global scope
pruThread* servoThread;
pruThread* baseThread;

// unions for RX and TX data
rxData_t rxBuffer;				// temporary RX buffer
volatile rxData_t rxData;
volatile txData_t txData;

RemoraComms* comms;

mpgData_t mpgData;
Module* MPG;

// pointers to data
volatile rxData_t*  ptrRxData = &rxData;
volatile txData_t*  ptrTxData = &txData;
volatile int32_t* ptrTxHeader;
volatile bool*    ptrPRUreset;
volatile int32_t* ptrJointFreqCmd[JOINTS];
volatile int32_t* ptrJointFeedback[JOINTS];
volatile uint8_t* ptrJointEnable;
volatile float*   ptrSetPoint[VARIABLES];
volatile float*   ptrProcessVariable[VARIABLES];
volatile uint32_t* ptrInputs;
volatile uint32_t* ptrOutputs;
volatile uint16_t* ptrNVMPGInputs;

volatile mpgData_t* ptrMpgData = &mpgData;


// Json config file stuff

// 512 bytes of metadata in front of actual JSON file
typedef struct
{
  uint32_t crc32;   // crc32 of JSON
  uint32_t length;  // length in bytes
  uint8_t padding[504];
} metadata_t;
#define METADATA_LEN    512

volatile bool newJson;
uint32_t crc32;
FILE *jsonFile;
string strJson;
DynamicJsonDocument doc(JSON_BUFF_SIZE);
JsonObject thread;
JsonObject module;

uint8_t checkJson()
{
	metadata_t* meta = (metadata_t*)JSON_UPLOAD_ADDRESS;
	uint32_t* json = (uint32_t*)(JSON_UPLOAD_ADDRESS + METADATA_LEN);

	// Check length is reasonable
	//printf("Config length = %d\n", meta->length);
	if (meta->length > (USER_FLASH_END_ADDRESS - JSON_UPLOAD_ADDRESS))
	{
		newJson = false;
		printf("JSON Config length incorrect\n");
		return -1;
	}

	// Enable & Reset CRC
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
	CRC->CR = 1;

	// Compute CRC
	// Note: __RBIT funkiness is so that CRC will match standard calculation
	// See http://forum.chibios.org/phpbb/viewtopic.php?f=2&t=1475 for details
	for (uint32_t i = 0; i < meta->length; i++)
	  CRC->DR = __RBIT(*(json+i));

	crc32 = __RBIT(CRC->DR) ^ 0xFFFFFFFF;

	//printf("crc32 = %x\n", crc32);

	// Disable CRC
	RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;

	// Check CRC
	if (crc32 != meta->crc32)
	{
		newJson = false;
		printf("JSON Config file CRC incorrect\n");
		return -1;
	}

	// JSON is OK, don't check it again
	newJson = false;
	printf("JSON Config file recieved Ok\n");
	return 1;
}


void moveJson()
{
	uint32_t i = 0;
	metadata_t* meta = (metadata_t*)JSON_UPLOAD_ADDRESS;

	uint16_t jsonLength = meta->length;

	// erase the old JSON config file
	FLASH_If_Erase(JSON_STORAGE_ADDRESS);

	HAL_StatusTypeDef status;

	// store the length of the file in the 0th byte
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, JSON_STORAGE_ADDRESS, jsonLength);

    for (i = 0; i < jsonLength; i++)
    {
        if (status == HAL_OK)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (JSON_STORAGE_ADDRESS + 4 + i), *((uint8_t*)(JSON_UPLOAD_ADDRESS + METADATA_LEN + i)));
        }
    }

}

void loadModules()
{
	int joint;
    ptrInputs = &txData.inputs;
    ptrOutputs = &rxData.outputs;
    ptrNVMPGInputs = &txData.NVMPGinputs;

    // Ethernet communication monitoring
	comms = new RemoraComms();
	servoThread->registerModule(comms);


	// -------------------- TESTING ---------------------------------
	//printf("\nCreate 1 second blink module to test SERVO thread");
    //Module* blinkServo = new Blink("PC_2", PRU_SERVOFREQ, 1); //"PB_8"
    //servoThread->registerModule(blinkServo);

    //printf("\nCreate 1 second blink module to test BASE thread");
    //Module* blinkBase = new Blink("PC_3", PRU_BASEFREQ, 1); //"PC_12"
    //baseThread->registerModule(blinkBase);
	// -------------------- TESTING ---------------------------------


	// STEP GENERATORS

	// Step generator for Joint 0 [X axis]
	joint = 0;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    Module* joint0 = new Stepgen(PRU_BASEFREQ, joint, "PE_15", "PE_14", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint0);
    baseThread->registerModulePost(joint0);


	// Step generator for Joint 1 [Y axis]
	joint = 1;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    Module* joint1 = new Stepgen(PRU_BASEFREQ, joint, "PE_13", "PE_12", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint1);
    baseThread->registerModulePost(joint1);


	// Step generator for Joint 2 [Z axis]
	joint = 2;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    Module* joint2 = new Stepgen(PRU_BASEFREQ, joint, "PE_11", "PE_10", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint2);
    baseThread->registerModulePost(joint2);


	// Step generator for Joint 3 [A axis]
	joint = 3;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
	ptrJointFeedback[joint] = &txData.jointFeedback[joint];
	ptrJointEnable = &rxData.jointEnable;

	Module* joint3 = new Stepgen(PRU_BASEFREQ, joint, "PE_9", "PE_8", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
	baseThread->registerModule(joint3);
	baseThread->registerModulePost(joint3);


 	// Step generator for Joint 4 [B axis]
	joint = 4;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    Module* joint4 = new Stepgen(PRU_BASEFREQ, joint, "PE_7", "PA_8", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint4);
    baseThread->registerModulePost(joint4);


	// Step generator for Joint 5 [C axis]
	joint = 5;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    //Module* joint5 = new Stepgen(PRU_BASEFREQ, joint, "PA_6", "PA_5", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    Module* joint5 = new Stepgen(PRU_BASEFREQ, joint, "PE_9", "PE_8", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint5);
    baseThread->registerModulePost(joint5);


    // INPUTS
    Module* FHA = new DigitalPin(*ptrInputs, 0, "PD_12", 0, true, NONE);
    servoThread->registerModule(FHA);

    Module* FHB = new DigitalPin(*ptrInputs, 0, "PD_13", 1, true, NONE);
    servoThread->registerModule(FHB);

    Module* SRO = new DigitalPin(*ptrInputs, 0, "PB_14", 2, true, NONE);
    servoThread->registerModule(SRO);

    Module* SJR = new DigitalPin(*ptrInputs, 0, "PB_15", 3, true, NONE);
    servoThread->registerModule(SJR);

    Module* STOP = new DigitalPin(*ptrInputs, 0, "PD_8", 4, true, NONE);
    servoThread->registerModule(STOP);

    Module* PROBE = new DigitalPin(*ptrInputs, 0, "PD_9", 5, true, NONE);
    servoThread->registerModule(PROBE);

    Module* INP3 = new DigitalPin(*ptrInputs, 0, "PD_10", 6, true, NONE);
    servoThread->registerModule(INP3);

    Module* INP4 = new DigitalPin(*ptrInputs, 0, "PD_11", 7, true, NONE);
    servoThread->registerModule(INP4);

    Module* INP5 = new DigitalPin(*ptrInputs, 0, "PD_14", 8, true, NONE);
    servoThread->registerModule(INP5);

    Module* INP6 = new DigitalPin(*ptrInputs, 0, "PD_15", 9, true, NONE);
    servoThread->registerModule(INP6);

    Module* INP7 = new DigitalPin(*ptrInputs, 0, "PC_6", 10, true, NONE);
    servoThread->registerModule(INP7);

    Module* INP8 = new DigitalPin(*ptrInputs, 0, "PC_7", 11, true, NONE);
    servoThread->registerModule(INP8);

    Module* INP9 = new DigitalPin(*ptrInputs, 0, "PC_8", 12, true, NONE);
    servoThread->registerModule(INP9);

    Module* INP10 = new DigitalPin(*ptrInputs, 0, "PC_9", 13, true, NONE);
    servoThread->registerModule(INP10);

    Module* INP11 = new DigitalPin(*ptrInputs, 0, "PA_11", 14, true, NONE);
    servoThread->registerModule(INP11);

    Module* INP12 = new DigitalPin(*ptrInputs, 0, "PA_12", 15, true, NONE);
    servoThread->registerModule(INP12);

    Module* INDEX = new DigitalPin(*ptrInputs, 0, "PC_15", 16, true, NONE);
    servoThread->registerModule(INDEX);

    Module* x100 = new DigitalPin(*ptrInputs, 0, "PA_15", 17, true, NONE);
    servoThread->registerModule(x100);

    Module* x10 = new DigitalPin(*ptrInputs, 0, "PC_10", 18, true, NONE);
    servoThread->registerModule(x10);

    Module* x1 = new DigitalPin(*ptrInputs, 0, "PC_11", 19, true, NONE);
    servoThread->registerModule(x1);

    Module* ESTOP = new DigitalPin(*ptrInputs, 0, "PC_12", 20, true, NONE);
    servoThread->registerModule(ESTOP);

    Module* Xin = new DigitalPin(*ptrInputs, 0, "PD_7", 21, true, NONE);
    servoThread->registerModule(Xin);

    Module* Yin = new DigitalPin(*ptrInputs, 0, "PD_4", 22, true, NONE);
    servoThread->registerModule(Yin);

    Module* Zin = new DigitalPin(*ptrInputs, 0, "PD_3", 23, true, NONE);
    servoThread->registerModule(Zin);

    Module* Ain = new DigitalPin(*ptrInputs, 0, "PD_2", 24, true, NONE);
    servoThread->registerModule(Ain);

    Module* Bin = new DigitalPin(*ptrInputs, 0, "PD_1", 25, true, NONE);
    servoThread->registerModule(Bin);

    Module* Cin = new DigitalPin(*ptrInputs, 0, "PD_0", 26, true, NONE);
    servoThread->registerModule(Cin);

    Module* WHA = new DigitalPin(*ptrInputs, 0, "PB_7", 27, false, NONE);
    servoThread->registerModule(WHA);

    Module* WHB = new DigitalPin(*ptrInputs, 0, "PB_6", 28, false, NONE);
    servoThread->registerModule(WHB);


    // OUTPUTS
    Module* OUT1 = new DigitalPin(*ptrOutputs, 1, "PC_3", 0, false, NONE);
    servoThread->registerModule(OUT1);

    Module* OUT2 = new DigitalPin(*ptrOutputs, 1, "PC_2", 1, false, NONE);
    servoThread->registerModule(OUT2);

    Module* OUT3 = new DigitalPin(*ptrOutputs, 1, "PB_8", 3, false, NONE);
    servoThread->registerModule(OUT3);

    Module* OUT4 = new DigitalPin(*ptrOutputs, 1, "PB_9", 4, false, NONE);
    servoThread->registerModule(OUT4);

    Module* OUT5 = new DigitalPin(*ptrOutputs, 1, "PE_0", 5, false, NONE);
    servoThread->registerModule(OUT5);

    Module* OUT6 = new DigitalPin(*ptrOutputs, 1, "PE_1", 6, false, NONE);
    servoThread->registerModule(OUT6);

    Module* OUT7 = new DigitalPin(*ptrOutputs, 1, "PE_2", 7, false, NONE);
    servoThread->registerModule(OUT7);

    Module* OUT8 = new DigitalPin(*ptrOutputs, 1, "PE_3", 8, false, NONE);
    servoThread->registerModule(OUT8);

    Module* OUT9 = new DigitalPin(*ptrOutputs, 1, "PC_13", 9, false, NONE);
    servoThread->registerModule(OUT9);

    Module* OUT10 = new DigitalPin(*ptrOutputs, 1, "PC_14", 10, false, NONE);
    servoThread->registerModule(OUT10);

    // SPINDLE
    ptrSetPoint[0] = &rxData.setPoint[0];
    Module* spindle = new SpindlePWM(*ptrSetPoint[0]);
    servoThread->registerModule(spindle);

    //Module* spindle = new SoftPWM(*ptrSetPoint[0], "PA_0");
    //baseThread->registerModule(spindle);


    // (NVMPG serial) MANUAL PULSE GENERATOR
	MPG = new NVMPG(*ptrMpgData, *ptrNVMPGInputs);
	servoThread->registerModule(MPG);
}

void debugThreadHigh()
{
    Module* debugOnB = new Debug("PC_10", 1);
    baseThread->registerModule(debugOnB);

    Module* debugOnS = new Debug("PC_12", 1);
    servoThread->registerModule(debugOnS);
}

void debugThreadLow()
{
    Module* debugOffB = new Debug("PC_10", 0);
    baseThread->registerModule(debugOffB);

    Module* debugOffS = new Debug("PC_12", 0);
    servoThread->registerModule(debugOffS);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_LWIP_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  enum State currentState;
  enum State prevState;

  currentState = ST_SETUP;
  prevState = ST_RESET;

  printf("Remora-NVEM starting\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(currentState){
	          case ST_SETUP:
	              // do setup tasks
	              if (currentState != prevState)
	              {
	                  printf("\n## Entering SETUP state\n\n");
	              }
	              prevState = currentState;

	              createThreads();
	              //debugThreadHigh();
	              loadModules();
	              //debugThreadLow();
	              udpServer_init();
	              IAP_tftpd_init();

	              currentState = ST_START;
	              break;

	          case ST_START:
	              // do start tasks
	              if (currentState != prevState)
	              {
	                  printf("\n## Entering START state\n");
	              }
	              prevState = currentState;

	              if (!threadsRunning)
	              {
	                  // Start the threads
	                  printf("\nStarting the BASE thread\n");
	                  baseThread->startThread();

	                  printf("\nStarting the SERVO thread\n");
	                  servoThread->startThread();

	                  threadsRunning = true;
	              }

	              currentState = ST_IDLE;

	              break;


	          case ST_IDLE:
	              // do something when idle
	              if (currentState != prevState)
	              {
	                  printf("\n## Entering IDLE state\n");
	              }
	              prevState = currentState;

	              //wait for data before changing to running state
	              if (comms->getStatus())
	              {
	                  currentState = ST_RUNNING;
	              }

	              break;

	          case ST_RUNNING:
	              // do running tasks
	              if (currentState != prevState)
	              {
	                  printf("\n## Entering RUNNING state\n");
	              }
	              prevState = currentState;

	              if (comms->getStatus() == false)
	              {
	            	  currentState = ST_RESET;
	              }

	              break;

	          case ST_STOP:
	              // do stop tasks
	              if (currentState != prevState)
	              {
	                  printf("\n## Entering STOP state\n");
	              }
	              prevState = currentState;


	              currentState = ST_STOP;
	              break;

	          case ST_RESET:
	              // do reset tasks
	              if (currentState != prevState)
	              {
	                  printf("\n## Entering RESET state\n");
	              }
	              prevState = currentState;

	              // set all of the rxData buffer to 0
	              // rxData.rxBuffer is volatile so need to do this the long way. memset cannot be used for volatile
	              printf("   Resetting rxBuffer\n");
	              {
	                  int n = sizeof(rxData.rxBuffer);
	                  while(n-- > 0)
	                  {
	                      rxData.rxBuffer[n] = 0;
	                  }
	              }

	              currentState = ST_IDLE;
	              break;

	          case ST_WDRESET:
	              // do a watch dog reset
	              printf("\n## Entering WDRESET state\n");

	              // force a watchdog reset by looping here
	              while(1){}

	              break;
	  }

	  // do Ethernet tasks
	  ethernetif_input(&gnetif);
	  sys_check_timeouts();

	  if (newJson)
	  {
		  printf("\n\nChecking new configuration file\n");
		  if (checkJson() > 0)
		  {
			  moveJson();
		  }
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


void udpServer_init(void)
{
	// UDP Control Block structure
   struct udp_pcb *upcb, *upcb2;
   err_t err;

   /* 1. Create a new UDP control block  */
   upcb = udp_new();

   /* 2. Bind the upcb to the local port */
   ip_addr_t myIPADDR;
   IP_ADDR4(&myIPADDR, 10, 10, 10, 10);

   err = udp_bind(upcb, &myIPADDR, 27181);  // 27181 is the server UDP port


   /* 3. Set a receive callback for the upcb */
   if(err == ERR_OK)
   {
	   udp_recv(upcb, udp_data_callback, NULL);
   }
   else
   {
	   udp_remove(upcb);
   }


   // Try making a second UDP control block...?

   upcb2 = udp_new();
   err = udp_bind(upcb2, &myIPADDR, 27182);  // 27182 is the server UDP port for NVMPG

   if(err == ERR_OK)
   {
	   udp_recv(upcb2, udp_mpg_callback, NULL);
   }
   else
   {
	   udp_remove(upcb2);
   }
}

void udp_data_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	int txlen = 0;
	struct pbuf *txBuf;

	// copy the UDP payload into the rxData structure
	memcpy(&rxBuffer.rxBuffer, p->payload, p->len);

	if (rxBuffer.header == PRU_READ)
	{
		txData.header = PRU_DATA;
		txlen = BUFFER_SIZE;
		comms->dataReceived();
	}
	else if (rxBuffer.header == PRU_WRITE)
	{
		txData.header = PRU_ACKNOWLEDGE;
		txlen = sizeof(txData.header);
		comms->dataReceived();

		// then move the data
		for (int i = 0; i < BUFFER_SIZE; i++)
		{
			rxData.rxBuffer[i] = rxBuffer.rxBuffer[i];
		}
	}


	// allocate pbuf from RAM
	txBuf = pbuf_alloc(PBUF_TRANSPORT, txlen, PBUF_RAM);

	// copy the data into the buffer
	pbuf_take(txBuf, (char*)&txData.txBuffer, txlen);

	// Connect to the remote client
	udp_connect(upcb, addr, port);

	// Send a Reply to the Client
	udp_send(upcb, txBuf);

	// free the UDP connection, so we can accept new clients
	udp_disconnect(upcb);

	// Free the p_tx buffer
	pbuf_free(txBuf);

	// Free the p buffer
	pbuf_free(p);
}


void udp_mpg_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	// copy the UDP payload into the nvmpg structure
	memcpy(&mpgData.payload, p->payload, p->len);

	// Free the p buffer
	pbuf_free(p);

	if (mpgData.header == PRU_NVMPG)
	{
		// use a standard module interface to trigger the update of the MPG
		MPG->configure();
	}
}
