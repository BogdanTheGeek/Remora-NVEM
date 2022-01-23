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
#include <cstring>

#include "configuration.h"
#include "remora.h"

// Ethernet
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"

// drivers
#include "drivers/pin/pin.h"

// threads
#include "thread/irqHandlers.h"
#include "thread/interrupt.h"
#include "thread/pruThread.h"
#include "thread/createThreads.h"

// modules
#include "modules/module.h"
#include "modules/blink/blink.h"
#include "modules/stepgen/stepgen.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

void udpServer_init(void);
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

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
volatile uint16_t* ptrInputs;
volatile uint16_t* ptrOutputs;

void loadModules()
{
	int joint;

	//printf("\nCreate 1 second blink module to test SERVO thread");
    //Module* blinkServo = new Blink("PC_10", PRU_SERVOFREQ, 1);
    //servoThread->registerModule(blinkServo);

    //printf("\nCreate 1 second blink module to test BASE thread");
    //Module* blinkBase = new Blink("PC_12", PRU_BASEFREQ, 1);
    //baseThread->registerModule(blinkBase);


	// Step generator for Joint 0 [X axis]
	joint = 0;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    Module* joint0 = new Stepgen(PRU_BASEFREQ, joint, "PE_15", "PE_14", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint0);


	// Step generator for Joint 1 [Y axis]
	joint = 1;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    Module* joint1 = new Stepgen(PRU_BASEFREQ, joint, "PE_13", "PE_12", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint1);


	// Step generator for Joint 2 [Z axis]
	joint = 2;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    Module* joint2 = new Stepgen(PRU_BASEFREQ, joint, "PE_11", "PE_10", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint2);

/*
	// Step generator for Joint 3 [A axis]
	joint = 3;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    Module* joint3 = new Stepgen(PRU_BASEFREQ, joint, "PE_9", "PE_8", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint3);


	// Step generator for Joint 4 [B axis]
	joint = 4;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    Module* joint4 = new Stepgen(PRU_BASEFREQ, joint, "PE_7", "PA_8", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint4);


	// Step generator for Joint 5 [C axis]
	joint = 5;
	printf("\nCreate step generator for Joint %d\n", joint);

	ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    Module* joint5 = new Stepgen(PRU_BASEFREQ, joint, "PA_6", "PA_5", STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(joint5);
*/
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
  MX_USART1_UART_Init();
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

	                  // wait for threads to read IO before testing for PRUreset
	                  HAL_Delay(1000);
	              }

	              if (PRUreset)
	              {
	                  // RPi outputs default is high until configured when LinuxCNC spiPRU component is started, PRUreset pin will be high
	                  // stay in start state until LinuxCNC is started
	                  currentState = ST_START;
	              }
	              else
	              {
	                  currentState = ST_IDLE;
	              }

	              break;


	          case ST_IDLE:
	              // do something when idle
	              if (currentState != prevState)
	              {
	                  printf("\n## Entering IDLE state\n");
	              }
	              prevState = currentState;

	              /*
	              // check to see if there there has been SPI errors
	              if (comms.getError())
	              {
	                  printf("Communication data error\n");
	                  comms.setError(false);
	              }

	              //wait for SPI data before changing to running state
	              if (comms.getStatus())
	              {
	                  currentState = ST_RUNNING;
	              }

	              if (PRUreset)
	              {
	                  currentState = ST_WDRESET;
	              }
	              */

	              // ethernet testing - jump straight into RUNNING
	              currentState = ST_RUNNING;

	              break;

	          case ST_RUNNING:
	              // do running tasks
	              if (currentState != prevState)
	              {
	                  printf("\n## Entering RUNNING state\n");
	              }
	              prevState = currentState;

	              /*
	              // check to see if there there has been SPI errors
	              if (comms.getError())
	              {
	                  printf("Communication data error\n");
	                  comms.setError(false);
	              }

	              if (comms.getStatus())
	              {
	                  // SPI data received by DMA
	                  resetCnt = 0;
	                  comms.setStatus(false);
	              }
	              else
	              {
	                  // no data received by DMA
	                  resetCnt++;
	              }

	              if (resetCnt > SPI_ERR_MAX)
	              {
	                  // reset threshold reached, reset the PRU
	                  printf("   Communication data error limit reached, resetting\n");
	                  resetCnt = 0;
	                  currentState = ST_RESET;
	              }

	              if (PRUreset)
	              {
	                  currentState = ST_WDRESET;
	              }
	              */

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



	  ethernetif_input(&gnetif);
	  sys_check_timeouts();
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
   struct udp_pcb *upcb;
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
	   udp_recv(upcb, udp_receive_callback, NULL);
   }
   else
   {
	   udp_remove(upcb);
   }
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	int txlen;
	struct pbuf *txBuf;

	// copy the UDP payload into the rxData structure
	memcpy(&rxBuffer.rxBuffer, p->payload, p->len);

	if (rxBuffer.header == PRU_READ)
	{
		txData.header = PRU_DATA;
		txlen = BUFFER_SIZE;
	}
	else if (rxBuffer.header == PRU_WRITE)
	{
		txData.header = PRU_ACKNOWLEDGE;
		txlen = sizeof(txData.header);

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
