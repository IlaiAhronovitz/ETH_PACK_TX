/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CRC_SIZE 4
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

CRC_HandleTypeDef hcrc;

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM16_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TIM_HandleTypeDef htim;
GPIO_InitTypeDef GPIO_InitStruct;
uint8_t bitPosition = 0;    // Bit position tracker
uint8_t bytePosition = 0;
GPIO_PinState send_frame = GPIO_PIN_RESET;
start_bits = 0;
//GPIO_PinState packet_ready = GPIO_PIN_RESET;
// Ethernet frame structure
typedef struct {
    uint8_t preamble[7]; // Preamble bytes
    uint8_t sfd;
    uint8_t destMac[6];
    uint8_t srcMac[6];
    uint16_t etherType;
    uint8_t payload[2]; // Maximum payload size
    uint32_t crc;
    //uint32_t crc;          // CRC (Cyclic Redundancy Check)
} EthernetFrame;

uint8_t dataToSend[sizeof(EthernetFrame)];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_6 && send_frame == GPIO_PIN_SET) { // Pin PF6 (change to the actual pin)
        // Perform your desired action here upon the rising edge of the clock signal
        // For example, toggle or set/clear the state of another GPIO pin
//    	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (dataToSend[bytePosition] >> bitPosition) & 0x01);
//    	 // Toggle the clock pin to generate a clock pulse
//    	 // Move to the next bit position
//    	 bitPosition++;
//
//    	 if (bitPosition >= 8) {
//    	             bitPosition = 0;  // Reset the bit position for the next transmission
//    	             bytePosition++;
//    	         }
    	if(start_bits < 2){
    		start_bits++;
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    	}
    	else{
    		HAL_TIM_Base_Stop_IT(&htim16);
       	 if(bytePosition >= sizeof(EthernetFrame)){
   //    		 bitPosition = 0;  // Reset the bit position for the next transmission
   //    		 bytePosition = 0;
       		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
       	 }
       	 else{
           	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (dataToSend[bytePosition] >> bitPosition) & 0x01);
           	 // Move to the next bit position
           	 bitPosition++;
           	 bytePosition += bitPosition/8;
           	 bitPosition = bitPosition%8;


   //
   //        	 if (bitPosition >= 8) {
   //        	             bitPosition = 0;  // Reset the bit position for the next transmission
   //        	             bytePosition++;
   //        	         }
       	 }

//           HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); // Replace with your desired GPIO pin and action
       	HAL_TIM_Base_Start_IT(&htim16);
    	}
//    	 if(bytePosition >= sizeof(EthernetFrame)){
////    		 bitPosition = 0;  // Reset the bit position for the next transmission
////    		 bytePosition = 0;
//    		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
//    	 }
//    	 else{
//        	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (dataToSend[bytePosition] >> bitPosition) & 0x01);
//        	 // Toggle the clock pin to generate a clock pulse
//        	 // Move to the next bit position
//        	 bitPosition++;
//        	 bytePosition += bitPosition/8;
//        	 bitPosition = bitPosition%8;
//
//
////
////        	 if (bitPosition >= 8) {
////        	             bitPosition = 0;  // Reset the bit position for the next transmission
////        	             bytePosition++;
////        	         }
//    	 }
//
//        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); // Replace with your desired GPIO pin and action
    }

    if (GPIO_Pin == GPIO_PIN_13 && send_frame == GPIO_PIN_RESET) { // Pin PC13 (change to the actual pin)
        // Perform your desired action here upon the rising edge of the clock signal
        // For example, toggle or set/clear the state of another GPIO pin
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Replace with your desired GPIO pin and action
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        send_frame = GPIO_PIN_SET;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    }


}

uint8_t* constructEthernetPacket(
    const uint8_t* srcMac,
    const uint8_t* destMac,
    uint16_t etherType,
    const uint8_t* payload
) {
    static EthernetFrame frame;
    uint8_t dataToCrc[sizeof(EthernetFrame) - CRC_SIZE];
    memset(&frame, 0, sizeof(EthernetFrame)); // Clear the frame structure

    // Set preamble and SFD (Start of Frame Delimiter)
    for (int i = 0; i < 7; i++) {
        frame.preamble[i] = 0xAA; // Preamble pattern
//    	frame.preamble[i] = 0xFF; // Preamble pattern
    }
    frame.sfd = 0xAB; // SFD pattern

    // Set source and destination MAC addresses
    memcpy(frame.destMac, destMac, 6);
    memcpy(frame.srcMac, srcMac, 6);

    // Set EtherType
    frame.etherType = etherType;

    // Set payload
    memcpy(frame.payload, payload, 2);

    __HAL_RCC_CRC_CLK_ENABLE();
      hcrc.Instance = CRC;
      hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
      hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
      hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
      hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
      hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;

       if (HAL_CRC_Init(&hcrc) != HAL_OK)
       {
         /* Initialization Error */
         Error_Handler();
       }
       memcpy(dataToCrc, &frame, sizeof(EthernetFrame));
       frame.crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)dataToCrc, sizeof(frame) - CRC_SIZE );
       frame.crc = ~(frame.crc);


    // Return the constructed Ethernet frame as a uint8_t array
    return (uint8_t*)&frame;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	GPIO_Config();
	uint16_t timer_val;
  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
//  timeout = 0xFFFF;
//  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
//  if ( timeout < 0 )
//  {
//  Error_Handler();
//  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  uint8_t destMac[] = {0x01, 0x2, 0x3, 0x4, 0x5, 0x6};
  uint8_t srcMac[] = {0x00, 0x00, 0x00, 0xE1, 0x80, 0x00};
//  uint8_t destMac[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
//    uint8_t srcMac[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
  EthernetFrame frame;
  uint8_t preamble_arr[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
  memcpy(frame.preamble, preamble_arr, sizeof(preamble_arr)); // AA is the alternating pattern
//  memset(frame.preamble, 0x0, sizeof(frame.preamble)); // AA is the alternating pattern
  frame.sfd = 0xAB;
//  frame.sfd = 0x0;
  memcpy(frame.destMac, destMac, sizeof(destMac));
  memcpy(frame.srcMac, srcMac, sizeof(srcMac));
  frame.etherType = 0x0800; // IPv4 EtherType (0x0800)
//  frame.etherType = 0x0; // IPv4 EtherType (0x0800)
  memset(frame.payload, 0, sizeof(frame.payload)); // Fill payload with zeros
  // Send the Ethernet frame
//  uint16_t payloadSize = sizeof(frame.payload); // Example payload size
  // Copy the EthernetFrame to dataToSend
//  memcpy(dataToSend, &frame, sizeof(EthernetFrame));

  uint8_t* ethernetPacket = constructEthernetPacket(srcMac, destMac, frame.etherType, frame.payload);
  memcpy(dataToSend, ethernetPacket, sizeof(EthernetFrame));

//  __HAL_RCC_CRC_CLK_ENABLE();
//  hcrc.Instance = CRC;
//  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
//  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
//  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
//  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
//  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
//
//   if (HAL_CRC_Init(&hcrc) != HAL_OK)
//   {
//     /* Initialization Error */
//     Error_Handler();
//   }
//
//   frame.crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)dataToSend, sizeof(dataToSend) - CRC_SIZE );
//   frame.crc = ~(frame.crc);

  __HAL_RCC_GPIOF_CLK_ENABLE(); // Enable clock for GPIOF
     GPIO_InitTypeDef GPIO_InitStruct;
     GPIO_InitStruct.Pin = GPIO_PIN_6; // Pin PF6 (change to the actual pin)
     GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
     GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up/pull-down
     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

     __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable clock for GPIOF
 	GPIO_InitTypeDef GPIOB_init = {};
 	GPIOB_init.Pin = GPIO_PIN_2;
 	GPIOB_init.Mode = GPIO_MODE_OUTPUT_PP;
 	HAL_GPIO_Init(GPIOB, &GPIOB_init);

 	 __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIOF
 	GPIO_InitTypeDef GPIOA_InitStruct;
	 GPIOA_InitStruct.Pin = GPIO_PIN_3; // Pin PA3 (change to the actual pin)
	 GPIOA_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Interrupt on rising edge
	 GPIOA_InitStruct.Pull = GPIO_NOPULL; // No pull-up/pull-down
	  HAL_GPIO_Init(GPIOA, &GPIOA_InitStruct);

     // Enable the external interrupt
     HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0); // EXTI9_5_IRQn is for PF6 (change as needed)
     HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
//     HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

     HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0); // EXTI15_10_IRQn is for PC13 (push button)
     HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);





  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
//__HAL_RCC_HSEM_CLK_ENABLE();
///*Take HSEM */
//HAL_HSEM_FastTake(HSEM_ID_0);
///*Release HSEM in order to notify the CPU2(CM4)*/
//HAL_HSEM_Release(HSEM_ID_0,0);
///* wait until CPU2 wakes up from stop mode */
//timeout = 0xFFFF;
//while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
//if ( timeout < 0 )
//{
//Error_Handler();
//}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM16_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  timer_val = __HAL_TIM_GET_COUNTER(&htim16);
//	  HAL_Delay(1000);
//	  timer_val = __HAL_TIM_GET_COUNTER(&htim16);
//	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
//      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Replace with your desired GPIO pin and action
//	  }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6); // Call the HAL handler
}

void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13); // Call the HAL handler
}



void GPIO_Config(void){
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	//__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef GPIOB_init = {};
	GPIOB_init.Pin = GPIO_PIN_2;
	GPIOB_init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIOB_init);

	GPIO_InitTypeDef GPIOC_init = {};
	GPIOC_init.Pin = GPIO_PIN_13;
	GPIOC_init.Mode = GPIO_MODE_IT_RISING_FALLING;
	HAL_GPIO_Init(GPIOC, &GPIOC_init);

	GPIO_InitTypeDef GPIOB_init1 = {};
	GPIOB_init1.Pin = GPIO_PIN_0;
	GPIOB_init1.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIOB_init1);

}
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
