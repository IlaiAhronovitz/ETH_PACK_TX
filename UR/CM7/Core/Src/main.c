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
#include <stdio.h>
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

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

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
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    uint8_t preamble[7]; // Preamble bytes
    uint8_t sfd;
    uint8_t destMac[6];
    uint8_t srcMac[6];
    uint16_t etherType;
    uint8_t payload[2]; // Maximum payload size
    uint32_t crc;          // CRC (Cyclic Redundancy Check)
} EthernetFrame;

uint8_t dataToRecieve[sizeof(EthernetFrame)];

uint8_t bitPosition = 0;    // Bit position tracker
uint8_t bytePosition = 0;
GPIO_PinState rBit;
GPIO_PinState valid_frame = GPIO_PIN_RESET;
uint8_t first = 0;
uint8_t rflag = 0;
uint8_t lcd_row = 0;
uint8_t lcd_col = 0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_6 && valid_frame == GPIO_PIN_SET) { // Pin PF6 (change to the actual pin)
    	first++;
    	if(first > 3)
    	{
    		rflag = 1;
    	}
    	if(rflag){
        // Perform your desired action here upon the rising edge of the clock signal
        // For example, toggle or set/clear the state of another GPIO pin
    	rBit = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
//    	lcd_put_cur(lcd_row, lcd_col);
//    	lcd_col++;
//    	if(lcd_col == 16){
//    		lcd_row = (lcd_row + 1)%2;
//    		if(lcd_row == 0)
//    			lcd_clear();
//    		lcd_col = 0;
//    	}
//    	if(rBit == GPIO_PIN_SET){
//        	lcd_send_string("1");
//    	}
//    	else
//    		lcd_send_string("0");
   	 dataToRecieve[bytePosition] |= rBit << (bitPosition);
    	 // Toggle the clock pin to generate a clock pulse
    	 // Move to the next bit position
    	 bitPosition++;
    	 if (bitPosition >= 8) {
    		 bitPosition = 0;  // Reset the bit position for the next transmission
    		 bytePosition++;
//    		 lcd_clear();
//    		 lcd_put_cur(0, 0);
//    		 char positionString[12]; // Adjust the size based on your needs
//    		 snprintf(positionString, sizeof(positionString), "byte #%u", bytePosition);
//    		 lcd_send_string(positionString);
//    		 lcd_put_cur(1, 0);
//    		 char byteString[8]; // Assuming the byte value can be represented in 3 characters (+ null terminator)
//    		    snprintf(byteString, sizeof(byteString) + 1, "%c%c%c%c%c%c%c%c",
//    		             (dataToRecieve[bytePosition-1] & 0x80) ? '1' : '0',
//    		             (dataToRecieve[bytePosition-1] & 0x40) ? '1' : '0',
//    		             (dataToRecieve[bytePosition-1] & 0x20) ? '1' : '0',
//    		             (dataToRecieve[bytePosition-1] & 0x10) ? '1' : '0',
//    		             (dataToRecieve[bytePosition-1] & 0x08) ? '1' : '0',
//    		             (dataToRecieve[bytePosition-1] & 0x04) ? '1' : '0',
//    		             (dataToRecieve[bytePosition-1] & 0x02) ? '1' : '0',
//    		             (dataToRecieve[bytePosition-1] & 0x01) ? '1' : '0');    		 lcd_send_string(byteString);

    	 }
    	 if(bytePosition == sizeof(EthernetFrame))
    	 {
    		    for(uint8_t count = 0; count < sizeof(EthernetFrame); count++)
    		    {
    	    		 lcd_clear();
    	    		 lcd_put_cur(0, 0);
    	    		 char positionString[9]; // Adjust the size based on your needs
    	    		 snprintf(positionString, sizeof(positionString), "byte #%u", count);
    	    		 lcd_send_string(positionString);
    	    		 lcd_put_cur(1, 0);
    	    		 char byteString[8]; // Assuming the byte value can be represented in 3 characters (+ null terminator)
    	    		    snprintf(byteString, sizeof(byteString) + 1, "%c%c%c%c%c%c%c%c",
    	    		             (dataToRecieve[count] & 0x80) ? '1' : '0',
    	    		             (dataToRecieve[count] & 0x40) ? '1' : '0',
    	    		             (dataToRecieve[count] & 0x20) ? '1' : '0',
    	    		             (dataToRecieve[count] & 0x10) ? '1' : '0',
    	    		             (dataToRecieve[count] & 0x08) ? '1' : '0',
    	    		             (dataToRecieve[count] & 0x04) ? '1' : '0',
    	    		             (dataToRecieve[count] & 0x02) ? '1' : '0',
    	    		             (dataToRecieve[count] & 0x01) ? '1' : '0');    		 lcd_send_string(byteString);


    		    }
    		 bytePosition = 0;
    		 memset(dataToRecieve, 0, sizeof(dataToRecieve));
    	 }
    	}


//    	 switch(bytePosition){
//    	 case 1 ... 56:
//		 preamble[bytePosition % 8] |= rBit << bitPosition;
//    	 break;
//    	 case 57:
//    		 sfd |= rBit << bitPosition;
//    		 break;
//    	 case 58 ... 64:
//		 	 destMac[bytePosition - 58] |= rBit << bitPosition;
//    	 	 break;
//    	 case 65 ... 71:
//		 	 srcMac[bytePosition - 65] |= rBit << bitPosition;
//    	 	 break;
//    	 case 72 ... 73:
//		 	 etherType |= rBit << bitPosition;
//    	 	 break;
//    	 case 74 ... 75:
//		 	 payload[bytePosition - 74] |= rBit << bitPosition;
//    	 	 break;
//    	 default:
//    		 break;
//    	 }




//    	 if (bytePosition >= sizeof(uint8_t) * 7) {
//    	             bitPosition = 0;  // Reset the bit position for the next transmission
//    	             bytePosition++;
//    	         }
//    	 if(bytePosition >= sizeof(EthernetFrame) - 1500){
//    		 bitPosition = 0;  // Reset the bit position for the next transmission
//    		 bytePosition = 0;
//    	 }
    }
    if (GPIO_Pin == GPIO_PIN_3) {
    	valid_frame = GPIO_PIN_SET;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	memset(dataToRecieve, 0, sizeof(dataToRecieve));
  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  //int32_t timeout;
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
  __HAL_RCC_GPIOF_CLK_ENABLE(); // Enable clock for GPIOF
  __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable clock for GPIOB
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIOF


     GPIO_InitTypeDef GPIO_InitStruct;
     GPIO_InitStruct.Pin = GPIO_PIN_6; // Pin PF6 (change to the actual pin)
     GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
     GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up/pull-down
     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  	GPIO_InitTypeDef GPIOB_init = {};
  	GPIOB_init.Pin = GPIO_PIN_2;
  	GPIOB_init.Mode = GPIO_MODE_INPUT;
  	HAL_GPIO_Init(GPIOB, &GPIOB_init);

 	GPIO_InitTypeDef GPIOB_init1 = {};
 	GPIOB_init1.Pin = GPIO_PIN_0;
 	GPIOB_init1.Mode = GPIO_MODE_OUTPUT_PP;
 	HAL_GPIO_Init(GPIOB, &GPIOB_init1);

 	  GPIO_InitTypeDef GPIOA_InitStruct;
 	 GPIOA_InitStruct.Pin = GPIO_PIN_3; // Pin PA3 (change to the actual pin)
 	 GPIOA_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
 	 GPIOA_InitStruct.Pull = GPIO_NOPULL; // No pull-up/pull-down
 	  HAL_GPIO_Init(GPIOA, &GPIOA_InitStruct);

// 	GPIO_InitTypeDef GPIOB_init = {};
// 	GPIOB_init.Pin = GPIO_PIN_2;
// 	GPIOB_init.Mode = GPIO_MODE_OUTPUT_PP;
// 	HAL_GPIO_Init(GPIOB, &GPIOB_init);

     // Enable the external interrupt
     HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0); // EXTI9_5_IRQn is for PF6 (change as needed)
     HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

     HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
     HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  lcd_clear();

//  lcd_put_cur(1, 0);

//  lcd_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0020081F;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Enable Fast Mode Plus
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

void EXTI3_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); // Call the HAL handler
}


void GPIO_Config(void){
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	//__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef GPIOB_init = {};
	GPIOB_init.Pin = GPIO_PIN_2;
	GPIOB_init.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOB, &GPIOB_init);

//	GPIO_InitTypeDef GPIOC_init = {};
//	GPIOC_init.Pin = GPIO_PIN_13;
//	GPIOC_init.Mode = GPIO_MODE_IT_RISING_FALLING;
//	HAL_GPIO_Init(GPIOC, &GPIOC_init);

//	GPIO_InitTypeDef GPIOB_init1 = {};
//	GPIOB_init1.Pin = GPIO_PIN_0;
//	GPIOB_init1.Mode = GPIO_MODE_OUTPUT_PP;
//	HAL_GPIO_Init(GPIOB, &GPIOB_init1);

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
