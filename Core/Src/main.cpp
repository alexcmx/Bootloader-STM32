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
#include <vector>

#include "FirmwareStruct.h"
using namespace std;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define FIRMWARE_START_ADDRESS 0x08006000
#define ONE_BLOCK_Bytes 32
#define WORD 4
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void AppStart(void)
{
	uint32_t appJumpAddress;
	void (*GoToApp)(void);
	appJumpAddress = *((volatile uint32_t*)(FIRMWARE_START_ADDRESS + 4)); //указатель на Reset_Handler
	GoToApp = (void (*)(void))appJumpAddress;
	HAL_DeInit();
	__disable_irq();
	__set_MSP(*((volatile uint32_t*)FIRMWARE_START_ADDRESS)); // указатель на вершину стека _estack
	GoToApp();
}

extern "C" int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t*>(ptr), len, HAL_MAX_DELAY);
    return len;
}

void dump(uint8_t* ptr, int size)
{
    for (int i = 0; i < size; i++)
    {
        printf("%02X ", ptr[i]);
    }
    printf("\n");
}

uint64_t conv_4byte_ptr_to_u64(uint8_t * ptr)
{
	uint64_t res = (uint64_t)(ptr[3]<<24) + (uint64_t)(ptr[2]<<16) + (uint64_t)(ptr[1]<<8) + (uint64_t)ptr[0];
	return res;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool is_big_endian(void)
{
	union {
		uint32_t i;
		char c[4];
	} bint = { 0x01020304 };

	return bint.c[0] == 1;
}
bool clear_memory(uint32_t firmware_size)
{
	  	  HAL_FLASH_Unlock();
		  static FLASH_EraseInitTypeDef EraseInitStruct;
		  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
		  EraseInitStruct.PageAddress = FIRMWARE_START_ADDRESS;
		  EraseInitStruct.NbPages = (((firmware_size) % FLASH_PAGE_SIZE) != 0) ? ((firmware_size / FLASH_PAGE_SIZE) + 1) : (firmware_size / FLASH_PAGE_SIZE) ;
		  uint32_t PAGEError;
		  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	  	  //HAL_FLASH_Lock();
		  return HAL_OK == status;
}

bool get_header(FirmwareHeader *header)
{
	memset(header, 0, sizeof(FirmwareHeader));
	if(HAL_UART_Receive(&huart2, (uint8_t *)header, sizeof(FirmwareHeader), 10000)==HAL_OK)
	{
		return check_crc_header(header);
	}
	return false;
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
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /*uint8_t app[] = "APP started";
  HAL_UART_Transmit(&huart2,app,sizeof(app),1000);*/
  //printf("Bootloader started!\n");
  FirmwareHeader header;
  FirmwareBlock block;
  int state = 0;
  uint32_t offset=0;
  /*while (true)
  {
	  switch(state){
	  	  case 0:
	  		  offset = 0;
			  if(get_header(&header)) state = 1;
			  break;
	  	  case 1:
	  		  if(clear_memory((uint32_t)header.blocksize * header.num_blocks)) state = 2;
	  		  break;
	  	  case 2:
	  		  for (int i = header.num_blocks; i >= 0; i--)
	  		  {
	  			  uint8_t *block_buff = new uint8_t [header.blocksize+sizeof(block.crc)];
	  			  block.data = block_buff;
	  			  if (HAL_OK == HAL_UART_Receive(&huart2, block.data, header.blocksize + sizeof(block.crc), 4000)){
	  				  block.crc = block.data[header.blocksize];
	  				  for (int i=0;i<header.blocksize;i++) printf("%02X ", block.data[i]);
	  				  printf("%02X\n",block.crc);
	  				  if (block.crc == gencrc(block.data, header.blocksize))
	  				  {
	  					  uint32_t local_offset = 0;

	  					  for (int j = 0; j < header.blocksize/WORD; j++)
	  					  {

	  						  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) (FIRMWARE_START_ADDRESS + offset), conv_4byte_ptr_to_u64(block.data + local_offset));
	  						  offset += 4;
	  						  local_offset += 4;
	  					  }
	  					  printf("OK\n");
	  				  }else {
	  					  printf("block crc err got %02X calc %02X\n",block.crc, gencrc(block.data, header.blocksize));
	  					  state = 0;
	  				  	  HAL_FLASH_Lock();
	  					  break;
	  				  }

	  			  }else
	  			  {
	  				  printf("get block err\n");
	  				  state = 0;
	  			  	  HAL_FLASH_Lock();
	  				  break;
	  			  }
	  			  //dump(block.data, header.blocksize);
	  			  delete []block_buff;
	  		  }
		  	  HAL_FLASH_Lock();
		  	  state = 3;
	  		  break;
	  	  case 3:
			  printf("START THE APP\n");
			  AppStart();
	  		  break;
	  }
  }*/
  if(HAL_UART_Receive(&huart2,(uint8_t *)&header,sizeof(FirmwareHeader),10000)==HAL_OK)
  {
	  //GOT HADER FLASH MEMORY
	  //dump((uint8_t *)&header,sizeof(header));
	  if (check_crc_header(&header))
	  {
		  //printf("CRC OK\n");
		  if(clear_memory((uint32_t)header.blocksize * header.num_blocks))
		  {

			  //printf("clear mem OK\n");
			  for (int i = header.num_blocks; i >= 0; i--)
			  		{
			  			uint8_t *block_buff = new uint8_t [header.blocksize+sizeof(block.crc)];
			  			block.data = block_buff;
			  			if (HAL_OK == HAL_UART_Receive(&huart2, block.data, header.blocksize + sizeof(block.crc), 1000)){
			  				block.crc = block.data[header.blocksize];
			  				for (int i=0;i<header.blocksize;i++) printf("%02X ", block.data[i]);
			  				printf("%02X\n",block.crc);
							if (block.crc == gencrc(block.data, header.blocksize))
							{
								uint32_t local_offset = 0;

								for (int j = 0; j < header.blocksize/WORD; j++)
								{

									HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) (FIRMWARE_START_ADDRESS + offset), conv_4byte_ptr_to_u64(block.data + local_offset));
									offset += 4;
									local_offset += 4;
								}
								printf("OK\n");
							}else {
								printf("block crc err got %02X calc %02X\n",block.crc, gencrc(block.data, header.blocksize));
							}

			  			}else
			  			{
			  				printf("get block err\n");
			  			}
			  			//dump(block.data, header.blocksize);
			  			delete []block_buff;

			  		}
			  printf("START THE APP\n");
			  AppStart();
		  }else{
			  printf("clear mem not success\n");
		  }
	  }else
	  {
		  printf("wrong crc header\n");
	  }
  }else {
	  printf("No data\n");
	  AppStart();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(2000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
