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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UartRingbuffer_multi.h"
#include "ESP8266_HAL.h"
#include "parse.h"
#include <string.h>
#include <stdio.h>
#include <boot.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//
#define SSID "Nguyen"
#define PASSWORD "0935935516"
#define pc_uart &huart2
#define wifi_uart &huart1
//
#define SECTOR_CURRENT 0x08005000
#define SECTOR_A 0x08008000
#define SECTOR_B 0x0800B000

//
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char recv_buf[2048] = { 0 };
char lat_ver[128] = { 0 };
int lat_ver0;
int lat_ver2;
char fw_buf[1024] = { 0 };
char version_buf[30]= {0};
//
int CountForFirmwareSlot =1 ;
int TriggerBase=1;
int BL_Version[2] = { 0 , 0 }; //MAJOR and MINOR 0.0
uint32_t current_sector  ;
//
uint32_t *FWSelect = (uint32_t *)0x0800E000;
//
uint32_t *ReadSlotForFirmware = (uint32_t *)0x0800E400;
uint32_t *TempForCountFirmware = (uint32_t *)0x0800E800;
//
//
int CountForCompare = 0;
uint32_t *BLversion0 = (uint32_t *)0x0800F000;
uint32_t *BLversion1 = (uint32_t *)0x0800F400;
/*if sector_select = 0 it will run at BOOT_BASE
 * 	 sector_select = 1 it will run at SECTOR_A
 * 	 sector_select = 2 it will run at SECTOR_B
 */

/*ví dụ v�? cách chỉnh vtor
 * void jump_to_sector(uint32_t sector_address) {
    // Cập nhật VTOR để tr�? đến bảng vectơ ngắt của sector
    SCB->VTOR = sector_address;

    // Lấy địa chỉ của reset handler
    uint32_t* reset_handler_address = (uint32_t*)(sector_address + 4);

    // G�?i reset handler
    void (*reset_handler)() = (void*)reset_handler_address;
    reset_handler();
}
 *


*/



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
//
void Flash_Erase(uint32_t address)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.Banks =1;
    EraseInitStruct.NbPages =1;
    EraseInitStruct.PageAddress = address;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    uint32_t pageerr;
    HAL_FLASHEx_Erase(&EraseInitStruct, &pageerr);
    HAL_FLASH_Lock();
}
void Flash_Write_Int(uint32_t address,uint32_t value)
{   HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address , value);
    HAL_FLASH_Lock();

}
void print_pc(char *str) {
	Uart_sendstring(str, pc_uart);
}
//
void print_pc_version( ){
	sprintf(version_buf , "%d.%d\n", BL_Version[0] , BL_Version[1]);
	print_pc(version_buf);
}
//
int count_string(char *string, char search) {
   int count = 0;
   while (*string != search) {
       count++;
       string++;
   }
   return count;
}

void parse_character_strings (){
	int number_of_major= count_string(lat_ver,'.');
	int number_of_minor= count_string(&lat_ver[number_of_major+1],'C');
	char temp1[10];
	for (int i = 0 ; i <number_of_major ; i++ ){
		temp1[i]=lat_ver[i];
	}
	temp1[number_of_major] = '\0';

	char temp2[10];
	for (int i = 0 ; i <number_of_minor ; i++ ){
		temp2[i]=lat_ver[i+number_of_major+1];
		}
	temp2[number_of_minor] = '\0';
	lat_ver0=atoi(temp1);
	lat_ver2=atoi(temp2);

}
void compare_version(){
	//lat_ver0=(int)lat_ver[0]-48;
	//lat_ver2=(int)lat_ver[2]-48;
	//tim hieu ham atoi() trong stdlib de chuyen chuoi thành số
	BL_Version[0]=*BLversion0;
	BL_Version[1]=*BLversion1;
	parse_character_strings ();
	if (lat_ver0>BL_Version[0]){
	        BL_Version[0] = lat_ver0;
	        Flash_Erase(0x0800F000);
	        Flash_Write_Int(0x0800F000,BL_Version[0]);
	        CountForCompare++;
	}
	if (lat_ver2>BL_Version[1]){
	        BL_Version[1] = lat_ver2;
	        Flash_Erase(0x0800F400);
	        Flash_Write_Int(0x0800F400,BL_Version[1]);
	        CountForCompare++;
	  }

	if (CountForCompare>0){
			HAL_Delay(1000);
			print_pc("Got higher version\n");
			print_pc("Version:");
			print_pc_version();
			CountForFirmwareSlot++;
	}
	if (CountForCompare==0){
			HAL_Delay(1000);
			print_pc ("Version on the server is not higher\n");
			print_pc ("Returning to application program...");
			TriggerBase++;



	}

}
//


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  Ringbuf_init();
  print_pc("\r\nBootloader initiated\r\n");
  ESP_Init(SSID, PASSWORD);
  print_pc("Connected to wifi\r\n");



  //get version
  memset(recv_buf, 0, sizeof(recv_buf)); // make sure recv_buf is clear
  ESP_Get_Latest_Version((uint8_t*) recv_buf);
  memset(lat_ver, 0, sizeof(lat_ver));
  version_parse(lat_ver, recv_buf);
  print_pc("Got latest file name\r\n");
  compare_version();




  if (*TempForCountFirmware != 0x00000001){
	  Flash_Erase(0x0800E400);
	  Flash_Write_Int(0x0800E400,0x00000001);
	  //
	  Flash_Erase(0x0800E800);
	  Flash_Write_Int(0x0800E800,0x00000001);
  }// này là cờ flag tại địa chỉ 0x0800E800 nếu giá trị tại 0x0800E400 = 1 thì bật cờ này lên để ko cần flash lại E400 =1 nữa
  if (CountForFirmwareSlot ==2 ){
	  CountForFirmwareSlot=1;
	  uint32_t check = *ReadSlotForFirmware + 1 ;
	  Flash_Erase(0x0800E400);
	  Flash_Write_Int(0x0800E400,check);
  }
  //
  if (*ReadSlotForFirmware%2==0){
	  Flash_Erase(0x0800E000);
	  Flash_Write_Int(0x0800E000,0x00000002);
  	}
  else if (*ReadSlotForFirmware%2!=0){
	  Flash_Erase(0x0800E000);
	  Flash_Write_Int(0x0800E000,0x00000001);
  	}
  if (TriggerBase==2){
	  Flash_Erase(0x0800E000);
	  Flash_Write_Int(0x0800E000,0x00000000);
  }

  /*#define SECTOR_CURRENT 0x08005000
	#define SECTOR_A 0x08008000
	#define SECTOR_B 0x0800B000

   */
	HAL_Delay(1000);

  if ( *FWSelect ==0x00000000){
	  current_sector=SECTOR_CURRENT;
  }
  if ( *FWSelect ==0x00000001){
   	  current_sector=SECTOR_A;
     }
  if ( *FWSelect ==0x00000002){
   	  current_sector=SECTOR_B;
     }

  // chua xu li
  jump_to_app(current_sector);

  // chua xu li


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
