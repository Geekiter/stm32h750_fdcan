/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FDCAN_FilterTypeDef sFilterConfig1;
FDCAN_FilterTypeDef sFilterConfig2;

void fdcan1_config(void)
{
  sFilterConfig1.IdType = FDCAN_STANDARD_ID;
  sFilterConfig1.FilterIndex = 0;
  sFilterConfig1.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig1.FilterID1 = 0;
  sFilterConfig1.FilterID2 = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig1);

  sFilterConfig1.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig1.FilterIndex = 0;
  sFilterConfig1.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig1.FilterID1 = 0;
  sFilterConfig1.FilterID2 = 0;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig1);

	/* Configure global filter to reject all non-matching frames */
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	
	/* Configure Rx FIFO 0 watermark to 2 */
  //HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 2);

  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);

  /* Configure and enable Tx Delay Compensation, required for BRS mode.
        TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
        TdcFilter default recommended value: 0 */
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1, 0);
  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

  HAL_FDCAN_Start(&hfdcan1);
}

void fdcan2_config(void)
{
  sFilterConfig2.IdType = FDCAN_STANDARD_ID;
  sFilterConfig2.FilterIndex = 0;
  sFilterConfig2.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig2.FilterID1 = 0;
  sFilterConfig2.FilterID2 = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig2);

  sFilterConfig2.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig2.FilterIndex = 0;
  sFilterConfig2.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig2.FilterID1 = 0;
  sFilterConfig2.FilterID2 = 0;
	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig2);

	/* Configure global filter to reject all non-matching frames */
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);

  /* Configure and enable Tx Delay Compensation, required for BRS mode.
        TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
        TdcFilter default recommended value: 0 */
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, hfdcan2.Init.DataPrescaler * hfdcan2.Init.DataTimeSeg1, 0);
  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);

  HAL_FDCAN_Start(&hfdcan2);
}


FDCAN_RxHeaderTypeDef RxHeader1;
FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t RxData1[64];
uint8_t RxData2[64];

////0xF0000 => 64
uint8_t dlc2len[]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
uint8_t can_dlc2len(uint32_t RxHeader_DataLength)
{
  return dlc2len[RxHeader_DataLength>>16];
}

uint8_t cnt = 0;
uint8_t brs[] = {'-', 'B'};
uint8_t esi[] = {'-', 'E'};
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
		
    //memset(&RxHeader1, 0, sizeof(FDCAN_RxHeaderTypeDef));	//if use, lose frame
        
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, RxData1);
		if (hfdcan->Instance == FDCAN1) {
		  printf("fdcan1, ");
	  } else if (hfdcan->Instance == FDCAN2) { 
		  printf("fdcan2, ");
	  } else {
		}
    printf("0x%8X, %02d, %c, %c:",RxHeader1.Identifier, 
                                  can_dlc2len(RxHeader1.DataLength), 
                                  brs[RxHeader1.BitRateSwitch>>20 & 0x1],
                                  esi[RxHeader1.ErrorStateIndicator>>31 & 0x1]);
    for(cnt = 0; cnt < can_dlc2len(RxHeader1.DataLength); cnt++) {
      printf(" %02X", RxData1[cnt]);
    }
    printf("\n\r");
  }
}

FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_TxHeaderTypeDef TxHeader3;
uint8_t TxData1[64];
uint8_t TxData2[64];

typedef struct {
	uint8_t flag;
	uint8_t index;
  FDCAN_TxHeaderTypeDef TxHeader[16];
  uint8_t TxData[64*16];
} FDCAN_SendFailTypeDef;

FDCAN_SendFailTypeDef fdcan1_send_fail = {0};
FDCAN_SendFailTypeDef fdcan2_send_fail = {0};

void fdcan1_transmit(uint32_t can_id, uint32_t DataLength, uint8_t tx_data[])
{
  TxHeader1.Identifier = can_id;
  TxHeader1.IdType = FDCAN_EXTENDED_ID;
  if(can_id < 0x800) {  //exactly not right
    TxHeader1.IdType = FDCAN_STANDARD_ID;
  }
  TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader1.DataLength = DataLength;
  TxHeader1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader1.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader1.FDFormat = FDCAN_FD_CAN;
  TxHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader1.MessageMarker = 0;  //Tx Event FIFO Use
  if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, tx_data) != HAL_OK) {
		//memset(&fdcan1_send_fail.TxHeader[fdcan1_send_fail.index], 0, sizeof(FDCAN_TxHeaderTypeDef));
		//memset(&fdcan1_send_fail.TxData[64*fdcan1_send_fail.index], 0, 64);
    memcpy(&fdcan1_send_fail.TxHeader[fdcan1_send_fail.index], &TxHeader1, sizeof(FDCAN_TxHeaderTypeDef));
    memcpy(&fdcan1_send_fail.TxData[64*fdcan1_send_fail.index], tx_data, can_dlc2len(DataLength));
		fdcan1_send_fail.index = (fdcan1_send_fail.index + 1) & 0x0F;	//0~15
		fdcan1_send_fail.flag = 1;
  } 
}

void fdcan2_transmit(uint32_t can_id, uint32_t DataLength, uint8_t tx_data[])
{
  TxHeader2.Identifier = can_id;
  TxHeader2.IdType = FDCAN_EXTENDED_ID;
  if(can_id < 0x800) {  //exactly not right
    TxHeader2.IdType = FDCAN_STANDARD_ID;
  }
  TxHeader2.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader2.DataLength = DataLength;
  TxHeader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader2.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader2.FDFormat = FDCAN_FD_CAN;
  TxHeader2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader2.MessageMarker = 0;  //Tx Event FIFO Use
  if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, tx_data) != HAL_OK) {
		//memset(&fdcan2_send_fail.TxHeader[fdcan2_send_fail.index], 0, sizeof(FDCAN_TxHeaderTypeDef));
		//memset(&fdcan2_send_fail.TxData[64*fdcan2_send_fail.index], 0, 64);
    memcpy(&fdcan2_send_fail.TxHeader[fdcan2_send_fail.index], &TxHeader2, sizeof(FDCAN_TxHeaderTypeDef));
    memcpy(&fdcan2_send_fail.TxData[64*fdcan2_send_fail.index], tx_data, can_dlc2len(DataLength));
		fdcan2_send_fail.index = (fdcan2_send_fail.index + 1) & 0x0F;	//0~15
		fdcan2_send_fail.flag = 1;
  } 
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  //__HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_BUS_OFF);
  if(hfdcan->Instance == FDCAN1) {
    MX_FDCAN1_Init();
    fdcan1_config();
  } else if(hfdcan->Instance == FDCAN2) {
    MX_FDCAN2_Init();
    fdcan2_config();
  } else {
  }
}

uint8_t tim6_flag = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
    if(htim->Instance == TIM6) {
        tim6_flag = 1;
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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	fdcan1_config();
	fdcan2_config();
	
	for(uint8_t i = 0; i < 64; i++) {
    TxData1[i] = i;
    TxData2[i] = i;
  }
		
	HAL_TIM_Base_Start_IT(&htim6);
	
	uint32_t count = 0;
  uint32_t cnt_100us = 0;
  uint32_t cnt_500us = 0;
	uint32_t i = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
    if(tim6_flag && count < 1000) {
      tim6_flag = 0;
      TxData1[0] = count >> 8 & 0xFF;
      TxData1[1] = count & 0xFF;

      ++cnt_100us;
      cnt_500us = cnt_100us / 5;
      if(cnt_500us && (cnt_100us%5==0) ) {
        switch(cnt_500us) {
          case 1: fdcan1_transmit(0x108, FDCAN_DLC_BYTES_64, TxData1); 
                   fdcan1_transmit(0x101, FDCAN_DLC_BYTES_64, TxData1);
                   fdcan1_transmit(0x102, FDCAN_DLC_BYTES_64, TxData1);
                   fdcan1_transmit(0x103, FDCAN_DLC_BYTES_64, TxData1);
                   fdcan1_transmit(0x104, FDCAN_DLC_BYTES_64, TxData1);
                   fdcan1_transmit(0x105, FDCAN_DLC_BYTES_64, TxData1);
                   fdcan1_transmit(0x106, FDCAN_DLC_BYTES_64, TxData1);
                   fdcan1_transmit(0x107, FDCAN_DLC_BYTES_64, TxData1);
									 fdcan1_transmit(0x12345678, FDCAN_DLC_BYTES_64, TxData1);
									 fdcan1_transmit(0x12345671, FDCAN_DLC_BYTES_64, TxData1);
									 fdcan1_transmit(0x12345672, FDCAN_DLC_BYTES_64, TxData1);
									 fdcan1_transmit(0x12345673, FDCAN_DLC_BYTES_64, TxData1);
									 fdcan1_transmit(0x12345674, FDCAN_DLC_BYTES_64, TxData1);
									 fdcan1_transmit(0x12345675, FDCAN_DLC_BYTES_64, TxData1);
									 fdcan1_transmit(0x12345676, FDCAN_DLC_BYTES_64, TxData1);
									 fdcan1_transmit(0x12345677, FDCAN_DLC_BYTES_64, TxData1); 
									 //fdcan1_transmit(0x12345670, FDCAN_DLC_BYTES_64, TxData1); 
									 break;
          case 17: /* next send */ break;
          case 18: break;
					case 19: break;
          case 20: ++count; cnt_100us = 0; break; //10ms
        }
      } else {  //fail retransmission once
				
        if(fdcan1_send_fail.flag && cnt_500us > 17) {	//can't conflict with normal send
					for(i = 0; i < fdcan1_send_fail.index; i++) {
						fdcan1_transmit(fdcan1_send_fail.TxHeader[i].Identifier, 
														  fdcan1_send_fail.TxHeader[i].DataLength,
														  &fdcan1_send_fail.TxData[64*i]);
					}
					fdcan1_send_fail.index = 0;	
          fdcan1_send_fail.flag = 0;	//maybe send 4 times or more
        }
				
				if(fdcan2_send_fail.flag && cnt_500us > 16) {
					for(i = 0; i < fdcan2_send_fail.index; i++) {
						fdcan2_transmit(fdcan2_send_fail.TxHeader[i].Identifier, 
														  fdcan2_send_fail.TxHeader[i].DataLength,
														  &fdcan2_send_fail.TxData[64*i]);
					}
					fdcan2_send_fail.index = 0;
          fdcan2_send_fail.flag = 0;
        }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch,FILE *f)
{
    HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,100);     
    return 0;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
