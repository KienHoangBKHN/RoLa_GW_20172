/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "STM32_SX1278.h"
/* USER CODE BEGIN Includes */
pack packet_sent, packet_received, ACK;

char msgid[30],msgauc[30],msg[30],msgacc[30];
uint8_t Node_Addr[4], val = 1;
uint8_t number = 0, i=1 ;
uint8_t GW_Addr = 9;
uint8_t state;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void UART_print(char msg[])
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&msg[0], strlen(msg),100);
}
// Phien lam viec cua Gateway: Add dia chi Node
void check_NodeID(char*msgid)
{
	  state = receivePacketTimeoutACK(&hspi1, 2000);
    if( state == 0 )
    {
			for (uint8_t i = 0; i < packet_received.length; i++)
    {
      msgid[i] = (char)packet_received.data[i];
    }
			UART_print("\nNodeID: ");
		  UART_print(msgid);
		if (msgid[0] >= '1' && msgid[0] <= '9')
		{
	  Node_Addr[number] = (uint8_t)msgid[0];
		number++;
		UART_print("\nAdd NodeID Succesful!\n");
	  }
		}
    else
		UART_print("\nAdd NodeID Failed!\n");
}
//Phien lam viec cua Gateway: Kiem tra ban tin "AuC"
//Kiem tra ban tin xac thuc co dung dang
 bool check_msgAuC(char *msgauc)
{
	UART_print("\nCheck msg Auc ?");
	state = receivePacketTimeoutACK(&hspi1, 2000);
	if( state == 0 )
    	{
			for (uint8_t i = 0; i < packet_received.length; i++)
			{
				msgauc[i] = (char)packet_received.data[i];
			}
		if((msgauc[0]=='s')&&(msgauc[1]=='a')&&(msgauc[2]=='n')&&(msgauc[3]=='s')&&(msgauc[4]=='l')&&(msgauc[5]=='a')&&(msgauc[6]=='b'))
			{
				UART_print("\nMessage AuC: ");
				UART_print(msgauc);
				UART_print("\nCheck: Succesful Authenticaton!");
				return true;
			}
    	}
	else
	{
		UART_print("\nCheck: Failed Authenticaton!");
		return false;
	}
	return false;
 }
// Phien lam viec cua Gateway: Gui ban tin "accept",
void MAC_Process(uint8_t Node_Addr)
{
	UART_print("\n\nStart send msg Acc");
	// Check sending statuscx
	// uint8_t e = sendPacketTimeout_sendtime(&hspi1, Node_Addr,"accept");
	sendPacketTimeout_sendtime(&hspi1, Node_Addr,"accept");
	// Check sending statuscx
  state = receivePacketTimeoutACK(&hspi1, 10000);
  if(state == 0 )
  {
		UART_print("\nSend acc Succesful!");
		for (uint8_t i = 0; i < packet_received.length; i++)
    {
      msg[i] = (char)packet_received.data[i];
    }
		UART_print("\n\nStart receive message from Node");
		UART_print("\nData from Node: ");
		UART_print(msg);
  }
	else
		{
		UART_print("\nSend acc Fail!\n");
    }
  HAL_Delay(2500);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Receive_msg(SPI_HandleTypeDef *hspi, char msg[])
{
 uint8_t val = receivePacketTimeout(hspi, 1000);
		if ( val == 0 )
  {
    for (uint8_t i = 0; i < packet_received.length; i++)
    {
      msg[i] = (char)packet_received.data[i];
    }
		UART_print("\nMessage: ");
		UART_print(msg);
  }
  else {
		UART_print("\nErorr");
  }
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  UART_print("\nStart configuring LoRa module");
    SX1278();
  	// Power ON the module
    ON(&hspi1);
    UART_print("\nSetting Power ON");
    // Set transmission mode and print the result
    setMode(&hspi1, 4);
    UART_print("\nSetting Mode 4");
    // Set header
    setHeaderON(&hspi1);
    UART_print("\nSetting Header ON");
    // Select frequency channel
  	setChannel(&hspi1,0x6C4000);
    UART_print("\nSetting Channel: CH = 11, 433MHz");
    // Set CRC
    setCRC_ON(&hspi1);
    UART_print("\nSetting CRC ON");
    // Select output power (Max, High or Low)
  	setPower(&hspi1,'H');
    UART_print("\nSetting Power: H");
    // Set the node address and print the result
  	setNodeAddress(&hspi1,GW_Addr);
    UART_print("\nSetting Node address: 9");
    // Print a success message
    UART_print("\nLoRa successfully configured\n");
    /* USER CODE END 2 */

    /* Infinite loop */
  	check_NodeID(msgid);
  	HAL_Delay(1500);
  	check_msgAuC(msgauc);
  	HAL_Delay(2500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  MAC_Process(Node_Addr[number]);
	  				if(++number == 4) number = 0;
	  		else number++;
	  		HAL_Delay(1000);
	  UART_print("Hello world\n");
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
