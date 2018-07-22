
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_usart.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



volatile 	uint8_t UART_Buffer[UART_RX_BUFFER_SIZE];
volatile 	uint16_t NumberOfBytesReceive = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void Uart_Init(void);
static void LedInit(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
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
	/* USER CODE BEGIN 2 */
	Uart_Init();
	LedInit();


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//LL_USART_TransmitData8(USART1,'T');
		HAL_GPIO_TogglePin(GPIOJ,(uint16_t)(GPIO_PIN_5 | GPIO_PIN_13));

		HAL_Delay(200);




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

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

static void Uart_Init(void)
{
	LL_USART_InitTypeDef	USART_Handle;
	LL_DMA_InitTypeDef		DMA_RX_Handle;
	LL_DMA_InitTypeDef		DMA_TX_Handle;
	LL_GPIO_InitTypeDef		GPIO_Handle;


	/* Enable all peripheral clock */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);



	/*Config GPIO Pins for UART Function */
	GPIO_Handle.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
	GPIO_Handle.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Handle.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Handle.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Handle.Pull = LL_GPIO_PULL_UP;
	GPIO_Handle.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOA,&GPIO_Handle);



	/* Configure USART */
	LL_USART_StructInit(&USART_Handle);
	USART_Handle.BaudRate = 1000000;
	USART_Handle.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_Handle.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_Handle.OverSampling = LL_USART_OVERSAMPLING_16;
	USART_Handle.Parity = LL_USART_PARITY_NONE;
	USART_Handle.StopBits = LL_USART_STOPBITS_1;
	USART_Handle.TransferDirection = LL_USART_DIRECTION_TX_RX;
	LL_USART_Init(USART1,&USART_Handle);

	/* Clear all of flags */
	USART1->ICR = USART_ICR_PECF|USART_ICR_FECF|USART_ICR_NCF|USART_ICR_ORECF|USART_ICR_IDLECF|USART_ICR_TCCF;

	/* Enable IDLE Interrupt */
	LL_USART_EnableIT_IDLE(USART1);

	/* Enable RX DMA Request */
	LL_USART_EnableDMAReq_RX(USART1);

	/* Enable TX DMA Request */
	LL_USART_EnableDMAReq_TX(USART1);

	HAL_NVIC_SetPriority(USART1_IRQn,0,0);
	NVIC_EnableIRQ(USART1_IRQn);

	/* Enable USART1 */
	LL_USART_Enable(USART1);




	/* Configure DMA for USART RX */
	LL_DMA_StructInit(&DMA_RX_Handle);
	DMA_RX_Handle.Channel 					= LL_DMA_CHANNEL_4;
	DMA_RX_Handle.Direction 				= LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	DMA_RX_Handle.PeriphOrM2MSrcAddress 	= (uint32_t)&USART1->RDR;
	DMA_RX_Handle.MemoryOrM2MDstAddress 	= (uint32_t)UART_Buffer;
	DMA_RX_Handle.NbData 					= UART_RX_BUFFER_SIZE;
	DMA_RX_Handle.MemoryOrM2MDstIncMode 	= LL_DMA_MEMORY_INCREMENT;
	DMA_RX_Handle.Priority 					= LL_DMA_PRIORITY_VERYHIGH;
	LL_DMA_Init(DMA2, LL_DMA_STREAM_2, &DMA_RX_Handle);

	/* Enable DMA2 Stream2 Tranmission Complete Interrupt */
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);

	/* Enable global DMA stream interrupts */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn,1,0);
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);




	/* Configure DMA for USART TX */
	LL_DMA_StructInit(&DMA_TX_Handle);
	DMA_TX_Handle.Channel 					= LL_DMA_CHANNEL_4;
	DMA_TX_Handle.Direction					= LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	DMA_TX_Handle.PeriphOrM2MSrcAddress		= (uint32_t)&USART1->TDR;
	DMA_TX_Handle.MemoryOrM2MDstAddress		= (uint32_t)UART_Buffer;
	DMA_TX_Handle.MemoryOrM2MDstIncMode		= LL_DMA_MEMORY_INCREMENT;
	DMA_TX_Handle.Priority					= LL_DMA_PRIORITY_HIGH;
	LL_DMA_Init(DMA2, LL_DMA_STREAM_7, &DMA_TX_Handle);

	/* Enable DMA2 Stream7 Tranmission Complete Interrupt */
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);

	/* Enable global DMA stream interrupts */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn,1,0);
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);


	/* Enable DMA USART RX Stream */
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);

}


static void LedInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOJ_CLK_ENABLE();


	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13|GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pins : PJ13 PJ5 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
