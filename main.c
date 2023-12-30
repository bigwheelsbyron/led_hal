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
#include "math.h"
#include "stdio.h"
#include <stdlib.h>
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define WRAP (1 << 0)
#define REVERSE (1 << 1)
#define ON (1 << 2)

struct LEDRange {
    int startLED;
    int endLED;
};

struct Color {
    int Red;
    int Green;
    int Blue;
    int Brightness;
};


#define MAX_LED 200
#define USE_BRIGHTNESS 1
#define PI 3.14159265
#define TIME_PERIOD 100 // Time period in milliseconds
#define BPM 120       // Beats per minute
#define BPS (BPM / 60.0) // Beats per second, use float for accuracy
#define PERIODS_PER_BEAT (1000 / (BPS * TIME_PERIOD))


/* first ring 0-12  */
#define RING_ONE_START 0
#define RING_ONE_END 13
#define RING_ONE_LENGTH = RING_ONE_END - RING_ONE_START;
struct LEDRange ringOne = {RING_ONE_START, RING_ONE_END};

/* second ring 16-21  */
#define RING_TWO_START 16
#define RING_TWO_END 22
#define RING_TWO_LENGTH = RING_TWO_END - RING_TWO_START;
struct LEDRange ringTwo = {RING_TWO_START, RING_TWO_END};

#define RING_THREE_START 23
#define RING_THREE_END 34
#define RING_THREE_LENGTH = RING_THREE_END - RING_THREE_START;
struct LEDRange ringThree = {RING_THREE_START, RING_THREE_END};


#define RING_FOUR_START 34
#define RING_FOUR_END 200
#define RING_FOUR_LENGTH = RING_FOUR_END - RING_FOUR_START;
struct LEDRange ringFour = {RING_FOUR_START, RING_FOUR_END};

/* first speaker 13-15  */
#define STRIP_ONE_START 13
#define STRIP_ONE_END 16
#define STRIP_ONE_LENGTH = STRIP_ONE_END - STRIP_ONE_START;
struct LEDRange stripOne = {STRIP_ONE_START, STRIP_ONE_END};

/* second speaker 22  */
#define STRIP_TWO_START 22
#define STRIP_TWO_END 23
#define STRIP_TWO_LENGTH = STRIP_TWO_END - STRIP_TWO_START;
struct LEDRange stripTwo = {STRIP_TWO_START, STRIP_TWO_END};


struct Color WHITE = {255, 255, 255, 45};
struct Color OFF = {0, 0, 0, 0};
struct Color RED = {255, 0, 0, 45};
struct Color BLUE = {0, 0, 255, 45};
struct Color GREEN = {0, 255, 0, 45};

uint16_t pwmData[(24*MAX_LED)+50];
volatile int datasentflag = 0;
uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

void
Set_LED (int LEDnum, int Red, int Green, int Blue, int Brightness)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
	float angle = 90-Brightness;  // in degrees
	angle = angle*PI / 180;  // in rad
	LED_Mod[LEDnum][0] = (LED_Data[LEDnum][0])/(tan(angle));
	LED_Mod[LEDnum][1] = (LED_Data[LEDnum][1])/(tan(angle));
	LED_Mod[LEDnum][2] = (LED_Data[LEDnum][2])/(tan(angle));
	LED_Mod[LEDnum][3] = (LED_Data[LEDnum][3])/(tan(angle));

}


void
Strip_FullColour(struct LEDRange range, struct Color colour) {
    for (int i = range.startLED; i < range.endLED; i++) {
        Set_LED(i, colour.Red, colour.Green, colour.Blue, colour.Brightness);
    }
}
void
Ring_FullColor(struct LEDRange range, struct Color colour)
{
	Strip_FullColour(range, colour);
}


void
Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}



void
WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 60;  // 2/3 of 90
			}

			else pwmData[indx] = 30;  // 1/3 of 90

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
//	while (!datasentflag){};
//	datasentflag = 0;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag=1;
}

void Thump(struct LEDRange range, struct Color color, int *counter, int *flag, int frequency) {
    int c = *counter;

    // Check if ON flag is set in *flag
    if (*flag & ON) {
        Strip_FullColour(range, color);
    } else {
        Strip_FullColour(range, OFF);
    }

    // Toggle the flag at the specified frequency
    int q = (int)floor(1000 / frequency);
    if (c % q < 10) {
        *flag ^= ON; // Toggle the ON flag
    }
}



int*
Spin(struct LEDRange range, struct Color *colours, int coloursCount, int *counter, int *flag, int heads, int direction, int frequency)
{

    //current counter position
    int c = *counter % (1000 / frequency);
    int rangeLength = range.endLED - range.startLED;
    int head;
    int headCounter = 0;
    int colourCounter = 0;
    int *headsArray = (int *)malloc(heads * sizeof(int));
    int split = (int)floor(rangeLength / heads);


    // get the normal spread
    if (direction == 0){
        int min_periods = (int)floor((1000 / frequency) / rangeLength);
        for (int i = 0; i < heads; i++){
            head = (int) floor(c / min_periods);
            head = head + (split * headCounter);
            head = head % rangeLength;
            head = head + range.startLED;
            headCounter += 1;
            int cPos = colourCounter % coloursCount;
            struct Color colour = colours[cPos];
            if (head < range.endLED && head >=range.startLED) Set_LED(head, colour.Red, colour.Green, colour.Blue,colour.Brightness);
            headsArray[i] = head;
            colourCounter += 1;
        }


    } else {
    	c = (c-990)*-1;
        int min_periods = (int)floor((1000 / frequency) / rangeLength);
        for (int i = 0; i < heads; i++){
            head = (int) floor(c / min_periods);
            head = head + (split * headCounter);
            head = head % rangeLength;
            head = head + range.startLED;
            headCounter += 1;
            int cPos = colourCounter % coloursCount;
            struct Color colour = colours[cPos];
            if (head < range.endLED && head >=range.startLED) Set_LED(head, colour.Red, colour.Green, colour.Blue,colour.Brightness);
            headsArray[i] = head;
            colourCounter += 1;
        }
    }

   return headsArray;


}

int*
Commet(struct LEDRange range, struct Color *colours, int coloursCount, int *counter, int *flag, int heads, int direction, int frequency, int tail)
{

	int *headers = Spin(range, colours, coloursCount, counter, flag, heads, direction, frequency);
    int colourCounter = 0;

    if (direction == 0){
        for (int i = 0; i < heads; i++){
        	int header = headers[i];
            int cPos = colourCounter % coloursCount;
            struct Color colour = colours[cPos];
        	if (header < range.endLED && header >=range.startLED) Set_LED(header, colour.Red, colour.Green, colour.Blue, 45);
    		for (int j = 0; j < (tail +1); j++){
    			int pos = header - j;

    			if((*flag & WRAP) != 0){
        			if (pos < range.startLED){
        				pos = range.endLED - (range.startLED - pos - 1);
        			}
    			}

    			int b = (int)floor(colour.Brightness);
    			b = b - (45 * j / (tail + 1));
    			if (pos < range.endLED && pos >=range.startLED) Set_LED(pos, colour.Red, colour.Green, colour.Blue, b);

    		}
            colourCounter += 1;
        }
    } else {
        for (int i = 0; i < heads; i++){
        	int header = headers[i];
            int cPos = colourCounter % coloursCount;
            struct Color colour = colours[cPos];
    		for (int j = 0; j < (tail +1); j++){
    			int pos = header + j;
    			if ((*flag & WRAP) != 0) {
    			    if (pos > range.endLED) {
    			        pos = range.startLED - (range.endLED - pos) - 1;
    			    }
    			}
    			int b = (int)floor(colour.Brightness);
    			b = b - (45 * j / (tail + 1));
    			if (pos < range.endLED && pos >=range.startLED) Set_LED(pos, colour.Red, colour.Green, colour.Blue, b);

    		}
            colourCounter += 1;
        }
    }

    return headers;

}

void
Mixer(struct LEDRange range, struct Color *colours, int coloursCount, int *counter, int *flag, int frequency, int tail) {
    for (int i = range.startLED; i < range.endLED; ++i) {
        Set_LED(i, 0, 0, 0, 0);
    }
    int rangeLength = range.endLED - range.startLED; //34 - 23 = 11
    int middle = range.startLED + (int) floor(rangeLength / 2); // 23 + 11 /2 = 28
    struct LEDRange aRange = {range.startLED, middle};
    struct LEDRange bRange = {middle, range.endLED};
    int *aHeader = NULL;
    int *bHeader = NULL;


    if (*flag & REVERSE){
        aHeader = Commet(aRange, colours, coloursCount, counter, flag, 1, 0, 1, 2);
        bHeader = Commet(bRange, colours, coloursCount, counter, flag, 1, 1, 1, 2);
    } else {
        aHeader = Commet(aRange, colours, coloursCount, counter, flag, 1, 1, 1, 2);
        bHeader = Commet(bRange, colours, coloursCount, counter, flag, 1, 0, 1, 2);
    }


    if (*bHeader == middle && (*counter % 1000) == 990) {
    	*flag ^= REVERSE; // Toggle the ON flag?
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    }

    if (*bHeader == (range.endLED-1) && (*counter % 1000) == 990) {
    	*flag ^= REVERSE; // Toggle the ON flag?
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    }

    free(aHeader);
    free(bHeader);








}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
#include "main.h"


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
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();

    /* USER CODE BEGIN 2 */
    int counter = 0;
    int stripOneFlag = 1;
    int stripTwoFlag = 1;
    int ringOneFlag = 1;
    int ringTwoFlag = 1;
    int ringThreeFlag = 1;
    int ringFourFlag = 1;
    ringOneFlag |= WRAP;
    ringThreeFlag |= REVERSE;

    // Start time for the loop
    uint32_t startTime = HAL_GetTick();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t currentTime = HAL_GetTick();
        if (currentTime - startTime >= TIME_PERIOD)
        {
            // Reset the start time for next iteration
            startTime = currentTime;

            // LED control code
            for (int i = 0; i < MAX_LED; ++i) {
                Set_LED(i, 0, 0, 0, 0);
            }
            struct Color colours [] = {BLUE, GREEN};
            struct Color c [] = {RED};
            Thump(stripOne, WHITE, &counter, &stripOneFlag, 8);
            Thump(stripTwo, WHITE, &counter, &stripTwoFlag, 8);

            Thump(ringOne, WHITE, &counter, &ringOneFlag, 8);
            Thump(ringTwo, WHITE, &counter, &ringTwoFlag, 8);
            Thump(ringThree, WHITE, &counter, &ringThreeFlag, 8);
            Thump(ringFour, WHITE, &counter, &ringFourFlag, 8);
//
//            int *headers = Commet(ringOne, colours, 2, &counter, &ringOneFlag, 2, 1, 1, 2);
//            free(headers);
//            headers = Spin(ringTwo, c, 1, &counter, &ringOneFlag, 2, 2, 1);
//            free(headers);
//            Mixer(ringThree, colours, 2, &counter, &ringThreeFlag, 1, 0);

            WS2812_Send(); // Update LED strip

            // Update counter and reset if necessary
            counter += TIME_PERIOD;
            if (counter >= 1000) counter = 0;
        }
        // Additional tasks can be performed here if needed

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 72-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  
