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
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define morse code values
#define DOT_DURATION 1
#define DASH_TIME 5
#define SPACE_TIME 7
#define MORSE_CODE_LENGTH 5
/* USER CODE END PD */
volatile uint8_t ButtonState = 0;
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "Task3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task4 */
osThreadId_t Task4Handle;
const osThreadAttr_t Task4_attributes = {
  .name = "Task4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
// Define a semaphore handle
SemaphoreHandle_t xSemaphore;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
char decodeMorse(char *morseChar);

/* USER CODE BEGIN PFP */
// Define the Morse code set and corresponding letters
typedef enum {
    DOT,
    DASH,
	SPACE
} qTemp;
// Define the queues and their sizes
#define QUEUE_1_SIZE 15
#define QUEUE_2_1_SIZE 15
#define QUEUE_2_2_SIZE 15

QueueHandle_t xQueue1;
QueueHandle_t xQueue2_1;
QueueHandle_t xQueue2_2;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
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
  /* Start UART reception */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  // Create the semaphore with an initial count of 0
   xSemaphore = xSemaphoreCreateCounting(1, 0);

   // Check if semaphore creation was successful
   if (xSemaphore == NULL) {
       // Handle semaphore creation error
       while (1);
   }


  /* USER CODE BEGIN RTOS_QUEUES */
  // Create the queues
  xQueue1 = xQueueCreate(QUEUE_1_SIZE, sizeof(char));
  xQueue2_1 = xQueueCreate(QUEUE_2_1_SIZE, sizeof(char));
  xQueue2_2 = xQueueCreate(15, sizeof(char));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  Task1Handle = osThreadNew(StartTask01, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(StartTask02, NULL, &Task2_attributes);

  /* creation of Task3 */
  Task3Handle = osThreadNew(StartTask03, NULL, &Task3_attributes);
  /* creation of Task4 */
  Task4Handle = osThreadNew(StartTask04, NULL, &Task4_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);


  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE END Header_StartTask01 */
/* Task 1: Read user input from button and enqueue dots and dashes to queue 1 */
void StartTask01(void *pvParameters) {
    uint32_t press = 0;
    uint32_t release = 0;
    uint32_t duration;
    qTemp qSend;
    bool spaceSent = false;
    char morseChar;

    for (;;) {
        /* Wait for button press on user button (PA0) */
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET && ButtonState == 1) {
        	HAL_Delay(10);
        }

        /* Measure duration of button press */
        press = HAL_GetTick();
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
        	HAL_Delay(10);
            ButtonState = 0;
        }
        release = HAL_GetTick();
        duration = release - press;

        /* Convert duration to dot, dash or end of Morse code */
        if (duration < 1000 && spaceSent == false) {
            qSend = DOT;
        }
        else if (duration < 3000 && spaceSent == false) {
            qSend = DASH;
        }

        else if (spaceSent == true){
        	spaceSent = false;
        	qSend = SPACE;
        }
        else {
            qSend = SPACE;
            spaceSent = true;
        }

        /* Enqueue dot, dash or end of Morse code to xQueue1 */
        xQueueSend(xQueue1, &qSend, 0);
        HAL_Delay(100);
    }

}

/* Task 2: Dequeue dots and dashes from queue 1 and enqueue to queue 2_2 */
void StartTask02(void *pvParameters) {
BaseType_t xStatus;
qTemp qSend;

for (;;) {
    /* Dequeue dot or dash from xQueue1 */
    xStatus = xQueueReceive(xQueue1, &qSend, portMAX_DELAY);
    if (xStatus == pdPASS) {
    	if (qSend != SPACE){
    	xQueueSendToBack(xQueue2_2, &qSend, 0);
    	}
    	xQueueSendToBack(xQueue2_1, &qSend, 0);

        /* Light up LED on GPIOA, PIN 4 or PIN 5 based on whether the dot or dash is a dot or a dash */
        if (qSend == DOT) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        else if (qSend == DASH) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_Delay(500);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        }

    }
}
}


/* Task 3: Dequeue dots and dashes from queue 2_1, decode them and print corresponding letter */
void StartTask03(void *pvParameters) {
    BaseType_t xStatus;
    qTemp qReceive;
    char morseCode[6] = {0};
    int morseIndex = 0;
    for (;;) {
        /* Wait for message from Queue2_1 */
        xStatus = xQueueReceive(xQueue2_1, &qReceive, portMAX_DELAY);

        /* Process Morse code */
        if (xStatus == pdPASS) {
            /* If end of Morse code, decode and print letter */
            if (qReceive == SPACE) {
                char letter = decodeMorse(morseCode);
                printf("%c", letter);
                xSemaphoreGive(xSemaphore);

                morseIndex = 0;
                memset(morseCode, 0, sizeof(morseCode));
            }
            /* Otherwise, store Morse code character */
            else {
                morseCode[morseIndex] = (qReceive == DOT) ? '.' : '-';
                morseIndex++;
                if (morseIndex >= sizeof(morseCode)) {
                    morseIndex = 0;
                    memset(morseCode, 0, sizeof(morseCode));
                }
            }
        }
    }
}


/* Task 4: Dequeue dots and dashes from queue 2_2 and display the entire sequence on LED */

void StartTask04(void *pvParameters) {

    BaseType_t xStatus;
    qTemp qSend;

    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    for (;;) {
        /* Wait for semaphore from Task 2 */


    	while(uxQueueMessagesWaiting(xQueue2_2)>0){
        /* Receive Morse code array from Task 2 */
        xStatus = xQueueReceive(xQueue2_2, &qSend, 0);

        /* Process Morse code array */
        if (xStatus == pdPASS) {
            /* If end of Morse code array, reset index */

            /* Otherwise, process Morse code character */

                if (qSend ==  DOT) {
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                    HAL_Delay(100);
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
                }
                else if (qSend == DASH) {
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                    HAL_Delay(500);
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
                }
                HAL_Delay(500);

        }
    	}
    	xSemaphoreGive(xSemaphore);
    }
}






// Function to decode a morse code string and return corresponding letter
char decodeMorse(char *morseChar)
{
    /* Define a lookup table for decoding morse code */
    static const char *morseTable[] = {".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--.."};

    /* Determine the index of the morse code in the lookup table */
    int i;
    for (i = 0; i < 26; i++)
    {
        if (strcmp(morseChar, morseTable[i]) == 0)
        {
            break;
        }
    }

    /* If morse code is not found in the lookup table, return space */
    if (i == 26)
    {
        return ' ';
    }

    /* Convert index to letter by adding 'A' */
    return i + 'A';
}
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
