/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bldc.h"
#include "FlashPROM.h"
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
// flash as eeprom
uint32_t res_addr = 0;
myBuf_t flash_buf[BUFFSIZE] = {0,};

// adc
uint16_t adc = 0;

// button
uint8_t short_state = 0;
uint8_t long_state = 0;
uint32_t time_key1 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef uint16_t filter_data_t;
typedef uint32_t filter_data_size_t;

/**
 * @brief init as `{0}`
 */
typedef struct {
    filter_data_t buf[3];
    filter_data_size_t indx;
} filter_median3_t;


/**
 * @brief Median filter. Select average from stored 2 values and new one
 * @param data init as `{0}`
 */
filter_data_t filter_median3 (filter_median3_t* data, filter_data_t new_data){
    data->buf[data->indx] = new_data;
    if (++data->indx >= 3) data->indx = 0;
    filter_data_t a = data->buf[0], b = data->buf[1], c = data->buf[2];
    return (a < b) ? ((b < c) ? b : ((c < a) ? a : c)) : ((a < c) ? a : ((c < b) ? b : c));
}

// true if empty
uint8_t check_empty_buf (myBuf_t buf[BUFFSIZE]){
  for (uint32_t i = 0; i < BUFFSIZE; i++){
    if (buf[i] != 0)
      return 0;
  }
  return 1;
}

void set_halls_pos (myBuf_t buf[BUFFSIZE]){
  hall_pos_t* pos = BLDC_get_ptr_pos();
  pos->a = buf[0];
  pos->b = buf[1];
  pos->c = buf[2];
}

void save_halls_pos (myBuf_t buf[BUFFSIZE]){
  hall_pos_t* pos = BLDC_get_ptr_pos();
  buf[0] = pos->a;
  buf[1] = pos->b;
  buf[2] = pos->c;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // ----- RUN ONCE -----
  erase_flash();
  while (1){}
  // ----- END ONCE ------

  // search and read last data
  res_addr = flash_search_adress(STARTADDR, BUFFSIZE * DATAWIDTH);
  read_last_data_in_flash(flash_buf);
  if ( ! check_empty_buf(flash_buf)){
    set_halls_pos(flash_buf);
  }


  BLDC_init();

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);

  filter_median3_t filter = {0};


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    adc = filter_median3(&filter, (uint16_t) HAL_ADC_GetValue(&hadc1));
    
    uint16_t speed = map(adc, 0, 4096, 0, BLDC_get_max_speed());
    BLDC_set_speed(speed);

    // button
    uint32_t ms = HAL_GetTick();
    uint8_t key1_state = HAL_GPIO_ReadPin(button_combination_GPIO_Port, button_combination_Pin); // подставить свой пин

    if(key1_state == 0 && !short_state && (ms - time_key1) > 50) 
    {
      short_state = 1;
      long_state = 0;
      time_key1 = ms;
    }
    else if(key1_state == 0 && !long_state && (ms - time_key1) > 2000) 
    {
      long_state = 1;
      // действие на длинное нажатие
      save_halls_pos(flash_buf);
      write_to_flash(flash_buf);
    }
    else if(key1_state == 1 && short_state && (ms - time_key1) > 50) 
    {
      short_state = 0;
      time_key1 = ms;

      if(!long_state)
      {
        // действие на короткое нажатие
        BLDC_next_hall_pos_combination();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
