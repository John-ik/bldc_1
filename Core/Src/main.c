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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    BLDC_SOFT_STOP = 0,
    BLDC_HARD_STOP = 1,
    BLDC_RUN = 2
} BLDC_Run;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Phases
#define BLDC_PHASE_A TIM_CHANNEL_1
#define BLDC_PHASE_B TIM_CHANNEL_2
#define BLDC_PHASE_C TIM_CHANNEL_3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t hall_state;
uint8_t forward_dir = 1;
uint16_t speed = 0, max_speed = 8800;
BLDC_Run bldc_run = BLDC_SOFT_STOP;


// Must be != 
// If A = 0, B = 1, C = 2 so hall state = 0bCBA 
uint8_t hall_A_pos = 1, hall_B_pos = 0, hall_C_pos = 2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t read_halls(){
  return (HAL_GPIO_ReadPin(hall_C_GPIO_Port, hall_C_Pin) << hall_C_pos)
      |  (HAL_GPIO_ReadPin(hall_B_GPIO_Port, hall_B_Pin) << hall_B_pos)
      |  (HAL_GPIO_ReadPin(hall_A_GPIO_Port, hall_A_Pin) << hall_A_pos);
}

/* 
@brief stop channel to set zero of phase
@param channel BLDC_PHASE_{A,B,C}
 */
void set_A_zero () {
  TIM1->CCR1 = 0;
}
void set_B_zero () {
  TIM1->CCR2 = 0;
}
void set_C_zero () {
  TIM1->CCR3 = 0;
}
/* 

 */
void set_A_plus (){
  HAL_GPIO_WritePin(in_A_GPIO_Port, in_A_Pin, forward_dir);
  TIM1->CCR1 = speed;
}
void set_B_plus (){
  HAL_GPIO_WritePin(in_B_GPIO_Port, in_B_Pin, forward_dir);
  TIM1->CCR2 = speed;
}
void set_C_plus (){
  HAL_GPIO_WritePin(in_C_GPIO_Port, in_C_Pin, forward_dir);
  TIM1->CCR3 = speed;
}

void set_A_minus (){
  HAL_GPIO_WritePin(in_A_GPIO_Port, in_A_Pin, forward_dir ^ 1);
  TIM1->CCR1 = speed;
}
void set_B_minus (){
  HAL_GPIO_WritePin(in_B_GPIO_Port, in_B_Pin, forward_dir ^ 1);
  TIM1->CCR2 = speed;
}
void set_C_minus (){
  HAL_GPIO_WritePin(in_C_GPIO_Port, in_C_Pin, forward_dir ^ 1);
  TIM1->CCR3 = speed;
}

// for situation when all halls 0
void step_0 (){

}
void step_1 (){
  set_C_zero(); set_A_plus(); set_B_minus(); 
}
void step_2 (){
  set_A_zero(); set_B_plus(); set_C_minus();
}
void step_3 (){
  set_B_zero(); set_A_plus(); set_C_minus();
}
void step_4 (){
  set_B_zero(); set_A_minus(); set_C_plus();
}
void step_5 (){
  set_A_zero(); set_B_minus(); set_C_plus();
}
void step_6 (){
  set_C_zero(); set_A_minus(); set_B_plus();
}
// for situation when all halls 1 
void step_7 (){

}

void soft_stop (){
  set_A_zero(); set_B_zero(); set_C_zero();
}
void hard_stop (){
  HAL_GPIO_WritePin(in_A_GPIO_Port, in_A_Pin, 0);
  TIM1->CCR1 = speed;
  HAL_GPIO_WritePin(in_B_GPIO_Port, in_B_Pin, 0);
  TIM1->CCR2 = speed;
  HAL_GPIO_WritePin(in_C_GPIO_Port, in_C_Pin, 0);
  TIM1->CCR3 = speed;
}

// Steps. Array of function pointers 
void (*BLDC_STEPS_FORWARD[8])(void) = {
  &soft_stop, &step_1, &step_2, &step_3, &step_4, &step_5, &step_6, &soft_stop
}; 


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == hall_A_Pin || GPIO_Pin == hall_B_Pin || GPIO_Pin == hall_C_Pin){
    hall_state = read_halls();
    switch (bldc_run){
      case BLDC_SOFT_STOP:
        soft_stop();
        break;
      case BLDC_HARD_STOP:
        hard_stop();
        break;
      case BLDC_RUN:
        (BLDC_STEPS_FORWARD[hall_state])();
        break;
    }
  } 
}


void BLDC_init(){
  max_speed = (TIM1->ARR) * 0.88;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void BLDC_set_run (BLDC_Run state){
  bldc_run = state;
}

void BLDC_soft_stop (){
  bldc_run = BLDC_SOFT_STOP;
  soft_stop();
}

void BLDC_hard_stop (){
  bldc_run = BLDC_HARD_STOP;
  hard_stop();
}

void BLDC_start (){
  bldc_run = BLDC_RUN;
}


// @param speed up for *TIM1 -> Counter Period* `TIM1->ARR`, but available only 88%
void BLDC_set_speed (uint16_t speed_){
  if (speed_ == 0) BLDC_soft_stop();
  if (speed_ > max_speed) speed_ = max_speed;
  BLDC_start();
  speed = speed_;
}

void BLDC_toggle_direction (){
  forward_dir ^= 1;
}
// @param direction: 1 - forward, 0 - backward
void BLDC_set_direction (uint8_t direction){
  forward_dir = direction;
}

void BLDC_forward (uint16_t speed){
  forward_dir = 1;
  BLDC_set_speed(speed);
}

void BLDC_backward (uint16_t speed){
  forward_dir = 0;
  BLDC_set_speed(speed);
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
  /* USER CODE BEGIN 2 */
  BLDC_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    BLDC_forward(5000);
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
