#include "bldc.h"

#include "tim.h"



uint8_t hall_state;
uint8_t forward_dir = 1;
uint16_t speed = 0, max_speed = 8800;
BLDC_Run bldc_run = BLDC_SOFT_STOP;


uint8_t read_halls(){
  return (HAL_GPIO_ReadPin(BLDC_HALL_C_GPIO_Port, BLDC_HALL_C_Pin) << BLDC_HALL_C_pos)
      |  (HAL_GPIO_ReadPin(BLDC_HALL_B_GPIO_Port, BLDC_HALL_B_Pin) << BLDC_HALL_B_pos)
      |  (HAL_GPIO_ReadPin(BLDC_HALL_A_GPIO_Port, BLDC_HALL_A_Pin) << BLDC_HALL_A_pos);
}


void set_A_zero () {
  BLDC_TIM->CCR1 = 0;
}
void set_B_zero () {
  BLDC_TIM->CCR2 = 0;
}
void set_C_zero () {
  BLDC_TIM->CCR3 = 0;
}


void set_A_plus (){
  HAL_GPIO_WritePin(BLDC_IN_A_GPIO_Port, BLDC_IN_A_Pin, forward_dir);
  BLDC_TIM->CCR1 = speed;
}
void set_B_plus (){
  HAL_GPIO_WritePin(BLDC_IN_B_GPIO_Port, BLDC_IN_B_Pin, forward_dir);
  BLDC_TIM->CCR2 = speed;
}
void set_C_plus (){
  HAL_GPIO_WritePin(BLDC_IN_C_GPIO_Port, BLDC_IN_C_Pin, forward_dir);
  BLDC_TIM->CCR3 = speed;
}


void set_A_minus (){
  HAL_GPIO_WritePin(BLDC_IN_A_GPIO_Port, BLDC_IN_A_Pin, forward_dir ^ 1);
  BLDC_TIM->CCR1 = speed;
}
void set_B_minus (){
  HAL_GPIO_WritePin(BLDC_IN_B_GPIO_Port, BLDC_IN_B_Pin, forward_dir ^ 1);
  BLDC_TIM->CCR2 = speed;
}
void set_C_minus (){
  HAL_GPIO_WritePin(BLDC_IN_C_GPIO_Port, BLDC_IN_C_Pin, forward_dir ^ 1);
  BLDC_TIM->CCR3 = speed;
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
  HAL_GPIO_WritePin(BLDC_IN_A_GPIO_Port, BLDC_IN_A_Pin, 0);
  BLDC_TIM->CCR1 = speed;
  HAL_GPIO_WritePin(BLDC_IN_B_GPIO_Port, BLDC_IN_B_Pin, 0);
  BLDC_TIM->CCR2 = speed;
  HAL_GPIO_WritePin(BLDC_IN_C_GPIO_Port, BLDC_IN_C_Pin, 0);
  BLDC_TIM->CCR3 = speed;
}

// Steps. Array of function pointers 
void (*BLDC_STEPS_FORWARD[8])(void) = {
  &soft_stop, &step_1, &step_2, &step_3, &step_4, &step_5, &step_6, &soft_stop
}; 


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == BLDC_HALL_A_Pin || GPIO_Pin == BLDC_HALL_B_Pin || GPIO_Pin == BLDC_HALL_C_Pin){
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
  max_speed = (BLDC_TIM->ARR) * 0.88;
  HAL_TIM_PWM_Start(&BLDC_htim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&BLDC_htim, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&BLDC_htim, TIM_CHANNEL_3);
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