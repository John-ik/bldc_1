#include "bldc.h"

#include "tim.h"



uint8_t hall_state;
uint8_t forward_dir = 1;
uint16_t speed = 0, target_speed = 0, max_speed = 8800;
BLDC_Run bldc_run = BLDC_SOFT_STOP;


uint8_t mask_halls_pos[2][3] = {
  {BLDC_HALL_A_pos, BLDC_HALL_B_pos, BLDC_HALL_C_pos},
  {BLDC_HALL_A_pos, BLDC_HALL_C_pos, BLDC_HALL_B_pos}
};
uint8_t triplet;
hall_pos_t hall_pos;


hall_pos_t* BLDC_get_ptr_pos (){
  return &hall_pos;
}

// set next position combination 
// combination is looped
void BLDC_next_hall_pos_combination (){
  if (++triplet == 2) triplet = 0; // loop 0-1 value
  // loop increment 
    if (++mask_halls_pos[triplet][0] == 3) mask_halls_pos[triplet][0] = 0; // loop 0-2 value
    if (++mask_halls_pos[triplet][1] == 3) mask_halls_pos[triplet][1] = 0; // loop 0-2 value
    if (++mask_halls_pos[triplet][2] == 3) mask_halls_pos[triplet][2] = 0; // loop 0-2 value

  // set current combination
  hall_pos.a = mask_halls_pos[triplet][0];
  hall_pos.b = mask_halls_pos[triplet][1];
  hall_pos.c = mask_halls_pos[triplet][2];
}


uint8_t read_halls(){
  return (HAL_GPIO_ReadPin(BLDC_HALL_C_GPIO_Port, BLDC_HALL_C_Pin) << hall_pos.c)
      |  (HAL_GPIO_ReadPin(BLDC_HALL_B_GPIO_Port, BLDC_HALL_B_Pin) << hall_pos.b)
      |  (HAL_GPIO_ReadPin(BLDC_HALL_A_GPIO_Port, BLDC_HALL_A_Pin) << hall_pos.a);
}

// uint8_t read_halls_opt (){
//   return BLDC_HALL_A_GPIO_Port
// }


void set_A_zero () {
  BLDC_TIM_PWM->CCR1 = 0;
}
void set_B_zero () {
  BLDC_TIM_PWM->CCR2 = 0;
}
void set_C_zero () {
  BLDC_TIM_PWM->CCR3 = 0;
}


void set_A_plus (){
  HAL_GPIO_WritePin(BLDC_IN_A_GPIO_Port, BLDC_IN_A_Pin, forward_dir);
  BLDC_TIM_PWM->CCR1 = speed;
}
void set_B_plus (){
  HAL_GPIO_WritePin(BLDC_IN_B_GPIO_Port, BLDC_IN_B_Pin, forward_dir);
  BLDC_TIM_PWM->CCR2 = speed;
}
void set_C_plus (){
  HAL_GPIO_WritePin(BLDC_IN_C_GPIO_Port, BLDC_IN_C_Pin, forward_dir);
  BLDC_TIM_PWM->CCR3 = speed;
}


void set_A_minus (){
  HAL_GPIO_WritePin(BLDC_IN_A_GPIO_Port, BLDC_IN_A_Pin, forward_dir ^ 1);
  BLDC_TIM_PWM->CCR1 = BLDC_TIM_PWM->ARR;
}
void set_B_minus (){
  HAL_GPIO_WritePin(BLDC_IN_B_GPIO_Port, BLDC_IN_B_Pin, forward_dir ^ 1);
  BLDC_TIM_PWM->CCR2 = BLDC_TIM_PWM->ARR;
}
void set_C_minus (){
  HAL_GPIO_WritePin(BLDC_IN_C_GPIO_Port, BLDC_IN_C_Pin, forward_dir ^ 1);
  BLDC_TIM_PWM->CCR3 = BLDC_TIM_PWM->ARR;
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
  BLDC_TIM_PWM->CCR1 = speed;
  HAL_GPIO_WritePin(BLDC_IN_B_GPIO_Port, BLDC_IN_B_Pin, 0);
  BLDC_TIM_PWM->CCR2 = speed;
  HAL_GPIO_WritePin(BLDC_IN_C_GPIO_Port, BLDC_IN_C_Pin, 0);
  BLDC_TIM_PWM->CCR3 = speed;
}

// Steps. Array of function pointers 
void (*BLDC_STEPS_FORWARD[8])(void) = {
  &soft_stop, &step_1, &step_2, &step_3, &step_4, &step_5, &step_6, &soft_stop
}; 


void BLDC_tick (){
  static uint32_t counter = 0;
  switch (bldc_run){
    case BLDC_SOFT_STOP:
      soft_stop();
      break;
    case BLDC_HARD_STOP:
      hard_stop();
      break;
    case BLDC_RUN:
      if (speed < target_speed && counter++ > BLDC_COUNTER) {
        speed++;
        counter = 0;
      }
      hall_state = read_halls();
      (BLDC_STEPS_FORWARD[hall_state])();
      break;
  }
}

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//   if(GPIO_Pin == BLDC_HALL_A_Pin || GPIO_Pin == BLDC_HALL_B_Pin || GPIO_Pin == BLDC_HALL_C_Pin){
//     BLDC_tick();
//   } 
// }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM2)
  {
    BLDC_tick();
  }
}


void BLDC_init(){
  max_speed = (BLDC_TIM_PWM->ARR) * 0.88;
  
  // start pwm 
  HAL_TIM_PWM_Start(&BLDC_htim_PWM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&BLDC_htim_PWM, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&BLDC_htim_PWM, TIM_CHANNEL_3);

  // start timer
  HAL_TIM_Base_Start_IT(&BLDC_htim);

}

// use after `BLDC_init`
uint16_t BLDC_get_max_speed(){
  return max_speed;
}

void BLDC_set_run (BLDC_Run state){
  if (bldc_run == state) return;
  bldc_run = state;
  BLDC_tick();
}

void BLDC_soft_stop (){
  if (bldc_run == BLDC_SOFT_STOP) return;
  bldc_run = BLDC_SOFT_STOP;
  soft_stop();
}

void BLDC_hard_stop (){
  if (bldc_run == BLDC_HARD_STOP) return;
  bldc_run = BLDC_HARD_STOP;
  hard_stop();
}

void BLDC_start (){
  bldc_run = BLDC_RUN;
  BLDC_tick();
}


// @param speed up for *TIM1 -> Counter Period* `TIM1->ARR`, but available only 88%
void BLDC_set_speed (uint16_t speed_){
  if (speed_ == target_speed) return;
  if (speed_ > max_speed) speed_ = max_speed;
  
  // smooth switch speed
  target_speed = speed_;
  if (speed_ > speed  &&  speed_ - speed > max_speed * 0.15){
  } else {
    speed = speed_;
  }

  if (speed_ == 0) {
    BLDC_soft_stop();
    return;
  }
  BLDC_start();
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


long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}