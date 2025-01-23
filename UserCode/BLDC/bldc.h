#pragma once

#include "stdint.h"
#include "main.h"

// ==================== CONFIG ====================
#define BLDC_TIM_PWM TIM1
#define BLDC_htim_PWM htim1

#define BLDC_TIM TIM2
#define BLDC_htim htim2
#define BLDC_COUNTER 100 // every BLDC_TIMES times speed++ in BLDC_tick  

#define BLDC_HALL_A_GPIO_Port hall_A_GPIO_Port
#define BLDC_HALL_B_GPIO_Port hall_B_GPIO_Port
#define BLDC_HALL_C_GPIO_Port hall_C_GPIO_Port

#define BLDC_HALL_A_Pin hall_A_Pin
#define BLDC_HALL_B_Pin hall_B_Pin
#define BLDC_HALL_C_Pin hall_C_Pin

#define BLDC_IN_A_GPIO_Port in_A_GPIO_Port
#define BLDC_IN_B_GPIO_Port in_B_GPIO_Port
#define BLDC_IN_C_GPIO_Port in_C_GPIO_Port

#define BLDC_IN_A_Pin in_A_Pin
#define BLDC_IN_B_Pin in_B_Pin
#define BLDC_IN_C_Pin in_C_Pin

// Must be != 
// If A = 0, B = 1, C = 2 so hall state = 0bCBA 
#define BLDC_HALL_A_pos 1
#define BLDC_HALL_B_pos 0
#define BLDC_HALL_C_pos 2

// ==================== TYPEDEF ====================

typedef enum {
    BLDC_SOFT_STOP = 0,
    BLDC_HARD_STOP = 1,
    BLDC_RUN = 2
} BLDC_Run;


// ==================== API ====================

// initialize pwm channels on timer
void BLDC_init();
// set state. May be used for stop on next hall step   
void BLDC_set_run (BLDC_Run state);
// soft stop - smooth  
void BLDC_soft_stop ();
// hard stop - force of brake depend on current speed 
void BLDC_hard_stop ();
// set run state
void BLDC_start ();

// MAIN. run every iteration of infty loop
void BLDC_tick();

// @param speed up for *TIM1 -> Counter Period* `TIM1->ARR`, but available only 88%
void BLDC_set_speed (uint16_t speed_);
// toggle direction
void BLDC_toggle_direction ();
// @param direction: 1 - forward, 0 - backward
void BLDC_set_direction (uint8_t direction);
// run forward with speed (limited 88%)
void BLDC_forward (uint16_t speed);
// run backward with speed (limited 88%) 
void BLDC_backward (uint16_t speed);

long map(long x, long in_min, long in_max, long out_min, long out_max);

