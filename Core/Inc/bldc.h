#pragma once

#include "stdint.h"
#include "main.h"

// ==================== CONFIG ====================
#define BLDC_TIM TIM2
#define BLDC_htim htim2

#define BLDC_HALL_A_GPIO_PORT hall_A_GPIO_Port
#define BLDC_HALL_B_GPIO_PORT hall_B_GPIO_Port
#define BLDC_HALL_C_GPIO_PORT hall_C_GPIO_Port

#define BLDC_HALL_A_Pin hall_A_Pin
#define BLDC_HALL_B_Pin hall_B_Pin
#define BLDC_HALL_C_Pin hall_C_Pin

#define BLDC_IN_A_GPIO_PORT in_A_GPIO_Port
#define BLDC_IN_B_GPIO_PORT in_B_GPIO_Port
#define BLDC_IN_C_GPIO_PORT in_C_GPIO_Port

#define BLDC_IN_A_Pin in_A_Pin
#define BLDC_IN_B_Pin in_B_Pin
#define BLDC_IN_C_Pin in_C_Pin


typedef enum {
    BLDC_SOFT_STOP = 0,
    BLDC_HARD_STOP = 1,
    BLDC_RUN = 2
} BLDC_Run;


uint8_t hall_state;
uint8_t forward_dir = 1;
uint16_t speed = 0, max_speed = 8800;
BLDC_Run bldc_run = BLDC_SOFT_STOP;

// Must be != 
// If A = 0, B = 1, C = 2 so hall state = 0bCBA 
uint8_t hall_A_pos = 1, hall_B_pos = 0, hall_C_pos = 2;



void BLDC_init();

void BLDC_set_run (BLDC_Run state);

void BLDC_soft_stop ();

void BLDC_hard_stop ();

void BLDC_start ();


// @param speed up for *TIM1 -> Counter Period* `TIM1->ARR`, but available only 88%
void BLDC_set_speed (uint16_t speed_);

void BLDC_toggle_direction ();
// @param direction: 1 - forward, 0 - backward
void BLDC_set_direction (uint8_t direction);

void BLDC_forward (uint16_t speed);

void BLDC_backward (uint16_t speed);
