# USING
all code for motor in [./Core/Src/main.c]()

# CubeMX Config

## Sys
Debug = Serial Wire

## RCC
HSE = Crystal/Ceramic Resonator
(CLOCK CONFIG)->HCLK = 72

## NVIC
TIM2 global int = true

## GPIO
hall_{A,B,C} = (Input, No pull No push)
in_{A,B,C} = (Output, No pull No push, HIGH) TODO: check speed

## DMA
no

## TIM1
Clock Source = Internal
Channel{1,2,3} = PWM Generator CH{1,2,3}

Prescaler = 5
Counter Period = 999

## TIM2
Clock Source = Internal

Prescaler = 719
Counter Period = 9

NVIC->TIM2 global int = true 
