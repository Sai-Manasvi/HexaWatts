#include "main.h"
#include <math.h>

#define ADC_MAX     4095   //steering wheel if from 0-4095 (0-3.3V)
#define PWM_MAX     1000   //motor speed 0 to 1000, Adjust based on our timer's ARR
#define B           0.5f   // Track width (meters)
#define L           1.2f   // Wheelbase (meters)
#define STEER_MAX_ANGLE 30.0f  // Max steering angle in degrees

float V_vehicle = 2.0f; // Placeholder vehicle speed
uint32_t adcValue = 0;

// Placeholder PWM values
uint16_t pwmLeft = 0;
uint16_t pwmRight = 0;

float degToRad(float deg) {
    return deg * (3.14159f / 180.0f);
}

float getSteeringAngleFromADC(uint32_t adc) {
    return ((float)adc / ADC_MAX) * (2 * STEER_MAX_ANGLE) - STEER_MAX_ANGLE;
}

uint16_t velocityToPWM(float velocity, float Vmax) {
    if (velocity < 0) velocity = 0;
    if (velocity > Vmax) velocity = Vmax;
    return (uint16_t)((velocity / Vmax) * PWM_MAX);
}

void updateMotorPWMs(float theta_deg) {
    float theta_rad = degToRad(theta_deg);

    float V_inner = V_vehicle * (1.0f - (B * tanf(theta_rad)) / (2.0f * L));
    float V_outer = V_vehicle * (1.0f + (B * tanf(theta_rad)) / (2.0f * L));

    float VL, VR;

    if (theta_deg > 0) {
        // Right turn
        VR = V_inner;
        VL = V_outer;
    } else {
        // Left turn or straight
        VL = V_inner;
        VR = V_outer;
    }

    pwmLeft = velocityToPWM(VL, V_vehicle);
    pwmRight = velocityToPWM(VR, V_vehicle);

    // Send to PWM hardware (TIM2 used here as example)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmLeft);  // Left motor
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwmRight); // Right motor
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    adcValue = HAL_ADC_GetValue(hadc);
    float steeringAngle = getSteeringAngleFromADC(adcValue);
    updateMotorPWMs(steeringAngle);
}

int main(void) {

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Left motor
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Right motor
    HAL_ADC_Start_IT(&hadc1);  // ADC interrupt mode

    while (1) {
        // All work done in interrupt
    }
}

/*
 * Title       : Motors Speed Control Based on Steering Angle
 * File        : steer_motor.c
 * Author      : Gaddam Sai Manasvi
 * Team        : HexaWatts - Formula Bharat 2026
 *
 * Description :
 *    This program controls the speed of the left and right motors in an EV
 *    based on the steering angle of the vehicle. The purpose is to enable
 *    smooth cornering by adjusting motor speeds using Ackermann-like geometry.
 *
 *    - Steering angle is read via ADC (0–4095), mapped to -30° to +30°
 *    - Vehicle speed is assumed constant for now (2.0 m/s placeholder)
 *    - Calculates inner and outer wheel speeds using:
 *        V_inner = V * [1 - (B * tan(θ)) / (2L)]
 *        V_outer = V * [1 + (B * tan(θ)) / (2L)]
 *    - Converts each velocity to PWM and sends it to the motors
 *
 * Variables :
 *    - ADC_MAX 	= Steering Wheel *if* from 0-4095 (0-3.3V)
 *    - PWM_MAX 	= Motor Speed 0 to 1000, Adjust based on our timer's ARR
 *    - B 		= Track Width (meters)
 *    - L		= Wheelbase (meters)
 *    - STEER_MAX_ANGLE = Max steering angle in degrees
 *    - PWM (Pulse Width Modulation) output (to control motor speed)
 *    - ADC input (to read steering wheel)
 *    - V_vehicle = Vehicle Speed
 *
 * Functions:
 *	- float degToRad(float deg)
 *	// Converts degrees to radians
 *
 *	- float getSteeringAngleFromADC(uint32_t adc)
 *	// Map ADC value (0–4095) to angle (-30° to +30°)
 *
 *	- uint16_t velocityToPWM(float velocity, float Vmax)
 *	// Convert velocity (0–max) to PWM (0–PWM_MAX), Helper to map ADC value to PWM range
 *
 *	- void updateMotorPWMs(float theta_deg)	
 *	//It takes the steering angle (from 0 to 4095) and it adjusts the left and right motor speeds
 *	//If steering is centered (2048), both motors get speed 500
 *
 *	- void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
 *	//Whenever the steering wheel angle is updated, this will recalculate the motor speeds
 *	
 *	- int main(void)
 *	//This is the main function that runs when the STM32 powers up
 *
 * Hardware *Assumptions*:
 *    - STM32 microcontroller with Timer2 used for PWM
 *    - Left and right motors connected to TIM2_CH1 and TIM2_CH2
 *    - Steering angle from a potentiometer connected to ADC1
 *
 * Future Improvements:
 *    - Replace constant V with sensor-based speed input
 *    - Include real B (track width) and L (wheelbase) values
 *    - Add safety limits and fault detection
 */
