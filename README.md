# HexaWatts

## 1) Steer_Motor
### Description :
This program controls the speed of the left and right motors in an EV based on the steering angle of the vehicle.                                      
The purpose is to enable smooth cornering by adjusting motor speeds using Ackermann-like geometry.
* Steering angle is read via ADC (0–4095), mapped to -30° to +30°
* Vehicle speed is assumed constant for now (2.0 m/s placeholder)
* Calculates inner and outer wheel speeds using:
* V_inner = V * [1 - (B * tan(θ)) / (2L)]
* V_outer = V * [1 + (B * tan(θ)) / (2L)]
* Converts each velocity to PWM and sends it to the motors

### Variables :
 * ADC_MAX 	= Steering Wheel *if* from 0-4095 (0-3.3V)
 * PWM_MAX 	= Motor Speed 0 to 1000, Adjust based on our timer's ARR
 * B 		= Track Width (meters)
 * L		= Wheelbase (meters)
 * STEER_MAX_ANGLE = Max steering angle in degrees
 * PWM (Pulse Width Modulation) output (to control motor speed)
 * ADC input (to read steering wheel)
 * V_vehicle = Vehicle Speed
 
 ### Functions:
 *	float degToRad (float deg)      
 Converts degrees to radians
 
 *	float getSteeringAngleFromADC (uint32_t adc)          
 Map ADC value (0–4095) to angle (-30° to +30°)
 
 *	uint16_t velocityToPWM (float velocity, float Vmax)                     
 Convert velocity (0–max) to PWM (0–PWM_MAX), Helper to map ADC value to PWM range
 
 *	void updateMotorPWMs (float theta_deg)                        	                                                                        
 It takes the steering angle (from 0 to 4095) and it adjusts the left and right motor speeds.                            
 If steering is centered (2048), both motors get speed 500
 
 *	void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc)                       
 Whenever the steering wheel angle is updated, this will recalculate the motor speeds

 * int main(void)                       
 This is the main function that runs when the STM32 powers up

### Hardware *Assumptions*:
- STM32 microcontroller with Timer2 used for PWM
- Left and right motors connected to TIM2_CH1 and TIM2_CH2
- Steering angle from a potentiometer connected to ADC1
 
### Future Improvements:
- Replace constant V with sensor-based speed input
- Include real B (track width) and L (wheelbase) values
- Add safety limits and fault detection
