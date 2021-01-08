/*
 * control_algo.c
 *
 *  Created on: 6 nov. 2020
 *      Author: Patrick
 */

#include "control_algo.h"
#include "control_table.h"
#include "math_tool.h"
#include "binary_tool.h"
#include "pid.h"

#include <stdbool.h>
#include <math.h>

// Public variables
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern ADC_HandleTypeDef hadc1;
extern OPAMP_HandleTypeDef hopamp1;

// CONTROL LOOP
#define LOOP_FREQUENCY_HZ 1000.0f //Hz
// MOTOR
#define MOTOR_PWM_BRAKE 99
#define MOTOR_PWM_COAST 0
// POSITION FILTER
#define ALPHA_POSITION			0.05f // (default:0.05) F = 20kHz ==> Fc (-3dB) = 170.0Hz
#define ALPHA_VOLTAGE			0.05f // (default:0.05) F = 20kHz ==> Fc (-3dB) = 170.0Hz
#define ALPHA_CURRENT_SENSE		0.05f // (default:0.05) F = 20kHz ==> Fc (-3dB) = 170.0Hz
#define ALPHA_VELOCITY			0.12f // (default:0.12) F = 1000Hz ==> Fc (-3dB) = 20Hz
#define ALPHA_CURRENT_SETPOINT 	0.96f // (default:0.12) F = 1000Hz ==> Fc (-3dB) = 20Hz
#define ALPHA_PWM_SETPOINT		0.12f // (default:0.12) F = 1000Hz ==> Fc (-3dB) = 20.0Hz
#define ALPHA_B					0.001f
#define VOLTAGE_CALIBRATION 	1.08f
// PID WINDUP = 0..99
#define LIMIT_PID_POSITION_WINDUP 99
#define LIMIT_PID_CURRENT_WINDUP 33

// Private Variables
volatile uint16_t ADC_DMA[3] = { 0,0,0 };
static uint16_t motor_current_input_adc = 0;
static float position_input_adc = 0.0f;
static float voltage_input_adc = 0.0f;
static uint32_t current_control_mode = REG_CONTROL_MODE_POSITION;
static uint16_t const period_us = (uint16_t)(1000000.0f/LOOP_FREQUENCY_HZ);
static uint32_t counter = 0;
static pid_context_t pid_position;
static pid_context_t pid_current;
static float present_position_deg = 0.0f;
static float present_velocity_dps = 0.0f;
static float present_current_ma_on = 0.0f;
static float present_current_ma_off = 0.0f;
static float setpoint_pwm = 0.0f;
static float setpoint_current_ma = 0.0f;
static float setpoint_acceleration_dpss = 0.0f;
static float setpoint_velocity_dps = 0.0f;
static float setpoint_position_deg = 0.0f;
static float b = 0;
static float last_present_position_deg = 0.0f;
static float last_setpoint_velocity_dps = 0.0f;
static bool entering_state = true;


float query_position_sensor()
{
	// read input position sensor
	//position_input_adc = read_adc(&hadc2,ADC_CHANNEL_10,ADC_SAMPLETIME_47CYCLES_5);
	uint16_t const min_position_adc = MAKE_SHORT(regs[REG_MIN_POSITION_ADC_L],regs[REG_MIN_POSITION_ADC_H]);
	uint16_t const max_position_adc = MAKE_SHORT(regs[REG_MAX_POSITION_ADC_L],regs[REG_MAX_POSITION_ADC_H]);
	//bool const position_input_adc_valid = (position_input_adc>=min_position_adc) && (position_input_adc<=max_position_adc);

	// compute present position in deg and apply scale and inversion
	float const max_rotation_deg = (float)(regs[REG_MAX_ROTATION_DEG]);
	float present_position_deg_unfiltered = fmap((float)position_input_adc,(float)min_position_adc,(float)max_position_adc,0.0f,max_rotation_deg);
	float const inv_rotation_sensor = regs[REG_INV_ROTATION_SENSOR] > 0 ? -1.0f : 0.0f;
	if(inv_rotation_sensor<0)
	{
		present_position_deg_unfiltered =  max_rotation_deg-present_position_deg_unfiltered;
	}
	return present_position_deg_unfiltered;
}


void APP_Control_Reset()
{
	// reset
	entering_state = true;
	counter = 0;
	pid_reset(&pid_position);
	pid_reset(&pid_current);
	setpoint_pwm = 0.0f;
	setpoint_current_ma = 0.0f;
	setpoint_acceleration_dpss = 0.0f;
	setpoint_velocity_dps = 0.0f;
	last_setpoint_velocity_dps = 0.0f;
	setpoint_position_deg = present_position_deg;
	// when re-entering in the control mode 'position', avoid glitch from past goal position
	regs[REG_GOAL_POSITION_DEG_L] = LOW_BYTE((int16_t)(present_position_deg*10.0f));
	regs[REG_GOAL_POSITION_DEG_H] = HIGH_BYTE((int16_t)(present_position_deg*10.0f));

}

void APP_Control_Init()
{
	// start aMPOP
	HAL_OPAMP_Start(&hopamp1);
	// start adc conv trigger
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_DMA,3);

	// motor Hi-Z
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MOTOR_PWM_COAST);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MOTOR_PWM_COAST);
	// motor init
	HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim15);

	// first position update, force goal at present position to avoid mechanical glicth at startup
	present_position_deg = query_position_sensor();
	last_present_position_deg = present_position_deg;

	// defaut control mode and reset
	current_control_mode = REG_CONTROL_MODE_POSITION;
	APP_Control_Reset();
}


void APP_Control_Process()
{
	uint16_t current_time = __HAL_TIM_GET_COUNTER(&htim15);
	// wait for period
	if(current_time<period_us)
		return;
	__HAL_TIM_SET_COUNTER(&htim15,(current_time-period_us));

	// read position and filter
	float const present_position_deg = query_position_sensor();

	// compute present speed in dps, present position derivative
	float present_speed_dps_unfiltered = (present_position_deg - last_present_position_deg)*LOOP_FREQUENCY_HZ;
	last_present_position_deg =  present_position_deg;
	present_velocity_dps = ALPHA_VELOCITY * present_speed_dps_unfiltered + (1.0f-ALPHA_VELOCITY)*present_velocity_dps;

	// torque enable logic
	bool torque_enable = (regs[REG_TORQUE_ENABLE]!=0) && (regs[REG_HARDWARE_ERROR_STATUS]==0);
	if(torque_enable)
	{
		// modes & transitions
		// compute setpoint_pwm
		switch(current_control_mode)
		{
		// Simple PID from position to pwm
		// Mg92 Kp = 400 Ki = 20 Kd = 400
		case REG_CONTROL_MODE_POSITION:
			if(entering_state)
			{
				entering_state = false;
				// init goal RAM registers according this control mode
				regs[REG_GOAL_POSITION_DEG_L] = LOW_BYTE((int16_t)(present_position_deg*10.0f));
				regs[REG_GOAL_POSITION_DEG_H] = HIGH_BYTE((int16_t)(present_position_deg*10.0f));
				// init limit RAM registers according this control mode
				regs[REG_GOAL_PWM_100_L] = regs[REG_MAX_PWM_100_L];
				regs[REG_GOAL_PWM_100_H] = regs[REG_MAX_PWM_100_H];
				// reset others
				regs[REG_GOAL_VELOCITY_DPS_L] = 0;
				regs[REG_GOAL_VELOCITY_DPS_H] = 0;
				regs[REG_GOAL_CURRENT_MA_L] = 0;
				regs[REG_GOAL_CURRENT_MA_H] = 0;
				// set setpoint position at current position
				setpoint_position_deg = present_position_deg;
			}
			{
				// limit goal position
				float const min_goal_position_deg = (float)(MAKE_SHORT(regs[REG_MIN_POSITION_DEG_L],regs[REG_MIN_POSITION_DEG_H]));
				float const max_goal_position_deg = (float)(MAKE_SHORT(regs[REG_MAX_POSITION_DEG_L],regs[REG_MAX_POSITION_DEG_H]));
				float goal_position_deg = 0.1f * (float)(MAKE_SHORT(regs[REG_GOAL_POSITION_DEG_L],regs[REG_GOAL_POSITION_DEG_H]));
				goal_position_deg = fconstrain(goal_position_deg,min_goal_position_deg,max_goal_position_deg);
				// compute position setpoint from goal position
				// there is no profil, so position setpoint is goal position
				setpoint_position_deg = goal_position_deg;
				// compute position error
				float const error_position = setpoint_position_deg - present_position_deg;
				// compute pwm setpoint from position error using a PID position
				float const pid_pos_kp = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KP_L],regs[REG_PID_POSITION_KP_H]))/100.0f;
				float const pid_pos_ki = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KI_L],regs[REG_PID_POSITION_KI_H]))/1000.0f;
				float const pid_pos_kd = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KD_L],regs[REG_PID_POSITION_KD_H]))/10.0f;
				float const pwm_limit = (float)(MAKE_SHORT(regs[REG_GOAL_PWM_100_L],regs[REG_GOAL_PWM_100_H]));
				setpoint_pwm =
						ALPHA_PWM_SETPOINT * (
								pid_process_antiwindup_clamp(
										&pid_position,
										error_position,
										pid_pos_kp,
										pid_pos_ki,
										pid_pos_kd,
										pwm_limit,
										ALPHA_VELOCITY
								)
						) +
						(1.0f-ALPHA_PWM_SETPOINT) * setpoint_pwm;
			}
			// unused setpoints zero
			setpoint_velocity_dps = 0.0f;
			setpoint_current_ma = 0.0f;
			// mode change
			if(regs[REG_CONTROL_MODE]!=REG_CONTROL_MODE_POSITION)
			{
				APP_Control_Reset();
				current_control_mode = regs[REG_CONTROL_MODE];
			}
			break;


		case REG_CONTROL_MODE_POSITION_TORQUE:
			if(entering_state)
			{
				entering_state = false;
				// init goal RAM registers according this control mode
				regs[REG_GOAL_POSITION_DEG_L] = LOW_BYTE((int16_t)(present_position_deg*10.0f));
				regs[REG_GOAL_POSITION_DEG_H] = HIGH_BYTE((int16_t)(present_position_deg*10.0f));
				// init limit RAM registers according this control mode
				regs[REG_GOAL_CURRENT_MA_L] = regs[REG_MAX_CURRENT_MA_L];
				regs[REG_GOAL_CURRENT_MA_H] = regs[REG_MAX_CURRENT_MA_H];
				regs[REG_GOAL_PWM_100_L] = regs[REG_MAX_PWM_100_L];
				regs[REG_GOAL_PWM_100_H] = regs[REG_MAX_PWM_100_H];
				// reset others
				regs[REG_GOAL_VELOCITY_DPS_L] = 0.0f;
				regs[REG_GOAL_VELOCITY_DPS_H] = 0.0f;
				// set setpoint position at current position
				setpoint_position_deg = present_position_deg;
			}
			{
				// limit goal position
				float const min_goal_position_deg = (float)(MAKE_SHORT(regs[REG_MIN_POSITION_DEG_L],regs[REG_MIN_POSITION_DEG_H]));
				float const max_goal_position_deg = (float)(MAKE_SHORT(regs[REG_MAX_POSITION_DEG_L],regs[REG_MAX_POSITION_DEG_H]));
				float goal_position_deg = 0.1f * (float)(MAKE_SHORT(regs[REG_GOAL_POSITION_DEG_L],regs[REG_GOAL_POSITION_DEG_H]));
				goal_position_deg = fconstrain(goal_position_deg,min_goal_position_deg,max_goal_position_deg);
				// compute position setpoint from goal position
				// there is no profil, so position setpoint is goal position
				setpoint_position_deg = goal_position_deg;
				// compute position error
				float const error_position = setpoint_position_deg - present_position_deg;
				// compute current setpoint from position error using a PID position
				float const pid_pos_kp = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KP_L],regs[REG_PID_POSITION_KP_H]))/100.0f;
				float const pid_pos_ki = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KI_L],regs[REG_PID_POSITION_KI_H]))/1000.0f;
				float const pid_pos_kd = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KD_L],regs[REG_PID_POSITION_KD_H]))/10.0f;
				float const current_limit = (float)(MAKE_SHORT(regs[REG_GOAL_CURRENT_MA_L],regs[REG_GOAL_CURRENT_MA_H]));
				setpoint_current_ma =
						ALPHA_CURRENT_SETPOINT * (
								pid_process_antiwindup_clamp(
										&pid_position,
										error_position,
										pid_pos_kp,
										pid_pos_ki,
										pid_pos_kd,
										current_limit,
										ALPHA_VELOCITY
								)
						) +
						(1.0f-ALPHA_CURRENT_SETPOINT) * setpoint_current_ma;
			}
			{
				// compute current error
				float const error_current = setpoint_current_ma - present_current_ma_on;
				// compute pwm setpoint from current error using a PI
				float const pid_current_kp = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KP_L],regs[REG_PID_CURRENT_KP_H]))/1000.0f;
				float const pid_current_ki = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KI_L],regs[REG_PID_CURRENT_KI_H]))/100.0f;
				float const pid_current_kff = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KFF_L],regs[REG_PID_CURRENT_KFF_H]))/100.0f;
				float const pwm_limit = (float)(MAKE_SHORT(regs[REG_GOAL_PWM_100_L],regs[REG_GOAL_PWM_100_H]));
				setpoint_pwm =
						ALPHA_PWM_SETPOINT * (
								pid_current_kff * setpoint_current_ma +
								pid_process_antiwindup_clamp(
										&pid_current,
										error_current,
										pid_current_kp,
										pid_current_ki,
										0.0f,
										pwm_limit,
										0.0f
								)
							) +
							(1.0f-ALPHA_PWM_SETPOINT) * setpoint_pwm ;
			}
			// mode change
			if(regs[REG_CONTROL_MODE]!=REG_CONTROL_MODE_POSITION_TORQUE)
			{
				APP_Control_Reset();
				current_control_mode = regs[REG_CONTROL_MODE];
			}
			break;


		case REG_CONTROL_MODE_POSITION_PROFIL:
			if(entering_state)
			{
				entering_state = false;
				// init goal RAM registers according this control mode
				regs[REG_GOAL_POSITION_DEG_L] = LOW_BYTE((int16_t)(present_position_deg*10.0f));
				regs[REG_GOAL_POSITION_DEG_H] = HIGH_BYTE((int16_t)(present_position_deg*10.0f));
				// init limit RAM registers according this control mode
				regs[REG_GOAL_VELOCITY_DPS_L] = regs[REG_MAX_VELOCITY_DPS_L];
				regs[REG_GOAL_VELOCITY_DPS_H] = regs[REG_MAX_VELOCITY_DPS_H];
				regs[REG_GOAL_CURRENT_MA_L] = regs[REG_MAX_CURRENT_MA_L];
				regs[REG_GOAL_CURRENT_MA_H] = regs[REG_MAX_CURRENT_MA_H];
				regs[REG_GOAL_PWM_100_L] = regs[REG_MAX_PWM_100_L];
				regs[REG_GOAL_PWM_100_H] = regs[REG_MAX_PWM_100_H];
				// set setpoint position at current position
				setpoint_position_deg = present_position_deg;
			}
//			{
//				// compute position setpoint from goal position and velocity limit using a PID position
//				float const min_goal_position_deg = (float)(MAKE_SHORT(regs[REG_MIN_POSITION_DEG_L],regs[REG_MIN_POSITION_DEG_H]));
//				float const max_goal_position_deg = (float)(MAKE_SHORT(regs[REG_MAX_POSITION_DEG_L],regs[REG_MAX_POSITION_DEG_H]));
//				float goal_position_deg = 0.1f * (float)(MAKE_SHORT(regs[REG_GOAL_POSITION_DEG_L],regs[REG_GOAL_POSITION_DEG_H]));
//				goal_position_deg = fconstrain(goal_position_deg,min_goal_position_deg,max_goal_position_deg);
//				float const max_velocity_dps = (float)(MAKE_SHORT(regs[REG_GOAL_VELOCITY_DPS_L],regs[REG_GOAL_VELOCITY_DPS_H]));
//				float const max_acceleration_dpss = (float)(MAKE_SHORT(regs[REG_MAX_ACCELERATION_DPSS_L],regs[REG_MAX_ACCELERATION_DPSS_H]));
//
//				// trapezoidal profile for setpoint position
//
//				// compute remaining distance between setpoint position to goal position
//				float const remaining_distance_deg = goal_position_deg - setpoint_position_deg;
//
//				// compute maximun velocity to be able to stop at goal position
//				float vmax = sqrtf( 2.0f * max_acceleration_dpss * fabsf(remaining_distance_deg) );
//				// restore sign
//				vmax = ( remaining_distance_deg>0.0f) ? vmax : -vmax;
//				// limit maximum velocity, when far from stop
//				vmax = fconstrain(vmax,-max_velocity_dps,max_velocity_dps);
//				// compute distance between maximun velocity and current velocity
//				float delta_v = vmax - setpoint_velocity_dps;
//				// now compute new velocity according acceleration
//				setpoint_velocity_dps += fconstrain(delta_v, (-max_acceleration_dpss/LOOP_FREQUENCY_HZ), (max_acceleration_dpss/LOOP_FREQUENCY_HZ));
//				// now compute new position
//				setpoint_position_deg += (setpoint_velocity_dps/LOOP_FREQUENCY_HZ);
//				// now compute acceleration
//				setpoint_acceleration_dpss = (setpoint_velocity_dps - last_setpoint_velocity_dps)*LOOP_FREQUENCY_HZ;
//				last_setpoint_velocity_dps =  setpoint_velocity_dps;
//				// compute errors
//				float const error_position = setpoint_position_deg - present_position_deg;
//				float const error_velocity = setpoint_velocity_dps - present_velocity_dps;
//				// compute current setpoint from position setpoint using a PID position and velocity/acceleration feed forwards
//				float const pid_pos_kp = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KP_L],regs[REG_PID_POSITION_KP_H]))/100.0f;
//				float const pid_pos_ki = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KI_L],regs[REG_PID_POSITION_KI_H]))/100.0f;
//				float const pid_pos_kd = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KD_L],regs[REG_PID_POSITION_KD_H]))/100.0f;
//				float const pid_vel_kff = (float)(MAKE_SHORT(regs[REG_PID_VELOCITY_KFF_L],regs[REG_PID_VELOCITY_KFF_H]))/1000.0f;
//				float const pid_acc_kff = (float)(MAKE_SHORT(regs[REG_PID_ACCELERATION_KFF_L],regs[REG_PID_ACCELERATION_KFF_H]))/100000.0f;
//				float const velocity_feed_forward = pid_vel_kff * setpoint_velocity_dps;
//				float const acceleration_feed_forward = pid_acc_kff * setpoint_acceleration_dpss;
//				setpoint_current_ma =
//						ALPHA_CURRENT_SETPOINT * (
//								velocity_feed_forward +
//								acceleration_feed_forward +
//								pid_process_ex(
//										&pid_position,
//										error_position,
//										error_velocity,
//										pid_pos_kp,
//										pid_pos_ki,
//										pid_pos_kd,
//										LIMIT_PID_POSITION_WINDUP
//								)
//						) +
//						(1.0f-ALPHA_CURRENT_SETPOINT) * setpoint_current_ma;
//			}
//			// compute pwm setpoint from goal current using a PI
//			{
//				float const current_limit = (float)(MAKE_SHORT(regs[REG_GOAL_CURRENT_MA_L],regs[REG_GOAL_CURRENT_MA_H]));
//				setpoint_current_ma = fconstrain(setpoint_current_ma,-current_limit,current_limit);
//				float const error_current = setpoint_current_ma - present_current_ma_on;
//				float const pid_current_kp = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KP_L],regs[REG_PID_CURRENT_KP_H]))/100.0f;
//				float const pid_current_ki = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KI_L],regs[REG_PID_CURRENT_KI_H]))/100.0f;
//				float const pid_current_kff = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KFF_L],regs[REG_PID_CURRENT_KFF_H]))/100.0f;
//				setpoint_pwm = ALPHA_PWM_SETPOINT * ( pid_current_kff * setpoint_current_ma +	pid_process(&pid_current,error_current,pid_current_kp,pid_current_ki,0.0f,LIMIT_PID_CURRENT_WINDUP) ) + (1.0f-ALPHA_PWM_SETPOINT) * setpoint_pwm ;
//				float const pwm_limit = (float)(MAKE_SHORT(regs[REG_GOAL_PWM_100_L],regs[REG_GOAL_PWM_100_H]));
//				setpoint_pwm = fconstrain(setpoint_pwm,-pwm_limit,pwm_limit);
//			}
			setpoint_pwm = 0.0f;
			// mode change
			if(regs[REG_CONTROL_MODE]!=REG_CONTROL_MODE_POSITION)
			{
				APP_Control_Reset();
				current_control_mode = regs[REG_CONTROL_MODE];
			}
			break;
		case REG_CONTROL_MODE_CURRENT:
			if(entering_state)
			{
				entering_state = false;
				// init goal RAM registers according this control mode
				regs[REG_GOAL_CURRENT_MA_L] = 0;
				regs[REG_GOAL_CURRENT_MA_H] = 0;
				// init limit RAM registers according this control mode
				regs[REG_GOAL_PWM_100_L] = regs[REG_MAX_PWM_100_L];
				regs[REG_GOAL_PWM_100_H] = regs[REG_MAX_PWM_100_H];
				// reset unused RAM registers
				regs[REG_GOAL_POSITION_DEG_L] = LOW_BYTE((int16_t)(present_position_deg*10.0f));
				regs[REG_GOAL_POSITION_DEG_H] = HIGH_BYTE((int16_t)(present_position_deg*10.0f));
				regs[REG_GOAL_VELOCITY_DPS_L] = 0;
				regs[REG_GOAL_VELOCITY_DPS_H] = 0;
			}
			{
				float const goal_current = (int16_t)(MAKE_SHORT(regs[REG_GOAL_CURRENT_MA_L],regs[REG_GOAL_CURRENT_MA_H]));
				//float const current_limit = (float)(MAKE_SHORT(regs[REG_MAX_CURRENT_MA_L],regs[REG_MAX_CURRENT_MA_H]));
				//setpoint_current_ma = fconstrain(goal_current,-current_limit,current_limit);
				setpoint_current_ma = goal_current;
				// compute current error
				float const error_current = setpoint_current_ma - present_current_ma_on;
				// compute pwm setpoint from current error using a PI
				float const pid_current_kp = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KP_L],regs[REG_PID_CURRENT_KP_H]))/100.0f;
				float const pid_current_ki = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KI_L],regs[REG_PID_CURRENT_KI_H]))/1000.0f;
				float const pid_current_kff = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KFF_L],regs[REG_PID_CURRENT_KFF_H]))/100.0f;
				float const pwm_limit = (float)(MAKE_SHORT(regs[REG_GOAL_PWM_100_L],regs[REG_GOAL_PWM_100_H]));
				setpoint_pwm =
						ALPHA_PWM_SETPOINT * (
								pid_current_kff * setpoint_current_ma +
								pid_process_antiwindup_clamp(
										&pid_current,
										error_current,
										pid_current_kp,
										pid_current_ki,
										0.0f,
										pwm_limit,
										0.0f
								)
							) +
							(1.0f-ALPHA_PWM_SETPOINT) * setpoint_pwm ;
			}
			// mode change
			if(regs[REG_CONTROL_MODE]!=REG_CONTROL_MODE_CURRENT)
			{
				APP_Control_Reset();
				current_control_mode = regs[REG_CONTROL_MODE];
			}
			break;
		case REG_CONTROL_MODE_PWM:
			if(entering_state)
			{
				entering_state = false;
				// init goal RAM registers according this control mode
				regs[REG_GOAL_PWM_100_L] = 0;
				regs[REG_GOAL_PWM_100_H] = 0;
				// reset unused RAM registers
				regs[REG_GOAL_POSITION_DEG_L] = LOW_BYTE((int16_t)(present_position_deg*10.0f));
				regs[REG_GOAL_POSITION_DEG_H] = HIGH_BYTE((int16_t)(present_position_deg*10.0f));
				regs[REG_GOAL_VELOCITY_DPS_L] = 0;
				regs[REG_GOAL_VELOCITY_DPS_H] = 0;
				regs[REG_GOAL_CURRENT_MA_L] = 0;
				regs[REG_GOAL_CURRENT_MA_H] = 0;
			}
			// compute pwm setpoint from goal pwm
			{
				float const goal_pwm = (int16_t)(MAKE_SHORT(regs[REG_GOAL_PWM_100_L],regs[REG_GOAL_PWM_100_H]));
				float const pwm_limit = (float)(MAKE_SHORT(regs[REG_MAX_PWM_100_L],regs[REG_MAX_PWM_100_H]));
				setpoint_pwm = fconstrain(goal_pwm,-pwm_limit,pwm_limit);
			}
			// mode change
			if(regs[REG_CONTROL_MODE]!=REG_CONTROL_MODE_PWM)
			{
				APP_Control_Reset();
				current_control_mode = regs[REG_CONTROL_MODE];
			}
			break;
		}

		// common
		float const pwm_inv = regs[REG_INV_ROTATION_MOTOR] > 0 ? -1.0f : 1.0f;
		float pwm = pwm_inv * setpoint_pwm;

		// apply pwm
		if(pwm>0.0f)
		{
			//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm);
			//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MOTOR_PWM_COAST);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MOTOR_PWM_BRAKE);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MOTOR_PWM_BRAKE-pwm);


		}
		else if(pwm<0.0f)
		{
			//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MOTOR_PWM_COAST);
			//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,-pwm);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MOTOR_PWM_BRAKE+pwm);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MOTOR_PWM_BRAKE);
		}
		else
		{
			// motor brake
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MOTOR_PWM_BRAKE);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MOTOR_PWM_BRAKE);
		}
	}
	else
	{
		APP_Control_Reset();
		// motor brake
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MOTOR_PWM_BRAKE);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MOTOR_PWM_BRAKE);
	}

	// live update of RAM regs
	regs[REG_PRESENT_POSITION_DEG_L] = LOW_BYTE((uint16_t)(present_position_deg*10.0f));
	regs[REG_PRESENT_POSITION_DEG_H] = HIGH_BYTE((uint16_t)(present_position_deg*10.0f));

	regs[REG_PRESENT_VELOCITY_DPS_L] = LOW_BYTE((int16_t)present_velocity_dps);
	regs[REG_PRESENT_VELOCITY_DPS_H] = HIGH_BYTE((int16_t)present_velocity_dps);

	regs[REG_PRESENT_CURRENT_MA_L] = LOW_BYTE((int16_t)present_current_ma_on);
	regs[REG_PRESENT_CURRENT_MA_H] = HIGH_BYTE((int16_t)present_current_ma_on);

	regs[REG_PRESENT_VOLTAGE] = (uint8_t)(voltage_input_adc/4096.0f*3.3f*24.2f/2.2f*10.0f*VOLTAGE_CALIBRATION);
	regs[REG_PRESENT_TEMPERATURE] = 0;

	float moving_threshold = regs[REG_MOVING_THRESHOLD_DPS];
	regs[REG_MOVING] = ( fabs(present_velocity_dps) > moving_threshold ) ? 1 : 0;

	regs[REG_SETPOINT_POSITION_DEG_L] = LOW_BYTE((uint16_t)(setpoint_position_deg*10.0f));
	regs[REG_SETPOINT_POSITION_DEG_H] = HIGH_BYTE((uint16_t)(setpoint_position_deg*10.0f));

	//regs[REG_SETPOINT_VELOCITY_DPS_L] = LOW_BYTE((int16_t)setpoint_velocity_dps);
	//regs[REG_SETPOINT_VELOCITY_DPS_H] = HIGH_BYTE((int16_t)setpoint_velocity_dps);
	regs[REG_SETPOINT_VELOCITY_DPS_L] = LOW_BYTE((int16_t)pid_position.err_integral);
	regs[REG_SETPOINT_VELOCITY_DPS_H] = HIGH_BYTE((int16_t)pid_position.err_integral);

	regs[REG_SETPOINT_CURRENT_MA_L] = LOW_BYTE((int16_t)setpoint_current_ma);
	regs[REG_SETPOINT_CURRENT_MA_H] = HIGH_BYTE((int16_t)setpoint_current_ma);

	regs[REG_SETPOINT_PWM_100_L] = LOW_BYTE((int16_t)setpoint_pwm);
	regs[REG_SETPOINT_PWM_100_H] = HIGH_BYTE((int16_t)setpoint_pwm);

	regs[POSITION_INPUT_ADC_L] = LOW_BYTE((uint16_t)position_input_adc);
	regs[POSITION_INPUT_ADC_H] = HIGH_BYTE((uint16_t)position_input_adc);

	regs[CURRENT_INPUT_ADC_L] = LOW_BYTE((uint16_t)motor_current_input_adc);
	regs[CURRENT_INPUT_ADC_H] = HIGH_BYTE((uint16_t)motor_current_input_adc);

	regs[PRESENT_CURRENT_ON_L] = LOW_BYTE((int16_t)present_current_ma_on);
	regs[PRESENT_CURRENT_ON_H] = HIGH_BYTE((int16_t)present_current_ma_on);

	regs[PRESENT_CURRENT_OFF_L] = LOW_BYTE((int16_t)present_current_ma_off);
	regs[PRESENT_CURRENT_OFF_H] = HIGH_BYTE((int16_t)present_current_ma_off);

	regs[REG_EST_CURRENT_SENSE_B_L] = LOW_BYTE((uint16_t)b);
	regs[REG_EST_CURRENT_SENSE_B_H] = HIGH_BYTE((uint16_t)b);
// steps
	++counter;
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc==&hadc1)
	{
		motor_current_input_adc = ADC_DMA[0];
		position_input_adc = ALPHA_POSITION * (float)(ADC_DMA[1]) + (1.0f-ALPHA_POSITION) * position_input_adc;
		voltage_input_adc = ALPHA_VOLTAGE * (float)(ADC_DMA[2]) + (1.0f-ALPHA_VOLTAGE) * voltage_input_adc;

		// motor current is always positive in FORWARD and REVERSE drive phase (unipolar current sensing),
		// get sign from PWM setpoint
		float const pwm_sign = ( setpoint_pwm < 0.0f ) ? -1.0f : 1.0f;
		float const pwm_ratio = fabsf(setpoint_pwm)/100.0f;
		float const pwm_ratio_inv = 0.99f-pwm_ratio;
		// motor current cannot be estimated if pulse is too short for ADC conversion delay
		bool pwm_too_small = pwm_ratio < 0.02f;

		// instant motor current = ( ADC - b ) / a
		float const a = (float)(MAKE_SHORT(regs[REG_CAL_CURRENT_SENSE_A_L],regs[REG_CAL_CURRENT_SENSE_A_H]));
		// init b value from EEPROM
		if(b==0)
		{
			b = (float)(MAKE_SHORT(regs[REG_CAL_CURRENT_SENSE_B_L],regs[REG_CAL_CURRENT_SENSE_B_H]));
		}

		// There are two UPDATE EVENT per period
		// ADC is triggered twice per period by PWM TIM4 (centered mode)
		// We will measure ON and OFF instant motor current
		// when PWM is ON, TIMER COUNTER is 0
		// when PWM is OFF, TIMER COUNTER is 99

		// in DRIVE phase
		// PWM is ON
		if(__HAL_TIM_GET_COUNTER(&htim4) > 50)
		{
			if(pwm_too_small)
			{
				// We suppose that average current is very low ==> zero
				present_current_ma_on = (1.0f - ALPHA_CURRENT_SENSE)*present_current_ma_on;
			}
			else
			{
				present_current_ma_on = (1.0f - ALPHA_CURRENT_SENSE)*present_current_ma_on + ALPHA_CURRENT_SENSE * (float)((int16_t)motor_current_input_adc-b)/a*1000.0f*pwm_sign*pwm_ratio;
			}
		}
		// in COAST phase
		// PWM is OFF
		else
		{
			// self-calibrate ADC offset (b)
			if(setpoint_pwm==0.0f)
			{
				b = ALPHA_B * (float)motor_current_input_adc + (1.0f-ALPHA_B)*b;
			}

			if(pwm_too_small)
			{
				// We suppose that average current is very low ==> zero
				present_current_ma_off = (1.0f - ALPHA_CURRENT_SENSE)*present_current_ma_off;
			}
			else
			{
				present_current_ma_off = (1.0f - ALPHA_CURRENT_SENSE)*present_current_ma_off + ALPHA_CURRENT_SENSE * (float)((int16_t)motor_current_input_adc-b)/a*1000.0f*pwm_sign*pwm_ratio_inv;
			}
		}

		// restart adc conv trigger
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_DMA,3);

	}

//	//else if(hadc==&hadc2)
//	{
//		if(__HAL_TIM_GET_COUNTER(&htim4) < 50)
//		{
//			position_input_adc = ALPHA_POSITION * HAL_ADC_GetValue(&hadc2) + (1.0f-ALPHA_POSITION) * position_input_adc;
//		}
//	}
}


