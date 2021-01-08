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
#define ALPHA_POSITION				0.05f // (default:0.05) F = 20kHz ==> Fc (-3dB) = 170.0Hz
#define ALPHA_VOLTAGE				0.05f // (default:0.05) F = 20kHz ==> Fc (-3dB) = 170.0Hz
#define ALPHA_CURRENT_SENSE			0.05f // (default:0.05) F = 20kHz ==> Fc (-3dB) = 170.0Hz
#define ALPHA_CURRENT_SENSE_OFFSET 	0.001f


#define ALPHA_VELOCITY			0.12f // (default:0.12) F = 1000Hz ==> Fc (-3dB) = 20Hz
#define ALPHA_CURRENT_SETPOINT 	0.96f // (default:0.12) F = 1000Hz ==> Fc (-3dB) = 20Hz
#define ALPHA_PWM_SETPOINT		0.12f // (default:0.12) F = 1000Hz ==> Fc (-3dB) = 20.0Hz
#define VOLTAGE_CALIBRATION 	1.08f

// Private Variables

// raw sensor inputs
volatile uint16_t ADC_DMA[3] = { 0,0,0 };
// filtered sensor inputs
static float motor_current_input_adc = 0.0f;
static float position_input_adc = 0.0f;
static float voltage_input_adc = 0.0f;
static float motor_current_input_adc_offset = 0.0f;
static float pwm_sign = 0.0f;
static float pwm_ratio = 0.0f;
// scaled sensor inputs
static float present_motor_current_ma = 0.0f;
static float present_position_deg = 0.0f;
static float present_voltage_0v1 = 0.0f;
// setpoints
static float setpoint_pwm = 0.0f;
static float setpoint_current_ma = 0.0f;
static float setpoint_acceleration_dpss = 0.0f;
static float setpoint_velocity_dps = 0.0f;
static float setpoint_position_deg = 0.0f;
// control loop state
static bool entering_state = true;
static uint32_t current_control_mode = REG_CONTROL_MODE_POSITION_TORQUE;
static uint16_t const period_us = (uint16_t)(1000000.0f/LOOP_FREQUENCY_HZ);
static uint32_t counter = 0;
// PIDs
static pid_context_t pid_position;
static pid_context_t pid_current;
// variables
static float present_velocity_dps = 0.0f;
static float last_present_position_deg = 0.0f;
static float last_setpoint_velocity_dps = 0.0f;

void scale_all_sensors()
{
	// scale motor current sense (unit:mA) and estimated average motor current with sign (using PWM ratio and setpoint PWM sign)
	float const a = (float)(MAKE_SHORT(regs[REG_CAL_CURRENT_SENSE_A_L],regs[REG_CAL_CURRENT_SENSE_A_H]));
	present_motor_current_ma = (motor_current_input_adc-motor_current_input_adc_offset)/a*1000.0f*pwm_sign*pwm_ratio;

	// scale position (unit:degrees)
	float const min_position_adc = (float)(MAKE_SHORT(regs[REG_MIN_POSITION_ADC_L],regs[REG_MIN_POSITION_ADC_H]));
	float const max_position_adc = (float)(MAKE_SHORT(regs[REG_MAX_POSITION_ADC_L],regs[REG_MAX_POSITION_ADC_H]));
	float const max_rotation_deg = (float)(regs[REG_MAX_ROTATION_DEG]);
	present_position_deg = fmap(position_input_adc,min_position_adc,max_position_adc,0.0f,max_rotation_deg);
	// potentiometer leads maybe inverted, user can reverse polarity of potentiometer (EEPROM parameter)
	if(regs[REG_INV_ROTATION_SENSOR] > 0)
		present_position_deg =  max_rotation_deg-present_position_deg;

	// scale voltage (unit:0.1V)
	present_voltage_0v1 = voltage_input_adc/4096.0f*3.3f*24.2f/2.2f*10.0f*VOLTAGE_CALIBRATION;
}

// called once after SW REBBOT or HW RESET, and every time entering a new control loop mode
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
	last_present_position_deg = present_position_deg;
	// when re-entering in the control mode 'position', avoid glitch from past goal position
	regs[REG_GOAL_POSITION_DEG_L] = LOW_BYTE((int16_t)(present_position_deg*10.0f));
	regs[REG_GOAL_POSITION_DEG_H] = HIGH_BYTE((int16_t)(present_position_deg*10.0f));
}

// called once after SW REBOOT or HW RESET
void APP_Control_Init()
{
	// reset (EWMA) filtered sensor inputs
	motor_current_input_adc = 0.0f;
	position_input_adc = 0.0f; // NOTE : init by zero will delay the present position estimation by 1 ms at least
	voltage_input_adc = 0.0f; // NOTE : init by zero will delay the present voltage estimation by 1 ms at least
	motor_current_input_adc_offset = (float)(MAKE_SHORT(regs[REG_CAL_CURRENT_SENSE_B_L],regs[REG_CAL_CURRENT_SENSE_B_H]));

	// force motor in coast
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MOTOR_PWM_COAST);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MOTOR_PWM_COAST);
	// start motor PWM generation
	HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim15);
	// start OP AMP
	HAL_OPAMP_Start(&hopamp1);
	// start ADC
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_DMA,3);

	// 2ms delay for filtered sensor inputs to stabilize
	HAL_Delay(1);
	scale_all_sensors();
	HAL_Delay(1);
	scale_all_sensors();

	// reset all state control loop variables
	APP_Control_Reset();
}

// called from main loop
void APP_Control_Process()
{
	// apply 1ms period
	uint16_t current_time_us = __HAL_TIM_GET_COUNTER(&htim15);
	if(current_time_us<period_us)
		return;
	__HAL_TIM_SET_COUNTER(&htim15,(current_time_us-period_us));

	// acquire motor current, position and voltage (see ADC DMA completed conversion callback)

	// scale sensor at process rate
	scale_all_sensors();

	// compute velocity from position (derivative), and filter velocity (EWMA)
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
				float const error_current = setpoint_current_ma - present_motor_current_ma;
				// compute pwm setpoint from current error using a PI
				float const pid_current_kp = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KP_L],regs[REG_PID_CURRENT_KP_H]))/1000.0f;
				float const pid_current_ki = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KI_L],regs[REG_PID_CURRENT_KI_H]))/100.0f;
				float const pid_current_kff = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KFF_L],regs[REG_PID_CURRENT_KFF_H]))/100.0f;
				float const pwm_limit = (float)(MAKE_SHORT(regs[REG_GOAL_PWM_100_L],regs[REG_GOAL_PWM_100_H]));
				setpoint_pwm =
						ALPHA_PWM_SETPOINT * (
								pid_process_antiwindup_clamp_with_ff(
										&pid_current,
										error_current,
										pid_current_kp,
										pid_current_ki,
										0.0f,
										pwm_limit,
										0.0f,
										pid_current_kff * setpoint_current_ma
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


		case REG_CONTROL_MODE_VELOCITY_PROFIL_POSITION_TORQUE:
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
			{
				// limit goal position
				float const min_goal_position_deg = (float)(MAKE_SHORT(regs[REG_MIN_POSITION_DEG_L],regs[REG_MIN_POSITION_DEG_H]));
				float const max_goal_position_deg = (float)(MAKE_SHORT(regs[REG_MAX_POSITION_DEG_L],regs[REG_MAX_POSITION_DEG_H]));
				float goal_position_deg = 0.1f * (float)(MAKE_SHORT(regs[REG_GOAL_POSITION_DEG_L],regs[REG_GOAL_POSITION_DEG_H]));
				goal_position_deg = fconstrain(goal_position_deg,min_goal_position_deg,max_goal_position_deg);
				// compute position setpoint from goal position
				float const max_velocity_dps = (float)(MAKE_SHORT(regs[REG_GOAL_VELOCITY_DPS_L],regs[REG_GOAL_VELOCITY_DPS_H]));
				float const max_acceleration_dpss = (float)(MAKE_SHORT(regs[REG_MAX_ACCELERATION_DPSS_L],regs[REG_MAX_ACCELERATION_DPSS_H]));
				// trapezoidal profile for setpoint position
				// compute remaining distance between setpoint position to goal position
				float const remaining_distance_deg = goal_position_deg - setpoint_position_deg;
				// compute maximun velocity to be able to stop at goal position
				float vmax = sqrtf( 2.0f * max_acceleration_dpss * fabsf(remaining_distance_deg) );
				// restore sign
				vmax = ( remaining_distance_deg>0.0f) ? vmax : -vmax;
				// limit maximum velocity, when far from stop
				vmax = fconstrain(vmax,-max_velocity_dps,max_velocity_dps);
				// compute distance between maximun velocity and current velocity
				float delta_v = vmax - setpoint_velocity_dps;
				// now compute new velocity according acceleration
				setpoint_velocity_dps += fconstrain(delta_v, (-max_acceleration_dpss/LOOP_FREQUENCY_HZ), (max_acceleration_dpss/LOOP_FREQUENCY_HZ));
				// now compute new position
				setpoint_position_deg += (setpoint_velocity_dps/LOOP_FREQUENCY_HZ);
				// now compute acceleration
				setpoint_acceleration_dpss = (setpoint_velocity_dps - last_setpoint_velocity_dps)*LOOP_FREQUENCY_HZ;
				last_setpoint_velocity_dps =  setpoint_velocity_dps;
				// compute current setpoint from position setpoint using a PID position and velocity/acceleration feed forwards
				float const pid_vel_kff = (float)(MAKE_SHORT(regs[REG_PID_VELOCITY_KFF_L],regs[REG_PID_VELOCITY_KFF_H]))/1000.0f;
				float const pid_acc_kff = (float)(MAKE_SHORT(regs[REG_PID_ACCELERATION_KFF_L],regs[REG_PID_ACCELERATION_KFF_H]))/100000.0f;
				float const velocity_feed_forward = pid_vel_kff * setpoint_velocity_dps;
				float const acceleration_feed_forward = pid_acc_kff * setpoint_acceleration_dpss;
				// compute position error
				float const error_position = setpoint_position_deg - present_position_deg;
				// compute current setpoint from position error using a PID position
				float const pid_pos_kp = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KP_L],regs[REG_PID_POSITION_KP_H]))/100.0f;
				float const pid_pos_ki = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KI_L],regs[REG_PID_POSITION_KI_H]))/1000.0f;
				float const pid_pos_kd = (float)(MAKE_SHORT(regs[REG_PID_POSITION_KD_L],regs[REG_PID_POSITION_KD_H]))/10.0f;
				float const current_limit = (float)(MAKE_SHORT(regs[REG_GOAL_CURRENT_MA_L],regs[REG_GOAL_CURRENT_MA_H]));
				setpoint_current_ma =
						ALPHA_CURRENT_SETPOINT * (
								pid_process_antiwindup_clamp_with_ff(
										&pid_position,
										error_position,
										pid_pos_kp,
										pid_pos_ki,
										pid_pos_kd,
										current_limit,
										ALPHA_VELOCITY,
										velocity_feed_forward+acceleration_feed_forward
								)
						) +
						(1.0f-ALPHA_CURRENT_SETPOINT) * setpoint_current_ma;
			}
			{
				// compute current error
				float const error_current = setpoint_current_ma - present_motor_current_ma;
				// compute pwm setpoint from current error using a PI
				float const pid_current_kp = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KP_L],regs[REG_PID_CURRENT_KP_H]))/1000.0f;
				float const pid_current_ki = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KI_L],regs[REG_PID_CURRENT_KI_H]))/100.0f;
				float const pid_current_kff = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KFF_L],regs[REG_PID_CURRENT_KFF_H]))/100.0f;
				float const pwm_limit = (float)(MAKE_SHORT(regs[REG_GOAL_PWM_100_L],regs[REG_GOAL_PWM_100_H]));
				setpoint_pwm =
						ALPHA_PWM_SETPOINT * (
								pid_process_antiwindup_clamp_with_ff(
										&pid_current,
										error_current,
										pid_current_kp,
										pid_current_ki,
										0.0f,
										pwm_limit,
										0.0f,
										pid_current_kff * setpoint_current_ma
								)
							) +
							(1.0f-ALPHA_PWM_SETPOINT) * setpoint_pwm ;
			}
			// mode change
			if(regs[REG_CONTROL_MODE]!=REG_CONTROL_MODE_VELOCITY_PROFIL_POSITION_TORQUE)
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
				float const error_current = setpoint_current_ma - present_motor_current_ma;
				// compute pwm setpoint from current error using a PI
				float const pid_current_kp = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KP_L],regs[REG_PID_CURRENT_KP_H]))/100.0f;
				float const pid_current_ki = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KI_L],regs[REG_PID_CURRENT_KI_H]))/1000.0f;
				float const pid_current_kff = (float)(MAKE_SHORT(regs[REG_PID_CURRENT_KFF_L],regs[REG_PID_CURRENT_KFF_H]))/100.0f;
				float const pwm_limit = (float)(MAKE_SHORT(regs[REG_GOAL_PWM_100_L],regs[REG_GOAL_PWM_100_H]));
				setpoint_pwm =
						ALPHA_PWM_SETPOINT * (
								pid_process_antiwindup_clamp_with_ff(
										&pid_current,
										error_current,
										pid_current_kp,
										pid_current_ki,
										0.0f,
										pwm_limit,
										0.0f,
										pid_current_kff * setpoint_current_ma
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

		// motor leads maybe inverted, user can reverse polarity of motor (EEPROM parameter)
		float const pwm_inv = regs[REG_INV_ROTATION_MOTOR] > 0 ? -1.0f : 1.0f;
		float pwm = pwm_inv * setpoint_pwm;

		// apply pwm
		if(pwm>=0.0f)
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MOTOR_PWM_BRAKE);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MOTOR_PWM_BRAKE-pwm);


		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MOTOR_PWM_BRAKE+pwm);
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

	// Note : This is an unipolar current sensing architecture,
	// then motor current is always positive in FORWARD and REVERSE drive phase,
	// and zero in BRAKE phases. So, the sign of the current, is build from the sign of the PWM setpoint
	pwm_sign = ( setpoint_pwm < 0.0f ) ? -1.0f : 1.0f;
	pwm_ratio = fabsf(setpoint_pwm)/100.0f;

	// live update of RAM regs
	regs[REG_PRESENT_POSITION_DEG_L] = LOW_BYTE((uint16_t)(present_position_deg*10.0f));
	regs[REG_PRESENT_POSITION_DEG_H] = HIGH_BYTE((uint16_t)(present_position_deg*10.0f));

	regs[REG_PRESENT_VELOCITY_DPS_L] = LOW_BYTE((int16_t)present_velocity_dps);
	regs[REG_PRESENT_VELOCITY_DPS_H] = HIGH_BYTE((int16_t)present_velocity_dps);

	regs[REG_PRESENT_CURRENT_MA_L] = LOW_BYTE((int16_t)present_motor_current_ma);
	regs[REG_PRESENT_CURRENT_MA_H] = HIGH_BYTE((int16_t)present_motor_current_ma);

	regs[REG_PRESENT_VOLTAGE] = (uint8_t)(present_voltage_0v1);
	regs[REG_PRESENT_TEMPERATURE] = 0;

	float moving_threshold = regs[REG_MOVING_THRESHOLD_DPS];
	regs[REG_MOVING] = ( fabs(present_velocity_dps) > moving_threshold ) ? 1 : 0;

	regs[REG_SETPOINT_POSITION_DEG_L] = LOW_BYTE((uint16_t)(setpoint_position_deg*10.0f));
	regs[REG_SETPOINT_POSITION_DEG_H] = HIGH_BYTE((uint16_t)(setpoint_position_deg*10.0f));

	regs[REG_SETPOINT_VELOCITY_DPS_L] = LOW_BYTE((int16_t)setpoint_velocity_dps);
	regs[REG_SETPOINT_VELOCITY_DPS_H] = HIGH_BYTE((int16_t)setpoint_velocity_dps);
	//regs[REG_SETPOINT_VELOCITY_DPS_L] = LOW_BYTE((int16_t)pid_position.err_integral); // DEBUG
	//regs[REG_SETPOINT_VELOCITY_DPS_H] = HIGH_BYTE((int16_t)pid_position.err_integral); // DEBUG

	regs[REG_SETPOINT_CURRENT_MA_L] = LOW_BYTE((int16_t)setpoint_current_ma);
	regs[REG_SETPOINT_CURRENT_MA_H] = HIGH_BYTE((int16_t)setpoint_current_ma);

	regs[REG_SETPOINT_PWM_100_L] = LOW_BYTE((int16_t)setpoint_pwm);
	regs[REG_SETPOINT_PWM_100_H] = HIGH_BYTE((int16_t)setpoint_pwm);

	regs[REG_MOTOR_CURRENT_INPUT_ADC_L] = LOW_BYTE((uint16_t)motor_current_input_adc);
	regs[REG_MOTOR_CURRENT_INPUT_ADC_H] = HIGH_BYTE((uint16_t)motor_current_input_adc);

	regs[REG_MOTOR_CURRENT_INPUT_ADC_OFFSET_L] = LOW_BYTE((uint16_t)motor_current_input_adc_offset);
	regs[REG_MOTOR_CURRENT_INPUT_ADC_OFFSET_H] = HIGH_BYTE((uint16_t)motor_current_input_adc_offset);

	regs[REG_POSITION_INPUT_ADC_L] = LOW_BYTE((uint16_t)position_input_adc);
	regs[REG_POSITION_INPUT_ADC_H] = HIGH_BYTE((uint16_t)position_input_adc);

	regs[REG_VOLTAGE_INPUT_ADC_L] = LOW_BYTE((uint16_t)voltage_input_adc);
	regs[REG_VOLTAGE_INPUT_ADC_H] = HIGH_BYTE((uint16_t)voltage_input_adc);

	// steps
	++counter;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc==&hadc1)
	{
		// Filter (EWMA) position and voltage ADC samples
		voltage_input_adc = ALPHA_VOLTAGE * (float)(ADC_DMA[2]) + (1.0f-ALPHA_VOLTAGE) * voltage_input_adc;
		position_input_adc = ALPHA_POSITION * (float)(ADC_DMA[1]) + (1.0f-ALPHA_POSITION) * position_input_adc;

		// Filter (EWMA) motor current sense ADC samples

		// Note : In center aligned mode, two periods of TIM4 are used for motor PWM generation
		// TIM4 is 40KHz, motor PWM is 20KHz
		// So ADC is triggered twice per motor PWM period by TIM4
		// We will measure ON and OFF instant motor current

		// In FORWARD or REVERSE DRIVE phases, PWM is ON, counter decreases
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
		{
			// filter motor current
			motor_current_input_adc = ALPHA_CURRENT_SENSE*(float)(ADC_DMA[0]) + (1.0f-ALPHA_CURRENT_SENSE)*motor_current_input_adc;
		}
		// In BRAKE phase, PWM is OFF, counter increases
		else
		{
			// self-calibrate ADC offset (b) when motor is stopped
			if(setpoint_pwm==0.0f)
			{
				motor_current_input_adc_offset = ALPHA_CURRENT_SENSE_OFFSET*(float)(ADC_DMA[0]) + (1.0f-ALPHA_CURRENT_SENSE_OFFSET)*motor_current_input_adc_offset;
			}
		}

		// restart ADC
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_DMA,3);
	}
}


