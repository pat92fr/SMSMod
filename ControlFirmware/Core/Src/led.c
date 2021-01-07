/*
 * led.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */


/* Includes ------------------------------------------------------------------*/
#include "Led.h"
#include <stdbool.h>

/* Private data --------------------------------------------------------------*/

static int blinkin_period[LED_COUNT]; // 0 = no blinking
static int blinkin_counter[LED_COUNT];
static bool blinkin_infinite[LED_COUNT];
static uint32_t last_state_change_time[LED_COUNT]; // ms

/* Private functions ---------------------------------------------------------*/

void stop_blinking(int id)
{
    blinkin_period[id]=0;
    blinkin_counter[id]=0;
    blinkin_infinite[id]=false;
    last_state_change_time[id]=0;
}

/* HAL functions ---------------------------------------------------------*/

void HAL_Led_Init(void)
{
    // Init private data & ALL OFF
    int id = 0;
    for(id=0;id<LED_COUNT;++id)
    {
        stop_blinking(id);
        HAL_GPIO_WritePin(hal_led_id_to_port[id],hal_led_id_to_pin[id],GPIO_PIN_SET);
    }
}

void HAL_Led_Process(void)
{
    uint32_t const current_time = HAL_GetTick();
    int id = 0;
    for(id=0;id<LED_COUNT;++id)
    {
        // have to blink ?
        if( blinkin_period[id]!=0 )
        {
            // is it time to blink ?
            if( current_time >= last_state_change_time[id]+blinkin_period[id] )
            {
                // blink
                if(HAL_GPIO_ReadPin(hal_led_id_to_port[id],hal_led_id_to_pin[id])==GPIO_PIN_RESET)
                    HAL_GPIO_WritePin(hal_led_id_to_port[id],hal_led_id_to_pin[id],GPIO_PIN_SET);
                else
                    HAL_GPIO_WritePin(hal_led_id_to_port[id],hal_led_id_to_pin[id],GPIO_PIN_RESET);
                // next time
                last_state_change_time[id]+=blinkin_period[id];
                if(!blinkin_infinite[id])
                {
                    --blinkin_counter[id];
                    if(blinkin_counter[id]==0)
                    {
                        stop_blinking(id);
                    }
                }
            }
            // else don't change led state
        }
        // else don't change led state
    }
}

void HAL_Led_Set(int id)
{
    stop_blinking(id);
    HAL_GPIO_WritePin(hal_led_id_to_port[id],hal_led_id_to_pin[id],GPIO_PIN_RESET);
}

void HAL_Led_Reset(int id)
{
    stop_blinking(id);
    HAL_GPIO_WritePin(hal_led_id_to_port[id],hal_led_id_to_pin[id],GPIO_PIN_SET);
}

void HAL_Led_Toggle(int id)
{
    stop_blinking(id);
    if(HAL_GPIO_ReadPin(hal_led_id_to_port[id],hal_led_id_to_pin[id])==GPIO_PIN_RESET)
        HAL_GPIO_WritePin(hal_led_id_to_port[id],hal_led_id_to_pin[id],GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(hal_led_id_to_port[id],hal_led_id_to_pin[id],GPIO_PIN_RESET);
}

int HAL_Led_Get(int id)
{
    if(HAL_GPIO_ReadPin(hal_led_id_to_port[id],hal_led_id_to_pin[id])==GPIO_PIN_RESET)
        return LED_ON;
    else
        return LED_OFF;
}

void HAL_Led_Blink(int id, int times, int period_ms)
{
    blinkin_period[id]=period_ms;
    blinkin_counter[id]=times!=0?times*2-1:0;
    blinkin_infinite[id]=(times==0);
    last_state_change_time[id]=HAL_GetTick();
    // ON at once
    HAL_GPIO_WritePin(hal_led_id_to_port[id],hal_led_id_to_pin[id],GPIO_PIN_RESET);
}


