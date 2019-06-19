/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "stm32f0xx_hal.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );
static TIM_HandleTypeDef htim14;
static uint16_t timeout = 0;
static uint16_t counter = 0;
/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    htim14.Instance = TIM14;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Prescaler = (HAL_RCC_GetPCLK1Freq()/1000000);
    htim14.Init.Period = 50 - 1;
    timeout = usTim1Timerout50us;
    return HAL_OK == HAL_TIM_Base_Init(&htim14) ? TRUE : FALSE;
}

inline void
vMBPortTimersEnable(  )
{
    counter = timeout;
    HAL_TIM_Base_Start_IT(&htim14);
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
}

inline void
vMBPortTimersDisable(  )
{
    HAL_TIM_Base_Stop_IT(&htim14);
    /* Disable any pending timers. */
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}
void TIM14_DAC1_IRQHandler(void) {
	/* TIM Update event */
    HAL_Delay(1000);
	if(__HAL_TIM_GET_FLAG(&htim14, TIM_FLAG_UPDATE) != RESET && __HAL_TIM_GET_IT_SOURCE(&htim14, TIM_IT_UPDATE) !=RESET) {
		__HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
		if (!--counter)
			prvvTIMERExpiredISR();
	}
}
