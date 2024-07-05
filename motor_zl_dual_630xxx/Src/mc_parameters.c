
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"
#include "r1_ps_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

/**
  * @brief  Current sensor parameters Motor 1 - single shunt phase shift
  */
//cstat !MISRAC2012-Rule-8.4
const R1_Params_t R1_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx                  = ADC1,
  .IChannel              = 3,

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER,
  .TMin                  = TMIN,
  .TSample               = (uint16_t)(TBEFORE),
  .TIMx                  = TIM1,
  .DMAx                  = DMA1,
  .DMAChannelX           = LL_DMA_CHANNEL_1,
  .hTADConv              = (uint16_t)((ADC_SAR_CYCLES+ADC_TRIG_CONV_LATENCY_CYCLES) * (ADV_TIM_CLK_MHz/ADC_CLK_MHz)),

/* Internal OPAMP common settings --------------------------------------------*/
  .OPAMP_Selection       = OPAMP,

/* Internal COMP settings ----------------------------------------------------*/
  .CompOCPSelection      = COMP2,
  .CompOCPInvInput_MODE  = DAC_MODE,

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = 8092,

};
/**
  * @brief  Current sensor parameters Motor 2 - single shunt phase shift
  */
//cstat !MISRAC2012-Rule-8.4
const R1_Params_t R1_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION2,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx                  = ADC2,
  .IChannel              = 3,

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER2,
  .TMin                  = TMIN2,
  .TSample               = (uint16_t)(TBEFORE2),
  .TIMx                  = TIM8,
  .DMAx                  = DMA2,
  .DMAChannelX           = LL_DMA_CHANNEL_1,
  .hTADConv              = (uint16_t)((ADC_SAR_CYCLES+ADC_TRIG_CONV_LATENCY_CYCLES) * (ADV_TIM_CLK_MHz/ADC_CLK_MHz)),

/* Internal OPAMP common settings --------------------------------------------*/
  .OPAMP_Selection       = OPAMP2,

/* Internal COMP settings ----------------------------------------------------*/
  .CompOCPSelection      = COMP2,
  .CompOCPInvInput_MODE  = DAC_MODE,

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = 8092,

};

ScaleParams_t scaleParams_M1 =
{
 .voltage = NOMINAL_BUS_VOLTAGE_V/(1.73205 * 32767), /* sqrt(3) = 1.73205 */
 .current = CURRENT_CONV_FACTOR_INV,
 .frequency = (1.15 * MAX_APPLICATION_SPEED_UNIT * U_RPM)/(32768* SPEED_UNIT)
};

ScaleParams_t scaleParams_M2 =
{
 .voltage = NOMINAL_BUS_VOLTAGE_V2/(1.73205 * 32767), /* sqrt(3) = 1.73205 */
 .current = CURRENT_CONV_FACTOR_INV2,
 .frequency = (1.15 * MAX_APPLICATION_SPEED_UNIT2 * U_RPM)/(32768* SPEED_UNIT)
};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

