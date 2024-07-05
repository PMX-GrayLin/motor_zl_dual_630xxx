
/**
  ******************************************************************************
  * @file    stm32g4xx_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control for the STM32G4 Family.
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
  * @ingroup STM32G4xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
//cstat -MISRAC2012-Rule-3.1
#include "mc_tasks.h"
//cstat +MISRAC2012-Rule-3.1
#include "motorcontrol.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32G4xx_IRQ_Handlers STM32G4xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC1_2_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA2_Channel1_IRQHandler(void);
void SPD_TIM_M1_IRQHandler(void);
void TIMx_UP_M2_IRQHandler(void);
void TIMx_BRK_M2_IRQHandler(void);
void SPD_TIM_M2_IRQHandler(void);

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */

  /* Shared IRQ management - begin */
  if (LL_ADC_IsActiveFlag_JEOS(ADC1))
  {
    /* Clear Flags M1 */
    LL_ADC_ClearFlag_JEOS(ADC1);
    /* Highfrequency task M1 */
    TSK_HighFrequencyTask(M1);

  }
  else if (LL_ADC_IsActiveFlag_JEOS(ADC2))
  {
    /* Clear Flags M2 */
    LL_ADC_ClearFlag_JEOS(ADC2);
    /* Highfrequency task M2 */
    TSK_HighFrequencyTask(M2);
  }

  /* USER CODE BEGIN HighFreq */

  /* USER CODE END HighFreq  */

  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  */
void SPD_TIM_M1_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */

  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (0U == LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx))
  {
    /* Nothing to do */
  }
  else
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    (void)HALL_TIMx_UP_IRQHandler(&HALL_M1);

    /* USER CODE BEGIN M1 HALL_Update */

    /* USER CODE END M1 HALL_Update   */
  }

  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx) != 0U)
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    (void)HALL_TIMx_CC_IRQHandler(&HALL_M1);

    /* USER CODE BEGIN M1 HALL_CC1 */

    /* USER CODE END M1 HALL_CC1 */
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */
}

/**
  * @brief  This function handles TIMx global interrupt request for M2 Speed Sensor.
  * @param  None
  */
void SPD_TIM_M2_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M2_IRQn 0 */

  /* USER CODE END SPD_TIM_M2_IRQn 0 */

  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M2.TIMx) != 0)
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M2.TIMx);
    (void)HALL_TIMx_UP_IRQHandler(&HALL_M2);

    /* USER CODE BEGIN M2 HALL_Update */

    /* USER CODE END M2 HALL_Update   */
  }
  else
  {
    /* Nothing to do */
  }

  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M2.TIMx))
  {
    LL_TIM_ClearFlag_CC1(HALL_M2.TIMx);
    (void)HALL_TIMx_CC_IRQHandler(&HALL_M2);

    /* USER CODE BEGIN M2 HALL_CC1 */

    /* USER CODE END M2 HALL_CC1 */
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN SPD_TIM_M2_IRQn 1 */

  /* USER CODE END SPD_TIM_M2_IRQn 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  */
void TIMx_UP_M1_IRQHandler(void)
{
 /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

 /* USER CODE END  TIMx_UP_M1_IRQn 0 */

  LL_TIM_ClearFlag_UPDATE(TIM1);
  (void)R1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
  TSK_DualDriveFIFOUpdate(M1);

 /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

 /* USER CODE END  TIMx_UP_M1_IRQn 1 */
}

void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */

  if (0U == LL_TIM_IsActiveFlag_BRK(TIM1))
  {
    /* Nothing to do */
  }
  else
  {
    LL_TIM_ClearFlag_BRK(TIM1);
    PWMC_OCP_Handler(&PWM_Handle_M1._Super);
  }

  if (0U == LL_TIM_IsActiveFlag_BRK2(TIM1))
  {
    /* Nothing to do */
  }
  else
  {
    LL_TIM_ClearFlag_BRK2(TIM1);
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, TIM1);
  }

  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here */
  MC_RunMotorControlTasks();

  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */
}

void DMA1_Channel1_IRQHandler (void)
{
  uint32_t tempReg1 = LL_DMA_IsActiveFlag_HT1(DMA1);
  uint32_t tempReg2 = LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_1);

  if ((tempReg1 != 0U) && (tempReg2 != 0U))
  {
    (void)R1_DMAx_HT_IRQHandler(&PWM_Handle_M1);
    LL_DMA_ClearFlag_HT1(DMA1);
  }
  else
  {
    /* Nothing to do */
  }

  if (LL_DMA_IsActiveFlag_TC1(DMA1) != 0U)
  {
    LL_DMA_ClearFlag_TC1(DMA1);
    (void)R1_DMAx_TC_IRQHandler(&PWM_Handle_M1);
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN DMA1_Channel1_IRQHandler */

  /* USER CODE END DMA1_Channel1_IRQHandler */
}

void DMA2_Channel1_IRQHandler (void)
{
  if (LL_DMA_IsActiveFlag_HT1(DMA2) && LL_DMA_IsEnabledIT_HT(DMA2, LL_DMA_CHANNEL_1))
  {
    (void)R1_DMAx_HT_IRQHandler(&PWM_Handle_M2);
    LL_DMA_ClearFlag_HT1(DMA2);
  }
  else
  {
    /* Nothing to do */
  }

  if (LL_DMA_IsActiveFlag_TC1(DMA2))
  {
    LL_DMA_ClearFlag_TC1(DMA2);
    (void)R1_DMAx_TC_IRQHandler(&PWM_Handle_M2);
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN DMA2_Channel1_IRQHandler */

  /* USER CODE END DMA2_Channel1_IRQHandler */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles second motor TIMx Update interrupt request.
  * @param  None
  */
void TIMx_UP_M2_IRQHandler(void)
{
 /* USER CODE BEGIN TIMx_UP_M2_IRQn 0 */

 /* USER CODE END  TIMx_UP_M2_IRQn 0 */

  LL_TIM_ClearFlag_UPDATE(TIM8);
  (void)R1_TIMx_UP_IRQHandler(&PWM_Handle_M2);
  TSK_DualDriveFIFOUpdate(M2);

 /* USER CODE BEGIN TIMx_UP_M2_IRQn 1 */

 /* USER CODE END  TIMx_UP_M2_IRQn 1 */
}

void TIMx_BRK_M2_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M2_IRQn 0 */

  /* USER CODE END TIMx_BRK_M2_IRQn 0 */

  if (0U == LL_TIM_IsActiveFlag_BRK(TIM8))
  {
    /* Nothing to do */
  }
  else
  {
    LL_TIM_ClearFlag_BRK(TIM8);
    PWMC_OCP_Handler(&PWM_Handle_M2._Super);

    /* USER CODE BEGIN BRK */

    /* USER CODE END BRK */
  }

  if (0U == LL_TIM_IsActiveFlag_BRK2(TIM8))
  {
    /* Nothing to do */
  }
  else
  {
    LL_TIM_ClearFlag_BRK2(TIM8);
    PWMC_DP_Handler(&PWM_Handle_M2._Super);

    /* USER CODE BEGIN BRK2 */

    /* USER CODE END BRK2 */
  }

  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here */
  MC_RunMotorControlTasks();

  /* USER CODE BEGIN TIMx_BRK_M2_IRQn 1 */

  /* USER CODE END TIMx_BRK_M2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
