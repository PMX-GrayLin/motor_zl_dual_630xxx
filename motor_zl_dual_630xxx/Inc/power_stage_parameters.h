
/**
  ******************************************************************************
  * @file    power_stage_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a power stage.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef POWER_STAGE_PARAMETERS_H
#define POWER_STAGE_PARAMETERS_H

/**************************
 *** Motor 1 Parameters ***
 **************************/

/************* PWM Driving signals section **************/
#define HW_DEAD_TIME_NS                      500 /*!< Dead-time inserted
                                                      by HW if low side signals
                                                      are not used */
/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR             0.039968025579536375 /*!< It expresses how
                                                      much the Vbus is attenuated
                                                      before being converted into
                                                      digital value */
#define NOMINAL_BUS_VOLTAGE_V                24
/******** Current reading parameters section ******/
/*** Topology ***/
#define SINGLE_SHUNT_PHASE_SHIFT

#define RSHUNT                               0.01

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN                   7

/*** Noise parameters ***/
#define TNOISE_NS                            500
#define TRISE_NS                             310
#define MAX_TNTR_NS                          TNOISE_NS

/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                                 1.75 /*!< in Volts */
#define T0_C                                 50 /*!< in Celsius degrees */
#define dV_dT                                0.025 /*!< V/Celsius degrees */
#define T_MAX                                100 /*!< Sensor measured
                                                     temperature at maximum
                                                     power stage working
                                                     temperature, Celsius degrees */

/**************************
 *** Motor 2 Parameters ***
 **************************/

#define HW_DEAD_TIME_NS2                     500 /*!< Dead-time inserted
                                                         by HW if low side signals
                                                         are not used */

/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR2            0.039968025579536375 /*!< It expresses how
                                                       much the Vbus is attenuated
                                                       before being converted into
                                                       digital value */
#define NOMINAL_BUS_VOLTAGE_V2               24
/******** Current reading parameters section ******/
/*** Topology ***/
#define SINGLE_SHUNT_PHASE_SHIFT

#define RSHUNT2                              0.01

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN2                  7

/*** Noise parameters ***/
#define TNOISE_NS2                           500
#define TRISE_NS2                            310
#define MAX_TNTR_NS2                         TNOISE_NS2

/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V2                                1.75 /*!< in Volts */
#define T0_C2                                50 /*!< in Celsius degrees */
#define dV_dT2                               0.025 /*!< V/Celsius degrees */
#define T_MAX2                               100 /*!< Sensor measured
                                                       temperature at maximum
                                                       power stage working
                                                       temperature, Celsius degrees */

#endif /*POWER_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
