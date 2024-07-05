/**
  ******************************************************************************
  * @file    pmsm_motor_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure the motor to drive.
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
#ifndef PMSM_MOTOR_PARAMETERS_H
#define PMSM_MOTOR_PARAMETERS_H

/**************************
 *** Motor 1 Parameters ***
 **************************/

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM           15 /* Number of motor pole pairs */
#define RS                      0.64 /* Stator resistance , ohm*/
#define LS                      0.00154 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define MOTOR_MAX_SPEED_RPM     205 /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT  133.0 /*!< Volts RMS ph-ph /kRPM */
#define NOMINAL_CURRENT_A       17

#define ID_DEMAG_A              -17 /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        299 /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR          400  /*!< Number of pulses per
                                            revolution */

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM2          15 /* Number of motor pole pairs */
#define RS2                     0.64 /* Stator resistance , ohm*/
#define LS2                     0.00154 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_CURRENT2_A       17
#define MOTOR_MAX_SPEED_RPM2     205 /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT2  133.0 /*!< Volts RMS ph-ph /kRPM */
#define ID_DEMAG2_A              -17 /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT2  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT2        299 /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M2_ENCODER_PPR           400  /*!< Number of pulses per
                                            revolution */
#endif /* PMSM_MOTOR_PARAMETERS_H */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
