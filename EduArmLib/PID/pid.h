/**
  ******************************************************************************
  * @file    pid.h
  * @author  Samuel_D
  * @brief   This file contains all the functions prototypes for PID
  ******************************************************************************
  * @attention
  *
  * Open-source EduArm
  *
  ******************************************************************************
  */


#ifndef PID_H
#define PID_H

/**
  * @brief PID controller instance.
  */
typedef struct {
    float kp;                   /**< Proportional coefficient. */
    float ki;                   /**< Integral coefficient. */
    float kd;                   /**< Derivative coefficient. */
    float integrale;            /**< Accumulated integral error. */
    float erreur_precedente;    /**< Error measured during the previous calculation. */
} PID;

/**
  * @brief  Initializes a PID controller.
  * @param  p PID controller instance.
  * @param  kp Proportional coefficient.
  * @param  ki Integral coefficient.
  * @param  kd Derivative coefficient.
  * @retval None.
  */
void PID_Vitesse_Init(PID *p, float kp, float ki, float kd);

/**
  * @brief  Calculates the PID command.
  * @param  pid PID controller instance.
  * @param  consigne Target value.
  * @param  mesure Measured value.
  * @param  dt Sampling period in seconds.
  * @retval Calculated PID command.
  */
float calculer_commande_PID(PID *pid, float consigne, float mesure, float dt);

#endif