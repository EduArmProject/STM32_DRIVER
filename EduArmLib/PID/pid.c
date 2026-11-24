/**
  ******************************************************************************
  * @file    pid.c
  * @author  Samuel_D
  * @brief   This file contains all the basic functions for PID
  ******************************************************************************
  * @attention
  *
  * Open-source EduArm
  *
  ******************************************************************************
  */

#include "pid.h"

/**
  * @brief  Initializes a PID controller.
  * @param  p PID controller instance.
  * @param  kp Proportional coefficient.
  * @param  ki Integral coefficient.
  * @param  kd Derivative coefficient.
  * @retval None.
  */
void PID_Vitesse_Init(PID *p, float kp, float ki, float kd) {
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
  p->integrale = 0;
  p->erreur_precedente = 0;
}

/**
  * @brief  Calculates the PID command.
  * @param  pid PID controller instance.
  * @param  consigne Target value.
  * @param  mesure Measured value.
  * @param  dt Sampling period.
  * @retval PID command.
  */
float PID_Calculer_Commande(PID*pid, float consigne, float mesure, float dt) {
  float erreur = consigne - mesure;

  pid->integrale += erreur * dt;

  float max_integrale = 5000.0;
  if (pid->integrale > max_integrale)  pid->integrale = max_integrale;
  if (pid->integrale < -max_integrale) pid->integrale = -max_integrale;

  float derivee = (erreur - pid->erreur_precedente) / dt;
  pid->erreur_precedente = erreur;

  return (pid->kp * erreur) + (pid->ki * pid->integrale) + (pid->kd * derivee);
}