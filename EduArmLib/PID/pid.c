#include "pid.h"

/* PID init */
void PID_Vitesse_Init(PID *p, float kp, float ki, float kd) {
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integrale = 0;
    p->erreur_precedente = 0;
}

/* PID calculation*/
float calculer_commande_PID(PID*pid, float consigne, float mesure, float dt) {
    float erreur = consigne - mesure;

    pid->integrale += erreur * dt;

    float max_integrale = 5000.0f;
    if (pid->integrale > max_integrale)  pid->integrale = max_integrale;
    if (pid->integrale < -max_integrale) pid->integrale = -max_integrale;

    float derivee = (erreur - pid->erreur_precedente) / dt;
    pid->erreur_precedente = erreur;

    return (pid->kp * erreur) + (pid->ki * pid->integrale) + (pid->kd * derivee);
}