#ifndef PID_H
#define PID_H


typedef struct {
    float kp, ki, kd;
    float integrale;
    float erreur_precedente;
} PID;

void PID_Vitesse_Init(PID *p, float kp, float ki, float kd);
float calculer_commande_PID(PID*pid, float consigne, float mesure, float dt);

#endif