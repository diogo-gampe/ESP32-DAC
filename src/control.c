#include <Arduino.h>
#include "control.h"



void control_init(Controlador_PI *cntrl, float Kp, float Ki, float Ts){


    cntrl->Kp = Kp;
    cntrl->Ki = Ki;
    cntrl->Ts = Ts;
    control_reset(cntrl);

}

void control_reset(Controlador_PI *cntrl){


    cntrl->saida_atual = 0.0f;
    cntrl->saida_ant = 0.0f;
    cntrl->erro_atual = 0.0f;
    cntrl->erro_ant = 0.0f;

}

void control_update(Controlador_PI *cntrl, float u, float y){

    cntrl->erro_atual = u - y;
    cntrl->saida_atual = cntrl->saida_ant + (cntrl->Kp)*(cntrl->erro_atual) + ((-cntrl->Kp) + (cntrl->Ki*cntrl->Ts))*cntrl->erro_ant;

    cntrl->erro_ant = cntrl->erro_atual;
    cntrl->saida_ant = cntrl->saida_atual; 

}

uint8_t control_output(Controlador_PI *cntrl, uint8_t pino){

    
    dacWrite(pino, (uint8_t)cntrl->saida_atual);

    return (uint8_t) cntrl->saida_atual;
}