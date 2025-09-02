
#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{

    float Kp;
    float Ki;
    float Ts;

    float saida_atual;
    float saida_ant; 
    float erro_atual;
    float erro_ant; 

} Controlador_PI;

void control_init(Controlador_PI *cntrl, float Kp, float Ki, float Ts);

void control_reset(Controlador_PI *cntrl);
void control_update(Controlador_PI *cntrl, float u, float y);

uint8_t control_output(Controlador_PI *cntrl, uint8_t pino);

#ifdef __cplusplus
}
#endif

#endif