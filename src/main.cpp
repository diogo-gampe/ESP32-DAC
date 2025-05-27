#include <Arduino.h>

// Definições
#define DAC_CHANNEL_1 25  // GPIO25 - DAC1
#define DAC_CHANNEL_2 26  // GPIO26 - DAC2

// Variáveis globais
volatile uint8_t dac_value = 0;
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; 

const float interrupt_freq = 100; // 10Hz
const uint8_t resolution = 8;      // 8 bits para DAC
bool increase = false; 

// Função de interrupção do timer
void IRAM_ATTR onTimer() {
  // Desativa temporariamente as interrupções para evitar conflitos
  portENTER_CRITICAL_ISR(&timerMux);
  
  // Lê o valor atual do DAC1 (se estiver configurado como entrada)
  // Nota: Normalmente o DAC é apenas saída, esta parte é para ilustração
  // dac_value = dacRead(DAC_CHANNEL_1);
  
  // Processa o valor (aqui apenas incrementa como exemplo)
  if (dac_value == 255)
    increase = false; 
  else if(dac_value == 0)
    increase = true; 
    
  if(increase)
    dac_value++; 
  else
    dac_value--; 
  
  // Escreve no DAC1
  dacWrite(DAC_CHANNEL_1, dac_value);
  
  // Reativa as interrupções
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(9600);
  
  // Configura o DAC
  pinMode(DAC_CHANNEL_1, ANALOG);
  dacWrite(DAC_CHANNEL_1, 0); // Inicia com 0V

  
  // Configura o timer para interrupção periódica
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (para ESP32 80MHz -> 1MHz)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000 / interrupt_freq, true); 
  timerAlarmEnable(timer);
  
  Serial.println("DAC com interrupção periódica inicializado");
}

void loop() {
  // O processamento principal está sendo feito na interrupção
  // Aqui podemos fazer outras tarefas ou monitorar via serial
  static uint32_t last_print = 0;
  if (millis() - last_print > 1000) {
    last_print = millis();
    Serial.printf("Valor atual do DAC: %d\n", dac_value);
  }
  
  // Pequeno delay para não sobrecarregar a serial
  delay(10);
}