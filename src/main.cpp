#include <Arduino.h>
#include <Wifi.h>
#include <math.h>
#include "control.h"

// Definições
#define DAC_CHANNEL_1 25  // GPIO25 - DAC1
#define ADC_CHANNEL_1 34 // GPIO34 - ADC1
#define CLOCK_FREQ_MHZ 240 // clock de 240 MHz
#define TABLE_SIZE 1000 //tamanho de lookup table para multisenos

// Variáveis globais
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; 

volatile uint16_t adc_value = 0;
volatile uint8_t dac_value = 0;

volatile uint32_t start_isr, end_isr;
volatile uint32_t isr_interval = 0;
volatile uint32_t last_isr_time = 0;
volatile uint32_t elapsed = 0;
volatile float elapsed_us = 0;

const uint8_t ADC_PIN = 34; // ADC1_CHANNEL_6
const uint8_t DAC_PIN = 25; // DAC1
const int h_ms = 48;      

bool increase = true;
bool flag_timer = false;

bool input = true;



Controlador_PI cntrl;


// Número de componentes senoidais

// d = 5 para identificação  e d=9 para validação
const int d = 9;  

// Frequências e fases (pré-calculadas)
float wk[d];
float phase[d];

// Amplitude do sinal
float Amultisine = 50.0f;
float u = 0; 
uint8_t multisine_table[TABLE_SIZE];

volatile unsigned long idx = 0;
uint32_t count = 0;





// Função de interrupção do timer
void IRAM_ATTR onTimer() {

  

  // Lê valor da LUT
  // dac_value = multisine_table[idx];
  // adc_value = analogRead(ADC_PIN);
  // control_update(&cntrl, 1.0f, adc_value);
  // control_output(&cntrl, DAC_PIN);
  // // Atualiza índice
  // idx++;
  // if (idx >= TABLE_SIZE) idx = 0;

  // Serial.printf("ADC: %d | DAC: %d\n", 
  //                 adc_value, dac_value);
  
  flag_timer = true;
  if(count > 500){

    input = !input;
    count = 0;
  }
  count++;
  
}

void setup() {

  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.begin(115200);

  //configura LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Configura o DAC
  pinMode(DAC_CHANNEL_1, ANALOG);
  dacWrite(DAC_CHANNEL_1, 0); // Inicia com 0V

  // Configura ADC
  analogReadResolution(12); //ADC 12 bits
  analogSetPinAttenuation(ADC_PIN, ADC_11db);//atenuação 11db faz range de 150 mV ~ 2500 mV

  
  //  // Definição das frequências e fases
  // float wmin = 2.0f * M_PI * 0.01f;   // 0.01 Hz
  // float wmax = 2.0f * M_PI * 1.0f;  // 1 Hz
  // for (int k = 0; k < d; k++) {
  //   wk[k] = (wmin + k * (wmax - wmin) / (d - 1)) * (h_ms/1000.0f);  // frequência normalizada
  //   phase[k] = - (k * (k - 1) * M_PI / d);                // fase inicial
  // }


  // for (int n = 0; n < TABLE_SIZE; n++) {
  //   float soma = 0.0f;
  //   for (int k = 0; k < d; k++) {
  //     soma += cosf(wk[k] * n + phase[k]);
  //   }
  //   float u = Amultisine * soma;

  //   multisine_table[n] = (uint8_t)((u + 250) / 5.0f);
  // }


  // Configura o timer para interrupção periódica
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (para clock do timer0 80Mhz -> 1MHz)
  timerAttachInterrupt(timer, &onTimer, true); //associa interrupção ao timer
  timerAlarmWrite(timer, h_ms*1000, true);//associa frequencia de interrupção h_ms*1000 = n de ticks para interromper
  timerAlarmEnable(timer); // ativa timer


  control_init(&cntrl, 5, 12, 0.0475);
  control_reset(&cntrl);

  Serial.println("DAC e ADC com interrução periódica inicializado");
}

void loop() {

 if (flag_timer) {
        flag_timer = false;  // limpa flag
        portENTER_CRITICAL_ISR(&timerMux);

        // agora pode chamar analogRead e o PI sem travar
        int adc_value = analogRead(ADC_PIN);

        control_update(&cntrl, (3.3f)*input, (float)adc_value*(3.3/4095));
        dac_value = control_output(&cntrl, DAC_CHANNEL_1);

        Serial.printf("ADC: %d | DAC: %d\n", adc_value, dac_value);
        
       
        portEXIT_CRITICAL_ISR(&timerMux);

        // aqui pode até usar Serial
        // Serial.printf("ADC: %d\n", adc_value);
    }
}