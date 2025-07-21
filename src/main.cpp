#include <Arduino.h>
#include <Wifi.h>

// Definições
#define DAC_CHANNEL_1 25  // GPIO25 - DAC1
#define ADC_CHANNEL_1 34 // GPIO35 - ADC1
#define CLOCK_FREQ_MHZ 240 // clock de 240 MHz
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
const int h_ms = 1;      // tempo de amostragem em ms 

bool increase = true;


// Função de interrupção do timer
void IRAM_ATTR onTimer() {


  portENTER_CRITICAL_ISR(&timerMux);

  //armazena número de ciclos de CPU no ínicio do programa para calcular frequencia da interrupção
  // e armazenar duração de tempo dentro da interrupção
  start_isr = ESP.getCycleCount();

  //computa tempo gasto entre interrupções em numero de ciclos
  isr_interval = start_isr - last_isr_time;

  //atualiza variável de tempo
  last_isr_time = start_isr;


  //processa valor digital a ser convertido em analogico
  if(dac_value == 255)
    increase = false;
  if(dac_value == 0)
    increase = true;

  if (increase) 
    dac_value++;
  else
    dac_value--;


  dacWrite(DAC_PIN, dac_value);
  adc_value = analogRead(ADC_PIN); // 0-4095

  //armazena tempo ao final da interrupção
  end_isr = ESP.getCycleCount();
  elapsed = end_isr - start_isr; 
  portEXIT_CRITICAL_ISR(&timerMux);

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

  
  // Configura o timer para interrupção periódica
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (para clock do timer0 80Mhz -> 1MHz)
  timerAttachInterrupt(timer, &onTimer, true); //associa interrupção ao timer
  timerAlarmWrite(timer, h_ms*1000, true);//associa frequencia de interrupção h_ms*1000 = n de ticks para interromper
  timerAlarmEnable(timer); // ativa timer
  
  Serial.println("DAC e ADC com interrução periódica inicializado");
}

void loop() {

  static uint32_t last_print = 0;
  if (millis() - last_print > 500) {
    last_print = millis();
    

    elapsed_us = (float) elapsed/(CLOCK_FREQ_MHZ); // em microsegundos
    float time_isr = (float) isr_interval/(CLOCK_FREQ_MHZ*1000); // em milisegundos
    Serial.printf("ADC: %d \t DAC: %d \t Tempo dentro de ISR: %.2f us\t Tempo entre ISRs(ms): %.2f\n", 
                  adc_value, dac_value, elapsed_us, time_isr);
  }
}