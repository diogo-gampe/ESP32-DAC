#include <Arduino.h>

// Definições
#define DAC_CHANNEL_1 25  // GPIO25 - DAC1
#define ADC_CHANNEL_1 34 // GPIO35 - ADC1
#define CLOCK_FREQ 80.0e6 // clock de 80 MHz
// Variáveis globais
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; 

volatile int adc_value = 0;
volatile uint8_t dac_value = 0;

volatile uint32_t start_isr, end_isr;
volatile uint32_t isr_interval = 0;
volatile uint32_t last_isr_time = 0;

const uint8_t ADC_PIN = 34; // ADC1_CHANNEL_6
const uint8_t DAC_PIN = 25; // DAC1
const float h_ms = 1;      // tempo de amostragem em ms 

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

  if(dac_value == 255)
    increase = false;
  if(dac_value == 0)
    increase = true;

  if (increase) 
    dac_value++;
  else
    dac_value--;


  dacWrite(DAC_PIN, dac_value);
  int raw = analogRead(ADC_PIN); // 0-4095
  // Armazena para debug no loop()
  adc_value = raw;

  portEXIT_CRITICAL_ISR(&timerMux);

  //armazena tempo ao final da interrupção
  end_isr = ESP.getCycleCount();

}

void setup() {
  Serial.begin(9600);

  //configura LED_BUILTIN

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Configura o DAC
  pinMode(DAC_CHANNEL_1, ANALOG);
  dacWrite(DAC_CHANNEL_1, 0); // Inicia com 0V

  // Configura ADC
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);

  
  // Configura o timer para interrupção periódica
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (para ESP32 80MHz -> 1MHz)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, h_ms*1000, true);
  timerAlarmEnable(timer);
  
  Serial.println("DAC e ADC com interrução periódica inicializado");
}

void loop() {

  static uint32_t last_print = 0;
  if (millis() - last_print > 500) {
    last_print = millis();
    float vin = adc_value * 3.3 / 4095.0;
    float vout = dac_value * 3.3 / 255.0;

    float elapsed_us = (end_isr - start_isr)/80.0; //em microsegundos
    float time_isr = isr_interval/80000.0; // em milisegundos
    Serial.printf("ADC: %.2f V\tDAC: %.2f V\t Tempo dentro de ISR: %.2f us\t Tempo entre ISRs(ms): %.2f\n", 
                  vin, vout, elapsed_us, time_isr);
  }
}