const uint8_t NUM_ADC = 10;
const uint16_t UMBRAL_CAMBIO = 50;
const float ALPHA_EMA = 0.001f;

volatile uint16_t adcValues[NUM_ADC];
volatile uint8_t currentChannel = 0;
volatile bool discardSample = true;

float adc_Values_mean[NUM_ADC];
bool sensores_piso[NUM_ADC] = {false};


ISR(ADC_vect) {
  uint16_t value = ADC; 
  
  if (discardSample) { 
    discardSample = false;
    return;
  }
  
  adcValues[currentChannel] = value;
  
  currentChannel++;                  
  if (currentChannel >= NUM_ADC) {
    currentChannel = 0;            
  }
  
  ADMUX = (1 << REFS0) | (currentChannel & 0x07);
  ADCSRB = (currentChannel & 0x08) ? (1 << MUX5) : 0;
  discardSample = true; 
}


void setup() {
  Serial.begin(115200); 
  while (!Serial);
  
  // ADC ----------------------
  ADMUX = (1 << REFS0);
  ADCSRB = 0;           
  ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  
  sei();
  ADCSRA |= (1 << ADSC);

  delay(20);
  noInterrupts();
  for (uint8_t i = 0; i < NUM_ADC; i++) {
    adc_Values_mean[i] = adcValues[i]; 
  }
  interrupts();
  // --------------------------------------
}


void loop() {
  VoB();
}


void VoB() {
  uint16_t copias_adc[NUM_ADC];

  noInterrupts();
  for (uint8_t i = 0; i < NUM_ADC; i++) {
    copias_adc[i] = adcValues[i];
  }
  interrupts(); 
  
  for (uint8_t i = 0; i < NUM_ADC; i++) {
    float cambio = fabs(adc_Values_mean[i] - (float)copias_adc[i]);

    if (cambio > UMBRAL_CAMBIO) { 
      sensores_piso[i] = true;
    } else { 
      sensores_piso[i] = false;
      adc_Values_mean[i] = adc_Values_mean[i] + ALPHA_EMA * ((float)copias_adc[i] - adc_Values_mean[i]);
    }
  }

  for (uint8_t i = 0; i < NUM_ADC; i++) {
    Serial.print(copias_adc[i]); 
    Serial.print("\t");
  }
  Serial.println();
}
