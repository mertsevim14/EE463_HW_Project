
const uint8_t PWM_PIN = 9; 


const uint8_t ADC_CH0 = A0;
const uint8_t ADC_CH1 = A1;

void setup() {
  pinMode(PWM_PIN, OUTPUT);

  Serial.begin(115200);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  TCCR1A |= (1 << COM1A1);

  TCCR1B |= (1 << CS11); 

  ICR1 = 1999;   
  OCR1A = 1000; 

  ADCSRA = (ADCSRA & 0b11111000) | 0b110;
}

void setDutyPermille(uint16_t permille) {
  if (permille > 1000) permille = 1000;

  OCR1A = (uint32_t)permille * (ICR1 + 1) / 1000;
}

void loop() {

  setDutyPermille(600); 


  //float adc0 = (((analogRead(ADC_CH0))*5.0f-0.6f)/1023.0f)/0.04f;
  float adc0 = (2.5f-(((analogRead(ADC_CH0))*5.0f)/1023.0f))/0.066f;
  uint16_t adc1 = analogRead(ADC_CH1);


  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs >= 20) { 
    lastPrintMs = millis();
    Serial.print("A0=");
    Serial.print(adc0);
    Serial.print("  A1=");
    Serial.println(adc1);
  }
}
