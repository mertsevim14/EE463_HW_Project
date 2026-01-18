#include <Arduino.h>

const uint8_t PWM_PIN   = 9; 
const uint8_t ADC_I_PIN = A0; 

-
const float FS_CTRL = 1000.0f;              
const float TS      = 1.0f / FS_CTRL;

const uint32_t FPWM = 1000;                 


const float VREF    = 5.0f;
const float ADC_LSB = VREF / 1023.0f;


static constexpr float ACS712_SENS_V_PER_A = 0.066f; 


volatile float I_OFFSET_V = 2.60f; 

static constexpr uint8_t I_ADC_SAMPLES = 64; 


static constexpr float I_FILT_ALPHA = 0.90f;
static float i_filt = 0.0f;

float VBUS_NOM = 250.0f;        


const float I_MAX    = 10.0f;   
const float DUTY_MIN = 0.00f;
const float DUTY_MAX = 1.0f;

volatile float i_ref_target = 8.0f;  
volatile float i_ref        = 0.0f;  


static constexpr uint32_t SOFTSTART_MS = 15000UL; 
static uint32_t softstart_t0_ms = 0;


struct PIController {
  float Kp = 0;
  float Ki = 0;         
  float integrator = 0;
  float out_min = 0;
  float out_max = 0;

  float update(float err, float Ts) {
    float u_p = Kp * err;
    float integ_new = integrator + (Ki * Ts) * err;
    float u = u_p + integ_new;

    if (u > out_max) {
      u = out_max;
      if (err < 0) integrator = integ_new;
    } else if (u < out_min) {
      u = out_min;
      if (err > 0) integrator = integ_new;
    } else {
      integrator = integ_new;
    }
    return u;
  }
};
PIController piI; 

void pwmSetDuty(float duty) {
  if (duty < DUTY_MIN) duty = DUTY_MIN;
  if (duty > DUTY_MAX) duty = DUTY_MAX;

  uint16_t top = ICR1;
  uint16_t ocr = (uint16_t)(duty * (top + 1));
  if (ocr > top) ocr = top;
  OCR1A = ocr;
}

static inline float readAdcVoltageAveraged(uint8_t pin, uint8_t samples) {
  uint32_t sum = 0;
  for (uint8_t k = 0; k < samples; k++) {
    sum += (uint16_t)analogRead(pin);
  }
  float adc_avg = (float)sum / (float)samples;
  return adc_avg * ADC_LSB;
}

void calibrateCurrentOffset() {
 
  const int N = 1000;
  float sumV = 0;

  // Throw away a few reads after enabling ADC
  for (int k = 0; k < 100; k++) (void)analogRead(ADC_I_PIN);

  for (int k = 0; k < N; k++) {
    sumV += readAdcVoltageAveraged(ADC_I_PIN, I_ADC_SAMPLES);
    delay(2);
  }
  I_OFFSET_V = sumV / N;
}

float readArmatureCurrent() {
  float v_out = readAdcVoltageAveraged(ADC_I_PIN, I_ADC_SAMPLES);

  float i_raw = ((I_OFFSET_V - v_out) / ACS712_SENS_V_PER_A);

  i_filt = (I_FILT_ALPHA * i_filt) + ((1.0f - I_FILT_ALPHA) * i_raw);

  return i_filt;
}

void setupPwmTimer1_1kHz() {
  pinMode(PWM_PIN, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  TCCR1A |= (1 << COM1A1);

  TCCR1B |= (1 << CS10);

  uint16_t top = (uint16_t)(F_CPU / (FPWM * 1UL) - 1UL);
  ICR1 = top;

  OCR1A = 0;
}

static inline float softStartRamp(float target) {
  uint32_t elapsed = millis() - softstart_t0_ms;
  if (elapsed >= SOFTSTART_MS) return target;
  return target * ((float)elapsed / (float)SOFTSTART_MS);
}

void controlStep() {
  float i_meas = readArmatureCurrent();
  float v_sensor = readAdcVoltageAveraged(ADC_I_PIN, I_ADC_SAMPLES);

  float i_ref_cmd = softStartRamp(i_ref_target);
  i_ref = i_ref_cmd;

  float i_ref_limited = i_ref_cmd;
  if (i_ref_limited < 0) i_ref_limited = 0;
  if (i_ref_limited > I_MAX) i_ref_limited = I_MAX;

  float e_i = i_ref_limited - i_meas;
  float v_cmd = piI.update(e_i, TS);

  float duty = v_cmd / VBUS_NOM;

  float duty_cap = 0.60f * softStartRamp(1.0f);
  if (duty > duty_cap) duty = duty_cap;

  pwmSetDuty(duty);

  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs >= 20) {
    lastPrintMs = millis();
    Serial.print("Voff=");   Serial.print(I_OFFSET_V, 3);
    Serial.print(" Iref=");  Serial.print(i_ref_limited, 2);
    Serial.print(" I=");     Serial.print(i_meas, 3);
    Serial.print(" Vcmd=");  Serial.print(v_cmd, 1);
    Serial.print(" duty=");  Serial.print((float)OCR1A / (ICR1 + 1), 3);
    Serial.print(" Vsens="); Serial.println(v_sensor, 4);
  }
}

void setup() {
  Serial.begin(115200);

  analogReference(DEFAULT); // UNO: 5V reference (same as Vcc)

  // Optional: speed up ADC a bit (trade accuracy vs speed)
  // ADCSRA prescaler bits: 0b101 => /32 => 500 kHz ADC clock (faster, a bit noisier)
  ADCSRA = (ADCSRA & 0b11111000) | 0b101;

  setupPwmTimer1_1kHz();

  // Ensure motor is OFF for offset calibration
  pwmSetDuty(0.0f);
  delay(300);

  // IMPORTANT: If you can guarantee motor is OFF at boot, enable this:
  // calibrateCurrentOffset();

  i_filt = 0.0f;              // reset filter

  // ---- PI tuning ----
  piI.Kp = 2.0f;
  piI.Ki = 150.0f;
  piI.out_min = 0.0f;
  piI.out_max = VBUS_NOM * DUTY_MAX;

  // Soft-start init
  softstart_t0_ms = millis();
  i_ref = 0.0f;

  pwmSetDuty(0.0f);
}

void loop() {
  static uint32_t last_us = 0;
  uint32_t now = micros();
  const uint32_t period_us = (uint32_t)(1e6f / FS_CTRL);

  if ((uint32_t)(now - last_us) >= period_us) {
    last_us += period_us;
    controlStep();
  }
}
