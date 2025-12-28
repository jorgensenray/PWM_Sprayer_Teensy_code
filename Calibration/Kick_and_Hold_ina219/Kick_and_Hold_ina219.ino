#include <Wire.h>
#include <Adafruit_INA219.h>
#include <elapsedMillis.h>

// ===================== INA219 (I2C) =====================
// Default address 0x40. If you changed the ADDR pin, pass the new address.
Adafruit_INA219 ina219(0x40);

// ===================== Pins =====================
// Solenoid drive pin (unchanged)
const int SOLENOID_PIN = 2;

// ===================== User settings =====================
float hzRate = 10.0;
float dutyCycle = 0.5;
unsigned long kickDurationMs = 4;       // Kick duration (ms)  - 2..4 typical
float holdDutyCycle = 0.04;             // Base hold duty (0.0–1.0) - 0.02..0.04
unsigned long holdPWMFrequency = 500;   // Hz - 200..1000 typical

// ===================== Timing =====================
elapsedMillis cycleTimer;
elapsedMicros pwmTimer;   // from elapsedMillis.h
bool solenoidState = false;
bool holdPWMState = false;
unsigned long cyclePeriodMs = 0;
unsigned long onTimeMs = 0;

// ===================== Telemetry =====================
float peakCurrent_mA_cycle  = 0.0f;
float lastCyclePeak_mA      = 0.0f;

// ===================== Voltage tracking (from INA219) =====================
float lastBusV = 0.0f;

// ===================== Voltage-comp hold =====================
bool  holdCompEnable = true;  // enable/disable compensation
float holdRefV       = 12.6;  // reference voltage
float holdDutyMin    = 0.01;  // clamp range
float holdDutyMax    = 0.30;

// ===================== Prototypes =====================
static inline float clampf(float x, float lo, float hi);
float readINA219_mA();
float readBusVoltage();
float getEffectiveHoldDuty();
void  updateTiming();

// ===================== Helpers =====================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float readINA219_mA() {
  // Returns current in mA using INA219's internal math based on calibration.
  return ina219.getCurrent_mA();
}

float readBusVoltage() {
  // INA219 measures bus voltage directly (up to ~26V)
  return ina219.getBusVoltage_V();
}

float getEffectiveHoldDuty() {
  float v = lastBusV;
  bool bad = (v < 6.0f || v > 28.0f); // widened a bit to avoid false "bad"
  if (bad || !holdCompEnable) {
    return clampf(holdDutyCycle, holdDutyMin, holdDutyMax);
  }
  // Scale base hold to keep magnetic force roughly consistent vs. bus voltage
  float scaled = holdDutyCycle * (holdRefV / v);
  return clampf(scaled, holdDutyMin, holdDutyMax);
}

void updateTiming() {
  cyclePeriodMs = (unsigned long)(1000.0 / hzRate);
  if (cyclePeriodMs < 1) cyclePeriodMs = 1;
  onTimeMs = (unsigned long)(cyclePeriodMs * dutyCycle);
  if (kickDurationMs > onTimeMs) kickDurationMs = onTimeMs;  // clamp
}

// ===================== Arduino Setup/Loop =====================
void setup() {
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  Serial.begin(115200);
  Serial.setTimeout(30);

  // I2C + INA219 init
  Wire.begin();
  if (!ina219.begin()) {
    Serial.println(F("INA219 not found. Check wiring/address!"));
    while (1) delay(10);
  }

  // Choose a calibration that matches your shunt & range.
  // Common Adafruit breakout default is ~0.1Ω shunt and "32V, 2A" cal:
  ina219.setCalibration_32V_2A();

  // Other options (uncomment ONE if needed):
  // ina219.setCalibration_32V_1A();
  // ina219.setCalibration_16V_400mA();

  // Cold-start dip check using INA219 bus voltage
  float coldStartVoltage = 100.0f;
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    float v = readBusVoltage();
    if (v < coldStartVoltage) coldStartVoltage = v;
    delay(5);
  }
  Serial.print(F(">>> Cold Start Min Voltage (V): "));
  Serial.println(coldStartVoltage, 2);

  updateTiming();

  Serial.println(F("K&H PWM + INA219 (I2C)"));
  Serial.print(F("Hz="));   Serial.print(hzRate,2);
  Serial.print(F(" Duty=")); Serial.print(dutyCycle,2);
  Serial.print(F(" Kick(ms)=")); Serial.print(kickDurationMs);
  Serial.print(F(" Hold=")); Serial.print(holdDutyCycle,3);
  Serial.print(F(" PWM(Hz)=")); Serial.print(holdPWMFrequency);
  Serial.print(F(" Comp=")); Serial.print(holdCompEnable ? "ON" : "OFF");
  Serial.print(F(" RefV=")); Serial.println(holdRefV,2);
}

void loop() {
  // ---- Solenoid state machine ----
  if (solenoidState) {
    unsigned long elapsed = cycleTimer;

    if (elapsed >= onTimeMs) {
      // turn off and tally
      digitalWrite(SOLENOID_PIN, LOW);
      solenoidState = false;

      lastCyclePeak_mA = peakCurrent_mA_cycle;

      Serial.print(F("ON-pulse peak (mA): "));
      Serial.print(lastCyclePeak_mA, 1);
      Serial.print(F("  BusV: "));
      Serial.print(lastBusV, 2);
      Serial.print(F("  EffHold: "));
      Serial.println(getEffectiveHoldDuty(), 3);

    } else if (elapsed < kickDurationMs) {
      // Kick: full-on
      digitalWrite(SOLENOID_PIN, HIGH);

    } else {
      // Hold: software PWM on any pin
      float effHold = getEffectiveHoldDuty();
      unsigned long pwmPeriod = 1000000UL / (holdPWMFrequency < 20 ? 20 : holdPWMFrequency);
      unsigned long on_us     = (unsigned long)(pwmPeriod * effHold);

      if (pwmTimer < on_us) {
        if (!holdPWMState) { digitalWrite(SOLENOID_PIN, HIGH); holdPWMState = true; }
      } else if (pwmTimer < pwmPeriod) {
        if (holdPWMState)   { digitalWrite(SOLENOID_PIN, LOW);  holdPWMState = false; }
      } else {
        pwmTimer = 0;
      }
    }

  } else {
    // start of next cycle
    if (cycleTimer >= cyclePeriodMs) {
      cycleTimer = 0;
      pwmTimer   = 0;
      solenoidState = true;
      peakCurrent_mA_cycle = 0.0f;
    }
  }

  // ---- live measurements ----
  float current_mA = readINA219_mA();
  lastBusV = readBusVoltage();

  if (solenoidState && current_mA > peakCurrent_mA_cycle) {
    peakCurrent_mA_cycle = current_mA;
  }
}
