#pragma once
// I'm not smart enough to figure this out by myself.  ChatGPT is my friend,  Ray Jorgensen  11/7/2024

#include <CytronMotorDriver.h>
#include <PID_v1.h>
#include <math.h>
#include <elapsedMillis.h>
#include <RunningAverage.h>
#include <vector>


struct PinState {
  uint8_t pinNumber;  // physical GPIO pin to drive (1..15,18..26)
  uint8_t fn;         // function id (1=S01,2=S02,...). 0/255 = unassigned
  bool state;         // ON/OFF from machine.state.functions[fn]
};

std::vector<PinState> pinStates;

#define LOW 0

void addPinState(uint8_t pinNum, bool isPinOn) {
  // Add pin state to the vector
  pinStates.push_back({ pinNum, isPinOn });
}

const uint8_t maxNozzles = 24;           // Maximum number of nozzles plus one extra for safety
uint32_t PwmTimer[maxNozzles] = { 0 };   // Array for each pin's PWM timer
uint32_t OnTime[maxNozzles] = { 0 };     // Array for each pin's on time
bool nozzleState[maxNozzles] = { LOW };  // Track ON/OFF state per nozzle
// Global variable for EvenOdd timing
unsigned long lastToggleTime = 0;


// Configure the motor driver.
//CytronMD motor(PWM_DIR, 29, 28);  // PWM = Pin 29 White, DIR = Pin 28 Red,  Ball Valve
CytronMD motor(PWM_DIR, 37, 38);  // PWM = Pin 37 White, DIR = Pin 38 Red,  Ball Valve
int lastMotorCmd = 0;

// Pin Definitions
#define flowSensorPin 0  // 0
#define PressurePin 41   // A17

// Comm
IPAddress csharpIP;
uint16_t csharpPort = 0;

// User Variables
struct UserSettings {
  float GPATarget = 8.8;
  float SprayWidth = 36.0;
  float FlowCalibration = 405.84645825f;
  float PSICalibration = 0.054210;
  float DutyCycleAdjustment = 10;
  float PressureTarget = 15.0;
  byte numberNozzles = 1;   // not in use
  float currentDutyCycle = 50.0;
  unsigned int Hz = 10;
  byte LowBallValve = 8;
  byte Ball_Hyd = 0;  //not used
  byte WheelAngle = 5;
  double Kp = 2.5;
  double Ki = 0.12;
  double Kd = 0.00;
  char unit[9] = "Imperial";
  uint8_t debugPwmLevel = 0;
  uint8_t PWM_Conventional = 0;
  uint8_t Stagger = 0;
  // ---- Kick & Hold user settings no longer used----
  unsigned long KH_kickDurationMs = 4;     // 2..4 ms typical
  float KH_holdDutyCycle = 0.3f;           // 0.02..0.10 typical
  unsigned long KH_holdPWMFrequency = 70;  // 5..200 Hz
  float KH_holdRefV = 12.6f;
};

UserSettings userSettings;  // Instantiate the struct to hold settings

// int16_t eeAddr = -1;  // -1 defaults to no EEPROM saving/loading from machine.h
// void init(int16_t _eeAddr = -1, const uint8_t _eeSize = 33)  // 33 bytes of EEPROM used from machine.h
const int USER_SETTINGS_ADDR = 400;  // Start `UserSettings` EEPROM
const int EEPROM_MAGIC_ADDR = USER_SETTINGS_ADDR + sizeof(UserSettings);
const uint8_t EEPROM_MAGIC_VALUE = 42;

// PID
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, userSettings.Kp, userSettings.Ki, userSettings.Kd, DIRECT);

// EMA settings (pressure)
float filteredADC = 0;  // Holds the smoothed ADC value
float alpha = 0.4;      // Smoothing factor (0.05–0.2 is typical)

byte currentPin = 0;

// Variables for NozzleSpeed
float boomLength = 0;  // Spray width of total boom
float nozzleSpeedMph = 0;
bool TurnComp = false;

// Software switches
bool sprayerOn = false;     // Track the state of the sprayer
byte PWM_Conventional = 0;  // Select PWN or conventional On/Off
byte Stagger = 0;           //Select even/odd staggered firing
byte Start = 0;             // Used in calibrate

// Global variables for startup handling
bool isStartup = true;          // Flag to indicate if the system is in the startup phase
elapsedMillis startupTimer;     // Timer to track the startup delay
float defaultDutyCycle = 50.0;  // Default duty cycle at startup

// Flow variables
volatile uint32_t pulseCount = 0;
volatile uint32_t pulseCountTotal = 0;
float flowRate = 0.0;
float GPM = 0.0;
double actualGPA = 7.0;
double gpaDisplay = 0;
float newDutyCycle = 50.0;
double actualGPAave = 7.0;
uint8_t numActiveNozzles = 0;
uint8_t debugPwmLevel = 0;
float& PSSL = userSettings.FlowCalibration;  // pulses per liter (your calibration)
const float maxStep = 2.5f;
// ---- Acres tracking ----
float acresTotal = 0.0f;      // total acres sprayed this run
float acresPerHour = 0.0f;    // instantaneous rate for display


// Pressure Variables
const float minPressure = 0.0;
const float maxPressure = 100.0;
double pressure = 20;
float voltage = 0;

// state
bool pressureControlActive = false;
int settleCounter = 0;

// “done” criteria
const float PRESSURE_TOL = 1.5f;   // psi band
const int   SETTLE_COUNT = 5;     // 10 * 100ms = 1s stable
const float TARGET_EPS   = 0.01f;  // psi: ignore tiny target changes

// From AOG
float steerAngle = 4;
double gpsSpeed = 4.0;
bool isPinOn = false;
uint8_t pinNum = 0;

// Variables for timing
uint32_t period = 0;  // Total period of the cycle in milliseconds
double onTime = 50;   // On-time in milliseconds
uint32_t PwmPhaseOffset[maxNozzles] = { 0 };

// Running average objects
RunningAverage pulseAvg(16);  // Running average for sensor value (6-sample buffer)
RunningAverage GPA_Avg(16);   // Running average for voltage (6-sample buffer)

// Timer variables
elapsedMillis Flow_Timer;
elapsedMillis printTimer;
float PrintFrequency = 500;
elapsedMillis read_pressure;
elapsedMillis pwmTimer;             // Timer to control the PWM cycle
elapsedMillis timeSinceLastToggle;  // Automatically increments with elapsed time
elapsedMillis pwmCycleTimer;        // Track time for PWM cycle
// How often to sample the flow sensor (ms).
// Bigger = more pulses per sample = smoother, but slower response.
const uint16_t FLOW_SAMPLE_MS = 1000;   // try 500ms to start (0.5s)

// Interrupt Service Routine (ISR) for counting pulses
void pulseCounter() {
  pulseCount++;
  pulseCountTotal++;  // for debugging, never reset
}

//       Function name          Desciption
// 0 - debug OFF           - Turns OFF all reporting
// 1 - dutycycleTurncomp   - In a turn individual nozzle reporting
// 2 - setPWMTiming        - Sets the timing of the nozzle on cycle
// 3 - ControlNozzle       - Like it says, nozzle control - not used in a turn
// 4 - Pressure            - System pressure
// 5 - EvenOdd             - Toggle firing of even/odd nozzles
// 6 - Flow                - The heart of the system - flow rate
// 7 - NozzleSpeed         - Individual nozzle speed in a turn
// 8 - PrintDebug          - Over all system reporting
// 9 - PrintAOG            - Report variable passed from AOG
// 10 - Calibrate_PSI_Flow - Calibration function
// 11 - ActualSteerAngle   - extractActualSteerAngle
// 12 - Communication      - Communication between apps

// ===== Kick & Hold (K&H) Tunables =====
float KH_hzRate = 15.0f;     // informational
float KH_dutyCycle = 0.50f;  // informational

bool KH_holdCompEnable = true;
float KH_holdDutyMin = 0.01f;
float KH_holdDutyMax = 0.30f;
float KH_lastBusV = 0.0f;
float KH_lastCyclePeak_mA = 0.0f;
