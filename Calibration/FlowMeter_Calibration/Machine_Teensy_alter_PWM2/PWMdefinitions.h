// I'm not smart enough to figure this out by myself.  ChatGPT is my friend,  Ray Jorgensen  11/7/2024

#include <CytronMotorDriver.h>
#include <PID_v1.h>
#include <math.h>
#include <elapsedMillis.h>
#include <RunningAverage.h>

uint8_t debugPwmLevel = 0;  // select the # that corresponds to the function to print info
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
// 12 - PrintUserVariables - Debug UDP

// Configure the motor driver.
CytronMD motor(PWM_DIR, 29, 30);  // PWM = Pin 29, DIR = Pin 30.  //Ball Valve

// Pin Definitions
//#define N1Pin 4
#define flowSensorPin 0
#define PressurePin A17
#define GPS_Standin A16  // Pot


// User Variables - Define your data as a struct for easier handling
struct UserSettings {
  float GPATarget = 8.0;            // Target Gallonms per acre
  float SprayWidth = 36;            // Nozzel spray width for calibration
  float FlowCalibration = 7.71;     // Flow calibration factor. Calc by Calabration function
  float PSICalibration = .075;      // measured volts on pin vs volts machine shows
  float DutyCycleAdjustment = .10;  // factor to adjust nozzle duty cycle
  float PressureTarget = 25;        // Target Sprayer pressure
  byte numberNozzles = 1;           // How many nozzles on boom
  float currentDutyCycle = 35.0;    // Starting PWM %
  unsigned int Hz = 15;             // 0 - 30 Hz pwm rate
  byte LowBallValve = 28;           // Low value to start movement of ball valve
  byte Ball_Hyd = 1;                // 0 = elec ball valve 1 = Hyd PWM control
  byte WheelAngle = 5;              // Wheel angle to start turn compensasion

  // PID variables
  double Kp = 1.0;
  double Ki = 0.1;
  double Kd = 0.01;
};

UserSettings userSettings;  // Instantiate the struct to hold your settings

// PID
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, userSettings.Kp, userSettings.Ki, userSettings.Kd, DIRECT);

// Variables for NozzleSpeed
// byte numberNozzles = 1;  // In user variables
// Spraywidth above
// gpsSpeed below from AOG         Tractor speed in mph
//steerAngle below from AOG        Turn angle in degrees
float boomLength = 1;                  // Spray width of total boom
const float mphToInchesPerSec = 17.6;  // 1 mph = 17.6 inches/sec
float nozzleSpeedMph = 0;

// Software switches
bool sprayerOn = false;     // Main Switch
byte PWM_Conventional = 0;  // Select PWN or conventional On/Off
byte Stagger = 0;           //Select even/odd staggered firing
byte Start = 0;             // Used in calibrate
byte Update = 0;            // Used in calibrate

// Global variables for startup handling
bool isStartup = true;                  // Flag to indicate if the system is in the startup phase
elapsedMillis startupTimer;             // Timer to track the startup delay
const unsigned long startupDelay = 00;  // 5 seconds delay before starting dynamic adjustment
float defaultDutyCycle = 35.0;          // Default duty cycle at startup

// Flow variables
volatile int pulseCount = 0;
float flowRate = 0.0;
float GPM = 0.0;
double actualGPA = 0.0;
float newDutyCycle = 0;
bool nozzleState = LOW;  // Current state of the nozzle (HIGH or LOW)
double actualGPAave = 19;
uint8_t numActiveNozzles = 0;

// Pressure Variables
const float minPressure = 0.0;
const float maxPressure = 100.0;
double pressure = 20.0;
float voltage = 0;
float measuredVotlage = 0;

// From AOG
float steerAngle = 10;
double gpsSpeed = 4.0;
bool isPinOn = false;
byte PinNum = 0;
// numActiveNozzles = 0;  // In Flow Variable above
float leftSpeed = 0;
float rightSpeed = 0;
//byte numMachineOutputs = 1;

// Variables for timing
unsigned long period = 0;  // Total period of the cycle in milliseconds
double onTime = 50;        // On-time in milliseconds

// EvenOdd variables
elapsedMillis timeSinceLastToggle;  // Automatically increments with elapsed time
bool evenNozzlesActive = true;      // Flag to track if even nozzles are active
unsigned long toggleInterval = 0;   // This will be calculated based on newdutycycle

// Running average objects
RunningAverage pulseAvg(6);  // Running average for sensor value (10-sample buffer)
RunningAverage GPA_Avg(6);   // Running average for voltage (10-sample buffer)

// Timer variables
elapsedMillis Flow_Timer;
elapsedMillis printTimer;
elapsedMillis read_pressure;
elapsedMillis pwmTimer;  // Timer to control the PWM cycle
elapsedMillis Sensor_Data_Send;
elapsedMillis SendUserSetting;

const int USER_SETTINGS_ADDR = 300;  // Start `UserSettings` EEPROM

// Interrupt Service Routine (ISR) for counting pulses
void pulseCounter() {
  pulseCount++;
}
