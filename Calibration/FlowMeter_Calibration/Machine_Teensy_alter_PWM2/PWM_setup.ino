// I'm not smart enough to figure this out by myself.  ChatGPT is my friend,  Ray Jorgensen  11/7/2024

void PWM_setup() {

  pinMode(PinNum, OUTPUT);
  pinMode(flowSensorPin, INPUT_PULLUP);
  pinMode(29, OUTPUT);  // PWM pin
  pinMode(30, OUTPUT);  // DIR pin

  //EEPROM.begin(256);  // Initialize EEPROM

  setupUDP9999();  // Start UDP listening on port 9999
  GeneralAOGUDP();
  SprayerSetupUDP();

  Serial.print("Size of UserSettings: ");
  Serial.println(sizeof(UserSettings));

  loadUserSettingsFromEEPROM();

  // Initialize the PID controller with the loaded settings
  myPID.SetTunings(userSettings.Kp, userSettings.Ki, userSettings.Kd);
  Setpoint = userSettings.PressureTarget;
  myPID.SetMode(AUTOMATIC);  // Set PID to automatic mode if required
  myPID.SetOutputLimits(-255, 255);

  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);

  pulseAvg.clear();
  GPA_Avg.clear();

  // Set the default duty cycle at startup
  userSettings.currentDutyCycle = defaultDutyCycle;
  ControlNozzle();
}