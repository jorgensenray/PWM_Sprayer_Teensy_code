// I'm not smart enough to figure this out by myself.  ChatGPT is my friend.  Ray Jorgensen  11/7/2024
// M

// Add - PinNum = machineOutputPins[i];  // Store the pin number - to outputs.ino

void PWM_loop() {

  AOG_Info();

  if (SendUserSetting > 1000) {
    //sendSensorData();
    //sendUserSettingsToCSharp()
    SendUserSetting = 0;
  }

  if (sprayerOn == 1) {           // Main on off switch 1 == on
    Flow();                       // Flow sensor
    Pressure();                   // Pressure sensor
    if (PWM_Conventional == 1) {  // 1 = PWM or 0 = just on/off switch
      if (isPinOn) {              // Is the pin set ON

        // stagger switch ON with no turn compensation
        if ((Stagger == 1) && (steerAngle <= userSettings.WheelAngle || steerAngle >= -userSettings.WheelAngle)) {
          EvenOdd();        // Even on/odd off - even off/odd on
          ControlNozzle();  // control nozzles
        }

        // stagger switch ON with turn compensation
        if ((Stagger == 1) && (steerAngle >= userSettings.WheelAngle || steerAngle <= -userSettings.WheelAngle)) {
          EvenOdd();        // Even on/odd off - even off/odd on
          NozzleSpeed();    // individual nozzle speed compinsation in a turn
          ControlNozzle();  // control nozzles with normal PWM
        }

        // stagger switch OFF with turn compensation
        if ((Stagger == 0) && (steerAngle >= userSettings.WheelAngle || steerAngle <= -userSettings.WheelAngle)) {
          NozzleSpeed();    // individual nozzle speed compinsation in a turn
          ControlNozzle();  // control nozzles with normal PWM
        }

        // stagger switch OFF with no turn compensation
        ControlNozzle();  // control nozzles with normal PWM
      }
    } else {
      digitalWrite(PinNum, isPinOn ? HIGH : LOW);  // control nozzles conventional on/off
    }
  }
  debugPwmLevel = 0;
}


void AOG_Info() {
  //
  boomLength = (numActiveNozzles * userSettings.SprayWidth);
  gpsSpeed = ((machine.states.gpsSpeed * .10) * 0.621371);  // ((KMPH * 10 from AOG)*.621371) = MPH
  //leftSpeed = (((machine.states.leftSpeed * .01) * 3.28084) * 0.681818);    //** speed is m/s x10 * 3.28084 = F/S
  //rightSpeed = (((machine.states.rightSpeed * .01) * 3.28084) * 0.681818);  //** speed is m/s x10 * 3.28084 = F/S

  Is_Pin_On();
  Pwm_Num_ActivePins();

  if (printTimer > 1000) {
    if (debugPwmLevel == 9) {
      Serial.print(" boomLength = ");
      Serial.print(boomLength);
      Serial.print(" numActiveNozzles = ");
      Serial.print(numActiveNozzles);
      Serial.print(" gpsSpeed = ");
      Serial.print(gpsSpeed);
      Serial.print(" PinNum = ");
      Serial.print(PinNum);
      Serial.print(" isPinOn = ");
      Serial.println(isPinOn);
    }
    printTimer = 0;
  }
}


void Flow() {

  if (Flow_Timer > 500) {
    detachInterrupt(digitalPinToInterrupt(flowSensorPin));      // Suspend interrupt during calculations
    pulseAvg.addValue(pulseCount);                              // Add sensorvalue to the running average
    float pulseCountAve = pulseAvg.getAverage();                // Retrieve the averaged sensorvalue
    flowRate = (pulseCountAve / userSettings.FlowCalibration);  // Flowrate is LPM

    GPM = flowRate / 3.785;                              // Convert to GPM
    actualGPA = (GPM * 5940) / (gpsSpeed * boomLength);  // Calculate GPA
    GPA_Avg.addValue(actualGPA);                         // Add voltage to the running average
    actualGPAave = GPA_Avg.getAverage();                 // Retrieve the averaged voltage

    pulseCount = 0;  // Reset the pulse counter to get ready for next cycle
    Flow_Timer = 0;  // Reset for next cycle

    attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);

    // Avoid using invalid data during startup
    if (!isStartup && gpsSpeed > 0 && actualGPA > 0) {
      float GPAError = userSettings.GPATarget - actualGPA;
      userSettings.currentDutyCycle += GPAError * 0.1;                                   // Adjust the scaling factor (0.1) for smoother control
      userSettings.currentDutyCycle = constrain(userSettings.currentDutyCycle, 0, 100);  // Constrain cycle between 0 and 100
    }

    pulseCount = 0;  // Reset the pulse counter to get ready for next cycle
    Flow_Timer = 0;  //  Reset for next cycle

    attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);

    // Calculate the new duty cycle based on actual GPA vs target GPA
    if (gpsSpeed > 0) {                              // Ensure we're moving to calculate duty cycle
      newDutyCycle = userSettings.currentDutyCycle;  // Start with the current duty cycle

      if (actualGPAave < userSettings.GPATarget) {                                                           // actual lower than target, increase
        newDutyCycle += (userSettings.GPATarget - actualGPAave) * userSettings.DutyCycleAdjustment;          // Adjust factor as needed
      } else if (actualGPAave > userSettings.GPATarget) {                                                    // actual higher than target, decrease
        newDutyCycle -= ((actualGPAave - userSettings.GPATarget) * (1 + userSettings.DutyCycleAdjustment));  // Adjust factor
      }
      newDutyCycle = constrain(newDutyCycle, 0.0, 100.0);  //Limit duty cycle to a valid range (0 to 100%)
      setPWMTiming(userSettings.Hz, newDutyCycle);         // Update the duty cycle and set the new timing for PWM
    }
  }

  if (printTimer > 1000) {
    if (debugPwmLevel == 6) {
      Serial.print(" flowRate: ");
      Serial.print(flowRate);
      Serial.print(" actualGPAave: ");
      Serial.print(actualGPAave);
      Serial.print(" newDutyCycle: ");
      Serial.println(newDutyCycle);
    }
    printTimer = 7;
  }
}


void NozzleSpeed() {

  float tractorSpeedIps = gpsSpeed * mphToInchesPerSec;    // Convert tractor speed to inches per second
  float turnRadius = 360 / (2 * M_PI * steerAngle / 360);  // turning radius using turn angle and a simple model

  // distance from center of boom
  float distanceFromCenter = (PinNum - (boomLength / 2.0 / userSettings.SprayWidth)) * userSettings.SprayWidth;
  float nozzleRadius = turnRadius + distanceFromCenter;                  // radius for this nozzle position
  float nozzleSpeedIps = (nozzleRadius / turnRadius) * tractorSpeedIps;  // speed of the nozzle IPS
  nozzleSpeedMph = nozzleSpeedIps / 17.6;                                // speed of the nozzle MPH

  if (printTimer > 1000) {
    if (debugPwmLevel == 7) {
      Serial.print(" steerAngle ");
      Serial.print(steerAngle);
      Serial.print(" Nozzle # ");
      Serial.print(PinNum);
      Serial.print(": ");
      Serial.print(nozzleSpeedIps);
      Serial.print(" inches/sec ");
      Serial.print(" MPH ");
      Serial.println(nozzleSpeedMph);
    }
    printTimer = 0;
  }
  dutycycleTurncomp();
}


void dutycycleTurncomp() {

  actualGPA = (GPM * 5940) / (nozzleSpeedMph * userSettings.SprayWidth);  // Calculate GPA

  if (actualGPAave < userSettings.GPATarget) {                                                   // actual < target, increase
    newDutyCycle += (userSettings.GPATarget - actualGPAave) * userSettings.DutyCycleAdjustment;  // Adjust factor
  } else if (actualGPAave > userSettings.GPATarget) {                                            // actual > target, decrease
    newDutyCycle -= ((actualGPAave - userSettings.GPATarget) * (1 + userSettings.DutyCycleAdjustment));
  }
  newDutyCycle = constrain(newDutyCycle, 0.0, 100.0);  //Limit duty cycle to a valid range (0 to 100%)
  setPWMTiming(userSettings.Hz, newDutyCycle);         // Update the duty cycle and set the new timing for PWM

  if (printTimer > 1000) {
    if (debugPwmLevel == 1) {
      Serial.print("newDutyCycle ");
      Serial.print(newDutyCycle);
      Serial.print(" onTime ");
      Serial.print(onTime);
      Serial.print(" userSettings.Hz ");
      Serial.println(userSettings.Hz);
    }
    printTimer = 0;
  }
}


void setPWMTiming(unsigned int freq, float dutyCycle) {
  freq = userSettings.Hz;
  period = 1000 / freq;                          // Calculate the period in milliseconds
  onTime = period * (newDutyCycle / 100.0);      // Calculate the ON time based on duty cycle
  userSettings.currentDutyCycle = newDutyCycle;  // Update current duty cycle
  userSettings.Hz = freq;                        // Update frequency

  if (printTimer > 1000) {
    if (debugPwmLevel == 2) {
      Serial.print("newDutyCycle ");
      Serial.print(newDutyCycle);
      Serial.print(" onTime ");
      Serial.print(onTime);
      Serial.print(" userSettings.Hz ");
      Serial.println(userSettings.Hz);
    }
    printTimer = 0;
  }
}


void ControlNozzle() {

  if (nozzleState == HIGH && pwmTimer >= onTime) {                   // Function to control the nozzle based on elapsed time
    digitalWrite(PinNum, LOW);                                       // nozzle is currently ON and the onTime has passed,
    nozzleState = LOW;                                               // turn it OFF
    pwmTimer = 0;                                                    // Reset the timer for the OFF phase
  } else if (nozzleState == LOW && pwmTimer >= (period - onTime)) {  // nozzle is currently OFF and the offTime has passed,
    digitalWrite(PinNum, HIGH);                                      // turn it ON
    nozzleState = HIGH;                                              //
    pwmTimer = 0;                                                    // Reset the timer for the ON phase
  }

  if (printTimer > 1000) {
    if (debugPwmLevel == 3) {
      Serial.print("newDutyCycle ");
      Serial.print(newDutyCycle);
      Serial.print(" onTime ");
      Serial.print(onTime);
      Serial.print(" userSettings.Hz ");
      Serial.println(userSettings.Hz);
    }
    printTimer = 0;
  }
}


void Pressure() {
  if (read_pressure >= 100) {                     // Take readings every 100 milliseconds
    float sensorvalue = analogRead(PressurePin);  // Read the pressure pin
    voltage = sensorvalue * (5.0 / 1023.0);       // Convert the averaged sensor value to voltage
    //userSettings.PSICalibration = measuredVotlage - voltage;   // Calibration Calcs
    voltage -= userSettings.PSICalibration;  // Apply an adjustment factor (for calibration)
    pressure = ((voltage - 0.5) * (maxPressure - minPressure)) / (4.5 - 0.5) + minPressure;
    read_pressure = 0;  // Reset the timer for the next reading

    if (userSettings.Ball_Hyd == 1) {
      Output = userSettings.PressureTarget - pressure;
      if ((Output <= userSettings.LowBallValve) && (Output > 2)) {
        Output = userSettings.LowBallValve;
      }
      if ((Output >= -userSettings.LowBallValve) && (Output < -2)) {
        Output = -userSettings.LowBallValve;
      }
      if ((Output <= 1) && (Output >= -1)) {
        (Output = 0);
      }
    } else {
      Input = pressure;
      myPID.Compute();
    }

    motor.setSpeed(Output);

    if (printTimer > 1000) {
      if (debugPwmLevel == 4) {
        Serial.print("sensorvalue ");
        Serial.print(sensorvalue);
        Serial.print(" voltage ");
        Serial.print(voltage);
        Serial.print(" pressure ");
        Serial.println(pressure);
      }
      printTimer = 0;
    }
  }
}


void EvenOdd() {

  // Calculate the toggle interval based on the newdutycycle
  toggleInterval = (onTime / 2);  // 1/2 period in ms, assuming newdutycycle is in userSettings.Hz

  // Check if the toggle interval has passed
  if (timeSinceLastToggle >= toggleInterval) {
    timeSinceLastToggle = 0;                 // Reset the elapsed time
    evenNozzlesActive = !evenNozzlesActive;  // Toggle between even and odd nozzles
  }

  bool isEven = (PinNum % 2 == 0);  // Determine if it's an even nozzle

  if ((evenNozzlesActive && isEven) || (!evenNozzlesActive && !isEven)) {
    // If the nozzle should be active (either even or odd, depending on the toggle)
    setPWMTiming(PinNum, newDutyCycle);  // Set the PWM timing based on newdutycycle
  } else {
    // If the nozzle should not be active, set its PWM timing to 0 or turn it off
    setPWMTiming(PinNum, 0);  // Disable the nozzle by setting PWM to 0
  }

  if (printTimer > 1000) {
    if (debugPwmLevel == 5) {
      Serial.print(" isEven ");
      Serial.print(isEven);
      Serial.print(" PinNum ");
      Serial.println(PinNum);
    }
    printTimer = 0;
  }
}


void PrintDebug() {
  if (debugPwmLevel == 8) {
    Serial.print(" gpsSpeed ");
    Serial.print(gpsSpeed);
    Serial.print(" Actual GPA: ");
    Serial.print(actualGPA);
    Serial.print(" onTime: ");
    Serial.print(onTime);
    //Serial.print(" period: ");
    //Serial.print(period);
    Serial.print(" Pressure: ");
    Serial.println(pressure);
  }
}


void PrintAOG() {
  if (debugPwmLevel == 9) {
    Serial.print(" steerAngle ");
    Serial.print(steerAngle);
    Serial.print(" gpsSpeed ");
    Serial.print(gpsSpeed);
    Serial.print(" isPinOn ");
    Serial.print(isPinOn);
    Serial.print(" PinNum ");
    Serial.println(PinNum);
    //Serial.print(" uturn ");
    //Serial.println(uturn);
  }
}


void Calibrate_PSI_Flow() {

  Pressure();

  if (Flow_Timer > 500) {
    detachInterrupt(digitalPinToInterrupt(flowSensorPin));  // Suspend interrupt during calculations

    // Calculate the average pulse count and flow rate (LPM)
    pulseAvg.addValue(pulseCount);
    float pulseCountAve = pulseAvg.getAverage();
    flowRate = pulseCountAve / userSettings.FlowCalibration;  // Flow rate in LPM

    // Convert to GPM and calculate the actual GPA
    GPM = flowRate / 3.785;  // Convert LPM to GPM
    actualGPA = (GPM * 5940) / (gpsSpeed * (userSettings.SprayWidth));
    GPA_Avg.addValue(actualGPA);
    actualGPAave = GPA_Avg.getAverage();

    // Update FlowCalibration based on the difference between target and actual GPA
    float errorGPA = actualGPAave - userSettings.GPATarget;
    //float errorGPA = GPATarget - actualGPAave;

    float adjustmentFactor = 0.01;

    if ((errorGPA < 0.02) && (errorGPA > 0.00)) {
      if (debugPwmLevel == 10) {
        //Save To EEPROM();
        Serial.print("Calibration complete - value = ");
        Serial.print(userSettings.FlowCalibration);
        saveuserSettingsToEEPROM();
        Serial.print(" has been saved to EEPROM. ");
        debugPwmLevel = 0;
      }
    }

    userSettings.FlowCalibration += adjustmentFactor * errorGPA;

    pulseCount = 0;  // Reset pulse counter
    Flow_Timer = 0;  // Reset timer

    attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);

    // Print debug information
    if (debugPwmLevel == 10) {
      Serial.print(" Output: ");
      Serial.print(Output);
      Serial.print(" | pressure: ");
      Serial.print(pressure);
      Serial.print(" | FlowCalibration: ");
      Serial.print(userSettings.FlowCalibration, 3);
      Serial.print(" | actualGPAave: ");
      Serial.print(actualGPAave);
      Serial.print(" | GPATarget: ");
      Serial.println(userSettings.GPATarget);
    }
  }
}


void Pwm_Num_ActivePins() {
  if (isPinOn) {
    if (numActiveNozzles <= 15) {  // This will exclude Trams, etc.
      numActiveNozzles++;          // Increment the active nozzle count
      //activePins.push_back(machineOutputPins[i]);  // Track the active pin
    }
  }
}


void Is_Pin_On() {

  // determine if pin is "ON" depending on relay inverting or not
  if ((machine.state.functions[machine.config.pinFunction[PinNum]] == 1) && (machine.config.isPinActiveHigh == 1)) {
    isPinOn = true;
  }
  if ((machine.state.functions[machine.config.pinFunction[PinNum]] == 0) && (machine.config.isPinActiveHigh == 1)) {
    isPinOn = false;
  }
  if ((machine.state.functions[machine.config.pinFunction[PinNum]] == 0) && (machine.config.isPinActiveHigh == 0)) {
    isPinOn = true;
  }
  if ((machine.state.functions[machine.config.pinFunction[PinNum]] == 1) && (machine.config.isPinActiveHigh == 0)) {
    isPinOn = false;
  }
}


void setDebugPwmLevel(uint8_t level) {
  debugPwmLevel = level;
}

void loadUserSettingsFromEEPROM() {
  UserSettings tempSettings;
  EEPROM.get(USER_SETTINGS_ADDR, tempSettings);

  // Comprehensive validity check for multiple fields
  if (tempSettings.GPATarget > 0 && tempSettings.GPATarget < 20 && tempSettings.SprayWidth > 0 && tempSettings.SprayWidth < 100 && tempSettings.FlowCalibration > 0 && tempSettings.FlowCalibration < 50 && tempSettings.PSICalibration > 0 && tempSettings.PSICalibration < 1 && tempSettings.DutyCycleAdjustment > 0 && tempSettings.DutyCycleAdjustment < 1) {
    userSettings = tempSettings;
    Serial.println("User variables loaded successfully.");
    PrintUserVariables();
  } else {
    Serial.println("Invalid settings detected. Using defaults.");
    saveuserSettingsToEEPROM();  // Save defaults
  }
}

void saveuserSettingsToEEPROM() {
  EEPROM.put(USER_SETTINGS_ADDR, userSettings);
  Serial.println("User settings saved to EEPROM.");
  PrintUserVariables();
}

void PrintUserVariables() {
  // Debug: Print all loaded values
  if (debugPwmLevel == 10) {
    Serial.println("");
    Serial.println("");
    Serial.println("GPATarget: " + String(userSettings.GPATarget));
    Serial.println("SprayWidth: " + String(userSettings.SprayWidth));
    Serial.println("FlowCalibration: " + String(userSettings.FlowCalibration));
    Serial.println("PSICalibration: " + String(userSettings.PSICalibration));
    Serial.println("DutyCycleAdjustment: " + String(userSettings.DutyCycleAdjustment));
    Serial.println("PressureTarget: " + String(userSettings.PressureTarget));
    Serial.println("numberNozzles: " + String(userSettings.numberNozzles));
    Serial.println("currentDutyCycle: " + String(userSettings.currentDutyCycle));
    Serial.println("Hz: " + String(userSettings.Hz));
    Serial.println("LowBallValve: " + String(userSettings.LowBallValve));
    Serial.println("Ball_Hyd: " + String(userSettings.Ball_Hyd));
    Serial.println("WheelAngle: " + String(userSettings.WheelAngle));
    Serial.println("Kp: " + String(userSettings.Kp));
    Serial.println("Ki: " + String(userSettings.Ki));
    Serial.println("Kd: " + String(userSettings.Kd));
    Serial.println("");
    Serial.println("");
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}