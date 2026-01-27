
// I'm not smart enough to figure this out by myself.  ChatGPT is my friend.  Ray Jorgensen  11/7/2024
// https://github.com/jorgensenray

// Declared because function uses a default parameter:
void BasicPWM(size_t nozzleIndex, uint8_t currentPin, uint8_t dutyPercent = 50);
// Forward declarations
void ControlNozzles(size_t nozzleIndex, uint8_t currentPin, uint32_t cycleTimeMs);
void EvenOdd(size_t nozzleIndex, uint8_t currentPin, bool globalEvenNozzlesActive);

void PWM_Controls() {

  // Compute period ONCE per pass (and guarantee it's non-zero)
  uint32_t hz = userSettings.Hz;
  if (hz < 1) hz = 1;
  period = 1000UL / hz;
  if (period < 1) period = 1;

  // Use a single shared timebase for ALL nozzles this pass
  uint32_t cycleTimeMs = millis() % period;

  // Do these ONCE per pass, not once per nozzle
  boomLength = (numActiveNozzles * userSettings.SprayWidth);
  TurnComp = (fabs(steerAngle) > userSettings.WheelAngle);

  PrintAOGstuff();
  Pressure();
  Flow();

  for (size_t i = 0; i < pinStates.size(); i++) {

    bool isPinOnLocal = pinStates[i].state;
    uint8_t currentPinLocal = pinStates[i].pinNumber;  // GPIO pin to drive
    uint8_t fn = pinStates[i].fn;                      // function id

    // Only handle real spray sections (S01..Sxx)
    if (fn < 1 || fn > maxNozzles) {
      continue;
    }

    // fn 1->index0, fn 2->index1, ...
    size_t nozzleIndex = (size_t)(fn - 1);

    if (isPinOnLocal) {

      // Conventional PWM (no stagger)
      if ((userSettings.PWM_Conventional == 1) && (userSettings.Stagger == 0) && (TurnComp == false)) {
        setPWMTiming(userSettings.Hz, newDutyCycle, nozzleIndex);
        ControlNozzles(nozzleIndex, currentPinLocal, cycleTimeMs);
      }

      // Even/Odd (stagger)
      if ((userSettings.PWM_Conventional == 1) && (userSettings.Stagger == 1) && (TurnComp == false)) {
        EvenOdd(nozzleIndex, currentPinLocal, true);
        ControlNozzles(nozzleIndex, currentPinLocal, cycleTimeMs);
      }

      // Even/Odd with turn comp
      if ((userSettings.PWM_Conventional == 1) && (userSettings.Stagger == 1) && (TurnComp == true)) {
        EvenOdd(nozzleIndex, currentPinLocal, true);
        if (OnTime[nozzleIndex] > 0) {
          NozzleSpeed(nozzleIndex, currentPinLocal);
        }
        ControlNozzles(nozzleIndex, currentPinLocal, cycleTimeMs);
      }

      // Conventional with turn comp (no stagger)
      if ((userSettings.PWM_Conventional == 1) && (userSettings.Stagger == 0) && (TurnComp == true)) {
        NozzleSpeed(nozzleIndex, currentPinLocal);
        ControlNozzles(nozzleIndex, currentPinLocal, cycleTimeMs);
      }

      // Conventional ON/OFF (no PWM)
      if (userSettings.PWM_Conventional == 0) {
        digitalWrite(currentPinLocal, HIGH);
      }

    } else {
      digitalWrite(currentPinLocal, LOW);
    }
  }
}

// ----- Fast GPA display filter -----
// float gpaDisplay = 0.0f;           // fast-smoothed GPA for the user - made global
const float gpaDisplayAlpha = 0.45f;  // 0..1, higher = more responsive, more noisy

void Flow() {
  // Only bother if at least one nozzle is on
  if (numActiveNozzles == 0) {
    return;
  }

  // Wait until our flow sample window has elapsed
  if (Flow_Timer > FLOW_SAMPLE_MS) {

    // --- Snapshot pulses atomically ---
    noInterrupts();
    uint32_t pulses = pulseCount;
    pulseCount = 0;  // reset for next window
    interrupts();

    // Sample period in seconds
    const float sec = FLOW_SAMPLE_MS / 1000.0f;

    // --- Compute flow in LPM from pulses over this window ---
    float flowLpm = 0.0f;
    if (sec > 0.0f && PSSL > 0.0f) {
      // pulses/sec
      float pulsesPerSec = (float)pulses / sec;

      // LPM = (pulses/sec) / PSSL * 60
      flowLpm = (pulsesPerSec / PSSL) * 60.0f;
    }

    // --- Smooth LPM with moving average ---
    pulseAvg.addValue(flowLpm);
    float flowLpmAvg = pulseAvg.getAverage();

    // --- Convert to GPM (for display & control) ---
    GPM = flowLpmAvg / 3.78541f;

    // --- Compute GPA (for information) ---
    // --- GPA calculations ---
    // Instant GPA from current (smoothed) GPM
    float gpaInstant = 0.0f;
    if (gpsSpeed > 0.01f && boomLength > 0.0f) {
      gpaInstant = (GPM * 5940.0f) / (gpsSpeed * boomLength);
    }

    // 1) Fast display GPA (EMA) for the user
    //    More responsive than the moving average, but still smoothed.
    if (gpaDisplay == 0.0f) {
      // First-time init to avoid a big jump from 0 → real value
      gpaDisplay = gpaInstant;
    } else {
      gpaDisplay = gpaDisplayAlpha * gpaInstant + (1.0f - gpaDisplayAlpha) * gpaDisplay;
    }

    // 2) Slower moving-average GPA (if you still want it for logs)
    GPA_Avg.addValue(gpaInstant);
    actualGPAave = GPA_Avg.getAverage();

    // Keep actualGPA for compatibility if you use it elsewhere
    actualGPA = gpaInstant;


    // ==========================================================
    //   FLOW-BASED CLOSED-LOOP CONTROL (tames the runaway)
    // ==========================================================
    if (gpsSpeed > 0.01f && boomLength > 0.0f) {
      // How much flow we *should* have at this speed & boom size
      float targetGPM = (userSettings.GPATarget * gpsSpeed * boomLength) / 5940.0f;

      // Error in GPM (positive => we're low, need more duty)
      float errGPM = targetGPM - GPM;

      // User-tunable gain
      float k = userSettings.DutyCycleAdjustment;  // e.g. start with 0.5

      // Proposed duty step this sample
      float deltaDuty = errGPM * k;  // % per sample

      // Clamp how fast duty can move per sample
      if (deltaDuty > maxStep) deltaDuty = maxStep;
      if (deltaDuty < -maxStep) deltaDuty = -maxStep;

      userSettings.currentDutyCycle += deltaDuty;
      userSettings.currentDutyCycle = constrain(userSettings.currentDutyCycle, 0.0f, 100.0f);
    }

    // Use the adjusted duty for PWM / K&H
    newDutyCycle = userSettings.currentDutyCycle;

    // --- Acres calculation (rate + cumulative) ---
    // Only accumulate when we're actually spraying
    if (numActiveNozzles > 0 && gpsSpeed > 0.01f) {
      // boomLength is in inches; convert to feet
      float widthFt = boomLength / 12.0f;

      // Instantaneous acres/hour
      acresPerHour = (gpsSpeed * widthFt) / 8.25f;

      // Time step in hours, based on FLOW_SAMPLE_MS
      float dtHours = (float)FLOW_SAMPLE_MS / 3600000.0f;  // ms -> hours

      // Acres sprayed during this sample interval
      float deltaAcres = acresPerHour * dtHours;

      // Accumulate total
      acresTotal += deltaAcres;
    } else {
      // Not spraying – still keep a valid "zero" rate for UI
      acresPerHour = 0.0f;
    }

    // --- Debug output ---
    if (debugPwmLevel == 6) {
      Serial.print(F("Flow/GPA | pulses: "));
      Serial.print(pulses);
      Serial.print(F(" Pressure: "));
      Serial.print(pressure);
      Serial.print(F("  LPM(avg): "));
      Serial.print(flowLpmAvg, 2);
      Serial.print(F("  GPM: "));
      Serial.print(GPM, 3);
      Serial.print(F("  targetGPM: "));
      if (gpsSpeed > 0.01f && boomLength > 0.0f) {
        float targetGPM = (userSettings.GPATarget * gpsSpeed * boomLength) / 5940.0f;
        Serial.print(targetGPM, 3);
      } else {
        Serial.print(F("N/A"));
      }
      Serial.print(F("  GPA target: "));
      Serial.print(userSettings.GPATarget, 2);
      Serial.print(F("  gpaDisplay: "));
      Serial.print(gpaDisplay, 2);
      Serial.print(F("  GPA actual(avg): "));
      Serial.print(actualGPAave, 2);
      Serial.print(F("  duty: "));
      Serial.println(newDutyCycle, 2);
    }

    // Reset the timer for the next sample window
    Flow_Timer = 0;
  }
}


//-------------------------------------------------------------------
// setPWMTiming: Calculate OnTime per nozzle based on duty cycle
//-------------------------------------------------------------------
void setPWMTiming(unsigned int freq, float dutyCycle, size_t nozzleIndex) {
  // period is maintained in PWM_Controls() once per loop

  // Clamp duty
  if (dutyCycle < 0.0f) dutyCycle = 0.0f;
  if (dutyCycle > 100.0f) dutyCycle = 100.0f;

  if (nozzleIndex < maxNozzles) {
    OnTime[nozzleIndex] = (uint32_t)((period * dutyCycle) / 100.0f);
  }

  if (nozzleIndex < maxNozzles) {
    OnTime[nozzleIndex] = static_cast<uint32_t>((period * dutyCycle) / 100.0f);
  }
  if (debugPwmLevel == 2) {
    if (printTimer > PrintFrequency) {
      Serial.print("newDutyCycle (from parameter) ");
      Serial.print(dutyCycle);
      Serial.print(" OnTime for nozzle ");
      Serial.print(nozzleIndex);
      Serial.print(" = ");
      Serial.println(OnTime[nozzleIndex]);
      Serial.print(" userSettings.Hz ");
      Serial.println(userSettings.Hz);
      printTimer = 0;
    }
  }
}

void EvenOdd(size_t nozzleIndex, uint8_t currentPin, bool /*globalEvenNozzlesActive*/) {
  // Even pins: phase 0
  // Odd  pins: phase = half of the PWM period
  bool isEven = (currentPin % 2 == 0);

  uint32_t perMs = period;  // global PWM period in ms (already maintained)
  uint32_t phase = 0;

  if (!isEven) {
    // Half-period phase shift for odd nozzles
    phase = perMs / 2;
  }

  // Store per-nozzle phase offset
  if (nozzleIndex < maxNozzles) {
    PwmPhaseOffset[nozzleIndex] = phase;
  }

  // All nozzles keep the SAME duty cycle.
  // Even/odd only affects *when* in the period they are ON, not *how much*.
  setPWMTiming(userSettings.Hz, newDutyCycle, nozzleIndex);

  if (debugPwmLevel == 5) {
    Serial.print("EvenOdd Debug - Nozzle: ");
    Serial.print(nozzleIndex);
    Serial.print(" | Pin: ");
    Serial.print(currentPin);
    Serial.print(" | isEven: ");
    Serial.print(isEven ? "true" : "false");
    Serial.print(" | phaseOffset(ms): ");
    Serial.println(phase);
  }
}

float EMA(float newValue) {
  filteredADC = alpha * newValue + (1 - alpha) * filteredADC;
  return filteredADC;
}

void Pressure() {
  static uint32_t lastPressureMs = 0;
  uint32_t now = millis();

  // Only control pressure when spraying
  if (numActiveNozzles <= 0) return;

  // 100ms gate (match PID sample time)
  uint32_t dt = now - lastPressureMs;
  if (dt < 100) return;
  lastPressureMs = now;

  // ---- Read pressure (once) ----
  float rawADC = analogRead(PressurePin);
  float smoothedADC = EMA(rawADC);

  float voltage = smoothedADC * (3.3f / 1023.0f);
  voltage += userSettings.PSICalibration;

  pressure = ((voltage - 0.33f) * (maxPressure - minPressure)) / (2.97f - 0.33f) + minPressure;

  // ---- If not actively chasing a new target: hold last applied cmd ----
  if (!pressureControlActive) {
    motor.setSpeed(lastMotorCmd);
    return;
  }

  // ---- PID compute ----
  Input = (double)pressure;
  Setpoint = (double)userSettings.PressureTarget;

  // PID_v1 Compute() returns bool (true when it actually computed a new output)
  // If it returns false, Output is unchanged; we still "hold" lastMotorCmd.
  bool didCompute = myPID.Compute();

  // ---- Output shaping to prevent hunting ----
  float error = userSettings.PressureTarget - pressure;

  // Treat LowBallValve as PID_MIN_MOVE (adjust it down in your UI; start ~10–15)
  const int PID_MIN_MOVE = (int)userSettings.LowBallValve;

  // Two bands:
  // - NEAR_BAND: stop nudging to avoid limit-cycle caused by min-move + noise
  // - PRESSURE_TOL/SETTLE_COUNT: "done" detection (you already have these)
  const float NEAR_BAND = 1.0f;  // psi (tune 0.8–2.0)
  const int NEAR_MAX_CMD = 50;   // gentle near target (tune 50–120)

  int cmd;

  if (!didCompute) {
    // No new PID output this pass, hold last
    cmd = lastMotorCmd;
  } else {
    // Start with raw PID output
    cmd = (int)Output;

    // If we're already close, STOP moving (prevents endless hunting)
    if (fabsf(error) <= NEAR_BAND) {
      // near target: allow small commands (no min-move)
      cmd = (int)Output;
    } else {
      // far away: overcome stiction
      cmd = applyDeadzone(Output, PID_MIN_MOVE);
    }

    // Slow down near target to reduce overshoot
    int maxCmd = (fabsf(error) > 3.0f) ? 255 : NEAR_MAX_CMD;
    if (cmd > maxCmd) cmd = maxCmd;
    if (cmd < -maxCmd) cmd = -maxCmd;

    // Absolute clamp (safe)
    if (cmd > 255) cmd = 255;
    if (cmd < -255) cmd = -255;
  }

  motor.setSpeed(cmd);
  lastMotorCmd = cmd;

  if (debugPwmLevel == 4) {
    Serial.print("APPLY Cmd=");
    Serial.println(cmd);
  }

  // ---- Done detection ----
  if (fabsf(error) <= PRESSURE_TOL) {
    if (++settleCounter >= SETTLE_COUNT) {
      pressureControlActive = false;
      settleCounter = 0;
      lastMotorCmd = 0;  // ensure valve is stopped
      motor.setSpeed(0);
      myPID.SetMode(MANUAL);  // freeze integrator while holding
    }
  } else {
    settleCounter = 0;
  }

  if (debugPwmLevel == 4) {
    Serial.print("Err=");
    Serial.print(error);
    Serial.print(" | inTol=");
    Serial.print(fabsf(error) <= PRESSURE_TOL);
    Serial.print(" | settleCounter=");
    Serial.print(settleCounter);
    Serial.print(" | active=");
    Serial.println(pressureControlActive);
  }
}


int applyDeadzone(double out, int minMove) {
  // Clamp first
  if (out > 255) out = 255;
  if (out < -255) out = -255;

  // Deadzone bump to overcome stiction
  if (out > 0 && out < minMove) return minMove;
  if (out < 0 && out > -minMove) return -minMove;

  return (int)out;
}

void ControlNozzles(size_t nozzleIndex, uint8_t currentPin, uint32_t cycleTimeMs) {

  uint32_t perMs = period;
  if (perMs == 0 || nozzleIndex >= maxNozzles) {
    digitalWrite(currentPin, LOW);
    return;
  }

  uint32_t onMs = OnTime[nozzleIndex];
  if (onMs > perMs) onMs = perMs;  // clamp just in case

  // Phase with per-nozzle offset (stagger)
  uint32_t phaseMs = (cycleTimeMs + PwmPhaseOffset[nozzleIndex]) % perMs;

  // ON window?
  bool inOnWindow = (onMs > 0) && (phaseMs < onMs);

  // Debug override (your existing behavior)
  if (debugPwmLevel == 99) {
    digitalWrite(currentPin, inOnWindow ? HIGH : LOW);
    return;
  }

  // Simple PWM output
  digitalWrite(currentPin, inOnWindow ? HIGH : LOW);
}

void NozzleSpeed(size_t nozzleIndex, uint8_t currentPin) {
  // If EvenOdd already set this nozzle off, skip turn compensation.
  if (OnTime[nozzleIndex] == 0) {
    return;
  }

  float tractorSpeedIps = gpsSpeed * 17.6;                 // Convert tractor speed to inches per second
  float turnRadius = 360 / (2 * M_PI * steerAngle / 360);  // Calculate turn radius
  // Calculate the distance from the boom's center for this nozzle.
  float distanceFromCenter = (currentPin - (boomLength / 2.0 / userSettings.SprayWidth)) * userSettings.SprayWidth;
  float nozzleRadius = turnRadius + distanceFromCenter;
  float nozzleSpeedIps = (nozzleRadius / turnRadius) * tractorSpeedIps;
  float nozzleSpeedMph = nozzleSpeedIps / 17.6;

  if (debugPwmLevel == 7) {
    Serial.print("currentPin ");
    Serial.print(currentPin);
    Serial.print(": Speed = ");
    Serial.print(nozzleSpeedMph);
    Serial.println(" MPH");
    printTimer = 0;
  }

  // Now call dutycycleTurncomp with the actual currentPin.
  dutycycleTurncomp(nozzleIndex, currentPin, nozzleSpeedMph);
}


void dutycycleTurncomp(uint8_t nozzleIndex, uint8_t currentPin, float nozzleSpeedMph) {
  // Calculate the actual GPA for the nozzle.
  actualGPA = (GPM * 5940) / (nozzleSpeedMph * userSettings.SprayWidth);
  // Start with the base duty cycle.
  float nozzleDutyCycle = userSettings.currentDutyCycle;
  if (actualGPA < userSettings.GPATarget) {
    nozzleDutyCycle += (userSettings.GPATarget - actualGPA) * userSettings.DutyCycleAdjustment;
  } else if (actualGPA > userSettings.GPATarget) {
    nozzleDutyCycle -= ((actualGPA - userSettings.GPATarget) * (1 + userSettings.DutyCycleAdjustment));
  }

  // Constrain and update the duty cycle.
  newDutyCycle = constrain(nozzleDutyCycle, 0.0, 100.0);
  OnTime[nozzleIndex] = static_cast<uint32_t>(period * (newDutyCycle / 100.0));

  if (debugPwmLevel == 1) {
    Serial.print("Nozzle ");
    Serial.print(nozzleIndex);
    Serial.print(" (Pin ");
    Serial.print(currentPin);
    Serial.print("): newDutyCycle = ");
    Serial.print(newDutyCycle);
    Serial.print(", OnTime = ");
    Serial.print(OnTime[nozzleIndex]);
    Serial.print(": Speed = ");
    Serial.print(nozzleSpeedMph);
    Serial.println(" MPH");
    printTimer = 0;
  }
}

void PrintDebug() {
  if (debugPwmLevel == 8) {
    if (printTimer == PrintFrequency) {
      Serial.print(" gpsSpeed ");
      Serial.print(gpsSpeed);
      Serial.print(" Actual GPA: ");
      Serial.print(actualGPA);
      Serial.print(" onTime: ");
      Serial.print(onTime);
      Serial.print(" period: ");
      Serial.print(period);
      Serial.print(" Pressure: ");
      Serial.println(pressure);
      printTimer = 0;
    }
  }
}


void PrintAOG() {
  if (debugPwmLevel == 9) {
    if (printTimer == PrintFrequency) {
      Serial.print(" steerAngle ");
      Serial.print(steerAngle);
      Serial.print(" gpsSpeed ");
      Serial.print(gpsSpeed);
      Serial.print(" isPinOn ");
      Serial.print(isPinOn);
      Serial.print(" currentPin ");
      Serial.print(currentPin);
      printTimer = 0;
    }
  }
}

void setupNozzles() {
  for (uint8_t i = 0; i < maxNozzles; i++) {
    OnTime[i] = 0;
    PwmTimer[i] = 0;
  }
}

void setDebugPwmLevel(uint8_t level) {
  debugPwmLevel = level;
}

void PrintUserVariables() {
  // Debug: Print all loaded values
  Serial.println();
  Serial.println(F("===== User Settings ====="));
  Serial.print(F("GPATarget: "));
  Serial.println(userSettings.GPATarget);
  Serial.print(F("SprayWidth: "));
  Serial.println(userSettings.SprayWidth);
  Serial.print(F("FlowCalibration: "));
  Serial.println(userSettings.FlowCalibration);
  Serial.print(F("PSICalibration: "));
  Serial.println(userSettings.PSICalibration);
  Serial.print(F("DutyCycleAdjustment: "));
  Serial.println(userSettings.DutyCycleAdjustment);
  Serial.print(F("PressureTarget: "));
  Serial.println(userSettings.PressureTarget);
  Serial.print(F("currentDutyCycle: "));
  Serial.println(userSettings.currentDutyCycle);
  Serial.print(F("Hz: "));
  Serial.println(userSettings.Hz);
  Serial.print(F("LowBallValve: "));
  Serial.println(userSettings.LowBallValve);
  Serial.print(F("Ball_Hyd: "));
  Serial.println(userSettings.Ball_Hyd);
  Serial.print(F("WheelAngle: "));
  Serial.println(userSettings.WheelAngle);
  Serial.print(F("Kp: "));
  Serial.println(userSettings.Kp);
  Serial.print(F("Ki: "));
  Serial.println(userSettings.Ki);
  Serial.print(F("Kd: "));
  Serial.println(userSettings.Kd);
  Serial.print(F("Parsed Unit: "));
  Serial.println(userSettings.unit);
  Serial.print(F("Parsed PWM_Conventional: "));
  Serial.println(userSettings.PWM_Conventional);
  Serial.print(F("Stagger: "));
  Serial.println(userSettings.Stagger);
  Serial.print(F("debugPwmLevel: "));
  Serial.println(userSettings.debugPwmLevel);
}


void PrintAOGstuff() {
  if (debugPwmLevel == 9) {
    Serial.print(" boomLength = ");
    Serial.print(boomLength);
    Serial.print(" numActiveNozzles = ");
    Serial.print(numActiveNozzles);
    Serial.print(" gpsSpeed = ");
    Serial.print(gpsSpeed);
    Serial.print(" currentPin = ");
    Serial.print(currentPin);
    Serial.print(" isPinOn = ");
    Serial.println(isPinOn);
  }
}

void DebugRawPulses() {
  static uint32_t lastPrintMs = 0;

  if (millis() - lastPrintMs >= 500) {  // print every 500 ms
    lastPrintMs = millis();

    noInterrupts();
    uint32_t total = pulseCountTotal;
    interrupts();

    Serial.print("RAW pulseCountTotal (ever-growing): ");
    Serial.println(total);
  }
}

void resetFlowAverages() {
  pulseAvg.clear();  // clear LPM moving average
  GPA_Avg.clear();   // clear GPA moving average
  actualGPAave = 0.0f;
}

void debugPinMapping() {
  Serial.println("=== Machine pin mapping ===");
  for (uint8_t i = 0; i < numMachineOutputs; i++) {
    uint8_t pin = machineOutputPins[i];
    uint8_t funcIndex = machine.config.pinFunction[i];  // or [i-1] depending on your current code

    Serial.print("Index ");
    Serial.print(i);
    Serial.print(" -> pin ");
    Serial.print(pin);
    Serial.print(" uses function index ");
    Serial.print(funcIndex);
    Serial.print(" state=");
    Serial.println(machine.state.functions[funcIndex] ? "ON" : "OFF");
  }
}

// Reset PWM phase/timers (does NOT zero OnTime[] so you don't stomp duty calculations)
void resetPwmState() {
  for (uint8_t i = 0; i < maxNozzles; i++) {
    PwmTimer[i] = 0;
    PwmPhaseOffset[i] = 0;
    nozzleState[i] = LOW;
  }
}

// Call this to reset *everything* that should be “fresh” for a new spray run
void resetSprayRunState() {
  resetFlowAverages();  // your existing function
  resetPwmState();      // PWM timers/phase
}
