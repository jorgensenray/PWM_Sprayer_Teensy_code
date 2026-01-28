// callback function triggered by Machine class to update 1-64 "section" outputs only
void updateSectionOutputs() {
  if (debugPwmLevel == 8) {
    Serial.print("\r\n*** Section Outputs update! *** ");
  }
}

// callback function triggered by Machine class to update "machine" outputs
// this updates the Machine Module Pin Configuration outputs
// - sections 1-16, Hyd Up/Down, Tramline Right/Left, Geo Stop
void updateMachineOutputs() {
  //Serial.print("\r\nMachine Outputs update! ");

  updatePinStates();
  //setupNozzles();

  for (uint8_t i = 1; i < numMachineOutputs; i++) {
    /*
    Serial.print("\r\n- Pin ");
    Serial.print((machineOutputPins[i] < 10 ? " " : ""));
    Serial.print(machineOutputPins[i]); Serial.print(": ");
    Serial.print(machine.state.functions[machine.config.pinFunction[i - 1]]);
    Serial.print(" ");
    Serial.print(machine.functionNames[machine.config.pinFunction[i - 1]]);
    */

    //digitalWrite(machineOutputPins[i], machine.state.functions[machine.config.pinFunction[i]] == machine.config.isPinActiveHigh);  // ==, XOR
  }
}

void updatePinStates() {
  pinStates.clear();
  numActiveNozzles = 0;

  for (uint8_t i = 0; i < numMachineOutputs; i++) {

    uint8_t pinNumLocal = machineOutputPins[i];
    uint8_t fn = machine.config.pinFunction[i];

    bool isOnLocal = false;

    // Only read machine.state.functions[] when fn is valid
    if (fn != 0 && fn != 255) {
      isOnLocal = (bool)machine.state.functions[fn];
    }

    // Store BOTH the pin and the function id
    pinStates.push_back({ pinNumLocal, fn, isOnLocal });

    // Count only spray sections S01..S16 (fn 1..16)
    if (isOnLocal && fn >= 1 && fn <= 16) {
      numActiveNozzles++;
    }
  }

  // optional debug
  if (debugPwmLevel == 10) {
    Serial.print("updatePinStates: pinStates.size=");
    Serial.print(pinStates.size());
    Serial.print(" numActiveNozzles=");
    Serial.println(numActiveNozzles);
  }
}






void setOutputPinModes() {
  if (numMachineOutputs > 0) {
    for (uint8_t i = 0; i < numMachineOutputs; i++) {
      pinMode(machineOutputPins[i], OUTPUT);
      digitalWrite(machineOutputPins[i], LOW);  // set OFF
    }
  }
}
