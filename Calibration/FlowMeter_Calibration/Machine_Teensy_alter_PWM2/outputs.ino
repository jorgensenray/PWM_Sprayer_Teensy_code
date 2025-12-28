// callback function triggered by Machine class to update 1-64 "section" outputs only
void updateSectionOutputs() {
  Serial.print("\r\n*** Section Outputs update! *** ");
}

// callback function triggered by Machine class to update "machine" outputs
// this updates the Machine Module Pin Configuration outputs
// - sections 1-16, Hyd Up/Down, Tramline Right/Left, Geo Stop
void updateMachineOutputs() {
  Serial.print("\r\nMachine Outputs update! ");

  numActiveNozzles = 0;  // Reset the count of active pins

  for (uint8_t i = 1; i < numMachineOutputs; i++) {

    PinNum = machineOutputPins[i];  // Store the pin number
    //isPinOn = machine.config.isPinActiveHigh
    
    //Print_output_Debug();

    //digitalWrite(machineOutputPins[i], machine.state.functions[machine.config.pinFunction[i]] == );  // ==, XOR

    delay(0);
  }
}

void setOutputPinModes() {
  if (numMachineOutputs > 0) {
    for (uint8_t i = 0; i < numMachineOutputs; i++) {
      pinMode(machineOutputPins[i], OUTPUT);
      digitalWrite(machineOutputPins[i], !machine.config.isPinActiveHigh);  // set OFF
    }
  }
}

void Print_output_Debug() {

  //gpsSpeed = ((machine.states.gpsSpeed * 0.1) * 0.621371);
  //leftSpeed = (((machine.states.leftSpeed * .01) * 3.28084) * 0.681818);
  //rightSpeed = (((machine.states.rightSpeed * .01) * 3.28084) * 0.681818);

  //Serial.print(" numActiveNozzles = ");
  //Serial.print(numActiveNozzles);
  //Serial.print(" PinNum = ");
  //Serial.print(PinNum);
  //Serial.print(" isPinOn = ");
  //Serial.println(isPinOn);
  //Serial.print(" gpsSpeed = ");
  //Serial.print(gpsSpeed);
  //Serial.print(" leftSpeed = ");
  //Serial.print(leftSpeed);
  //Serial.print(" rightSpeed = ");
  //Serial.print(rightSpeed);
  //Serial.print("  Actual Steer Angle: ");
  //Serial.println(steerAngle);

  /*
    Serial.print(" - Pin  ");
    //Serial.print("\r\n- Pin ");
    Serial.print((machineOutputPins[i] < 10 ? " " : ""));
    Serial.print(machineOutputPins[i]);
    Serial.print(":  ");
    Serial.print(machine.state.functions[machine.config.pinFunction[i - 1]]);
    Serial.print("  ");
    Serial.println(machine.functionNames[machine.config.pinFunction[i - 1]]);
    */
}