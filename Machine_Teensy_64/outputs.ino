#include <Adafruit_MCP23X17.h>

// MCP23017 expanders are defined in Machine_Teensy_64.ino
extern Adafruit_MCP23X17 mcpA;  // sections 33-48
extern Adafruit_MCP23X17 mcpB;  // sections 49-64

// Local cache for downstream logic (0..63 == sections 1..64)
bool sectionState[64] = { false };

// Teensy pins for section outputs 1..32
// User mapping: pins 1..17, skip 18/19 (I2C), then 20..34
static const uint8_t sectionTeensyPins[32] = {
  1, 2, 3, 4, 5, 6, 7, 8,
  9, 10, 11, 12, 13, 14, 15, 16,
  17, 20, 21, 22, 23, 24, 25, 26,
  27, 28, 29, 30, 31, 32, 33, 34
};

inline void writeSectionPhysical(uint8_t sectionIdx, bool isOnLogical) {
  // Apply active-high/low setting
  bool isOnPhysical = machine.config.isPinActiveHigh ? isOnLogical : !isOnLogical;

  if (sectionIdx < 32) {
    digitalWrite(sectionTeensyPins[sectionIdx], isOnPhysical ? HIGH : LOW);
    if (debugPwmLevel == 3) {
      Serial.print("sectionIdx (Pin #) ");
      Serial.print(sectionIdx + 1);
      Serial.print(" logical ");
      Serial.print(isOnLogical);
      Serial.print(" physical ");
      Serial.println(isOnPhysical);
    }
  } else if (sectionIdx < 48) {
    mcpA.digitalWrite(sectionIdx - 32, isOnPhysical ? HIGH : LOW);
    if (debugPwmLevel == 3) {
      Serial.print("sectionIdx (Pin #) 33 - 48 ");
      Serial.print(sectionIdx) + 1;
      Serial.print(" logical ");
      Serial.print(isOnLogical);
      Serial.print(" physical ");
      Serial.println(isOnPhysical);
    }
  } else {
    mcpB.digitalWrite(sectionIdx - 48, isOnPhysical ? HIGH : LOW);
    if (debugPwmLevel == 3) {
      Serial.print("sectionIdx (Pin #) 49 - 64 ");
      Serial.print(sectionIdx + 1);
      Serial.print(" logical ");
      Serial.print(isOnLogical);
      Serial.print(" physical ");
      Serial.println(isOnPhysical);
    }
  }
}

void setSectionPinModes() {
  for (uint8_t i = 0; i < 32; i++) {
    pinMode(sectionTeensyPins[i], OUTPUT);
    digitalWrite(sectionTeensyPins[i], LOW);  // default OFF
  }
}

// callback function triggered by Machine class when PGN229 sections change.
// Single-writer design: this function ONLY updates cached section state.
// Actual hardware writes happen in PWM_Controls().
void updateSectionOutputs() {
  for (uint8_t i = 0; i < 64; i++) {
    bool on = (bool)machine.getSectionState(i);  // i=0..63
    sectionState[i] = on;
  }

  if (debugPwmLevel == 8) {
    Serial.print("\r\n*** Section Outputs update! sections=0x");
    Serial.print((uint32_t)(machine.state.sections.allSections >> 32), HEX);
    Serial.print((uint32_t)(machine.state.sections.allSections & 0xFFFFFFFF), HEX);
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
  }

  // optional debug
  if (debugPwmLevel == 10) {
    Serial.print("updatePinStates: pinStates.size=");
    Serial.print(pinStates.size());
    Serial.println();
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
