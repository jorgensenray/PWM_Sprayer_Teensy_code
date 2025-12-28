
// Enter MPH and nozzle width we'll do the rest
float MPH = 4;
float SprayWidth = 36;
#define FlowMeterPulsePerSecondPerLiter 6.9  //change this # to effect output (6.85)

byte sensorInterrupt = 0;  // 0 = digital pin 2
#define relay 1


volatile float FlowCalculationDuration = 10000;
float FlowCalculationStartTime;
volatile float PulseCount;
float LPM;
int LPMdecimal;
float GPM = 0;  //.34
float GPA = 0;  //12.6
float Sample_Weight = 0;

void setup() {
  PulseCount = 0;
  Serial.begin(115200);
  LPM = 0;
  LPMdecimal = 2;

  attachInterrupt(sensorInterrupt, pulseCounter2, CHANGE);
  FlowCalculationStartTime = millis();
  pinMode(relay, OUTPUT);

  //open the solenoid for the test duration to get timed sample to weigh.
  digitalWrite(relay, HIGH);
  delay(FlowCalculationDuration);
  digitalWrite(relay, LOW);
  delay(FlowCalculationDuration);
}

void loop() {
  digitalWrite(relay, HIGH);
  CalculateFlow2();
}

void CalculateFlow2() {
  if (millis() - FlowCalculationStartTime > FlowCalculationDuration) {
    LPM = PulseCount / (2 * FlowMeterPulsePerSecondPerLiter * (FlowCalculationDuration / 1000));
    GPM = LPM / 3.785;  //Gals per minute
    GPA = ((GPM * 5940) / (MPH * SprayWidth));
    Sample_Weight = (((GPM * 128) / 60) * (FlowCalculationDuration / 1000));
    
    DisplayLPM();
    FlowCalculationStartTime = millis();
    PulseCount = 0;
  }
}

void pulseCounter2() {
  PulseCount++;
}

void DisplayLPM() {
  Serial.print("    Pulse ");
  Serial.print(PulseCount);
  Serial.print("    GPM ");
  Serial.print(GPM);
  Serial.print("    GPA ");
  Serial.print(GPA);
  Serial.print("    Sample Weight OZ ");
  Serial.println(Sample_Weight);
}