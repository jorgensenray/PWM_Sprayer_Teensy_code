
#include <elapsedMillis.h>
#include <movingAvg.h>  // https://github.com/JChristensen/movingAvg

const int PressurePin = A1;       // Connect the sensor output to A2
const float minPressure = 0.0;    // Minimum pressure the sensor can read
const float maxPressure = 100.0;  // Maximum pressure the sensor can read (adjust based on your sensor's range)
float pressure = 0;

elapsedMillis read_pressure;

movingAvg avgPSI(10);     // define the moving average object
movingAvg avgVoltage(5);  // define the moving average object

void setup() {
  Serial.begin(115200);
  pinMode(PressurePin, INPUT);
  avgPSI.begin();      // initialize the moving average
  avgVoltage.begin();  // initialize the moving average
}

void loop() {
  if (read_pressure >= 500) {
    float sensorvalue = analogRead(PressurePin);
    int sensorvalueAve = avgPSI.reading(sensorvalue);  // calculate the moving average
    //float sensorvalueAve = PSImovingAverage(sensorvalue);
    float voltage = sensorvalueAve * (5.0 / 1023.0);
    voltage -= .075;  // Adjustment factor - measure against multimeter
    //int voltageAvg = avgVoltage.reading(voltage);             // calculate the moving average
    pressure = ((voltage - 0.5) * (maxPressure - minPressure)) / (4.5 - 0.5) + minPressure;

    Serial.print(" Analog value: ");
    Serial.print(sensorvalue);
    Serial.print(" Analog valueAve: ");
    Serial.print(sensorvalue);
    Serial.print("  Voltage: ");
    Serial.print(voltage, 2);
    Serial.print("  Pressure: ");
    Serial.print(pressure);
    Serial.println(" PSI");

    read_pressure = 0;
  }
}
