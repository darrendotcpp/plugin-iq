// #include <Arduino.h>

// //Analog pin connected to ZMPT101B
// #define ZMPT101B_PIN 15 

// void setup() {

//   // Initialize Serial Monitor
//   Serial.begin(9600); 
//   analogReadResolution(12); // Set resolution to 12-bit

// }

// void loop() {

//   // Read raw value from ZMPT101B
//   int sensorValue = analogRead(ZMPT101B_PIN); 
  
//   // Convert to voltage (0-3.3V range)
//   float voltage = (sensorValue * 3.3) / 4095.0; 
  
//   // Print the voltage reading
//   Serial.print("Raw Value: ");
//   Serial.print(sensorValue);
//   Serial.print(" Voltage: ");
//   Serial.println(voltage, 3); 

//   // Wait for 1 second
//   delay(1000); 
// }

#include <Arduino.h>

#define ZMPT101B_PIN 4//Analog pin connected to ZMPT101B
#define SAMPLES 1000    // Number of samples for RMS calculation
#define VOLTAGE_REF 3.3 // ESP32 reference voltage
#define ADC_RES 4095    // ESP32 ADC resolution (12-bit)

// Calibration factor
float CALIBRATION_FACTOR = 1; // Example value from calculation above

void setup() {
  Serial.begin(9400);
  analogReadResolution(12); // Set ADC resolution to 12 bits
  Serial.println("ZMPT101B Calibration Example");
}

float readRMSVoltage() {
  unsigned long sum = 0;
  
  // Collect multiple samples for more accurate RMS calculation
  for (int i = 0; i < SAMPLES; i++) {
    int rawADC = analogRead(ZMPT101B_PIN); // Read raw ADC value
    float voltage = (rawADC * VOLTAGE_REF) / ADC_RES; // Convert ADC value to voltage (0-3.3V)
    sum += voltage * voltage; // Sum of squared values
  }

  // Calculate RMS voltage
  float mean = sum / SAMPLES;
  float rmsVoltage = sqrt(mean);
  
  // Apply calibration factor to scale to actual AC voltage
  return rmsVoltage * CALIBRATION_FACTOR;
}

void loop() {
  float rmsVoltage = readRMSVoltage();
  Serial.print("RMS Voltage: ");
  Serial.print(rmsVoltage);
  Serial.println(" V");
  delay(1000); // Wait for 1 second
}


