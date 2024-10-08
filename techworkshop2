#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
uint16_t irBuffer[100]; // infrared LED sensor data
uint16_t redBuffer[100];  // red LED sensor data
#else
uint32_t irBuffer[100]; // infrared LED sensor data
uint32_t redBuffer[100];  // red LED sensor data
#endif

int32_t bufferLength; // data length
int32_t spo2; // SPO2 value
int8_t validSPO2; // indicator to show if the SPO2 calculation is valid
int32_t heartRate; // heart rate value
int8_t validHeartRate; // indicator to show if the heart rate calculation is valid

byte pulseLED = 11; // Must be on PWM pin
byte readLED = 13;  // Blinks with each data read

int tempPin = A0; // CHANGED: Added TMP36 connected to A0 for temperature reading

void setup() {
  Serial.begin(9600); // initialize serial communication

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize the MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {  // Use default I2C port, 400kHz speed
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  // Configuration for the sensor
  byte ledBrightness = 60; // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;  // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;    // Options: 69, 118, 215, 411
  int adcRange = 4096;     // Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);  // Configure sensor
}

void loop() {
  bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps

  // Read the first 100 samples, and determine the signal range
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false)  // do we have new data?
      particleSensor.check(); // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to the next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  // Calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // CHANGED: Read temperature sensor and calculate temperature in Celsius
  int tempReading = analogRead(tempPin); // CHANGED: Read analog value from TMP36 sensor
  float voltage = tempReading * (5.0 / 1024.0); // CHANGED: Convert the analog reading to voltage
  float temperatureC = (voltage - 0.5) * 100.0; // CHANGED: Convert voltage to Celsius
  
  // Continuously taking samples from MAX30105. Heart rate and SpO2 are calculated every 1 second
  while (1) {
    // Dump the first 25 sets of samples and shift the last 75 sets to the top
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    // Take 25 sets of samples before calculating the heart rate
    for (byte i = 75; i < 100; i++) {
      while (particleSensor.available() == false) // do we have new data?
        particleSensor.check(); // Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); // Blink onboard LED with each data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); // We're finished with this sample so move to the next sample

      // Print heart rate, SpO2, and temperature
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);
      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);
      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);
      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);
      Serial.print(F(", SPO2Valid="));
      Serial.print(validSPO2, DEC);
      
      // CHANGED: Print temperature reading
      Serial.print(F(", Temp="));
      Serial.print(temperatureC); // CHANGED: Printing temperature in Celsius
      Serial.println("C");
    }

    // After gathering 25 new samples, recalculate HR and SPO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // CHANGED: Recalculate temperature every loop
    tempReading = analogRead(tempPin); // CHANGED: Read the temperature sensor again
    voltage = tempReading * (5.0 / 1024.0); // CHANGED: Convert analog reading to voltage again
    temperatureC = (voltage - 0.5) * 100.0; // CHANGED: Convert voltage to Celsius again
  }
}
