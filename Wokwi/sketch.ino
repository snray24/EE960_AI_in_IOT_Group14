/*
  Sita's Smart Home Sensor Simulation
  
  UPDATED VERSION using the DS18B20 Digital Temperature Sensor.
  
  This code reads data from a DS18B20, a PIR motion sensor,
  and an HC-SR04 ultrasonic sensor. It displays the data on the Serial
  Monitor and a 16x2 I2C LCD. It also controls LEDs based on sensor thresholds.
*/

// --- Libraries ---
#include <Wire.h>                 // For I2C
#include <LiquidCrystal_I2C.h>    // For I2C LCD
#include <OneWire.h>              // For DS18B20
#include <DallasTemperature.h>    // For DS18B20

// --- Pin Definitions ---
// DS18B20 Temperature Sensor
const int ONE_WIRE_BUS_PIN = 3;  // Data pin for DS18B20

// PIR Motion Sensor
const int PIR_PIN = 2;

// Ultrasonic Sensor
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// LEDs
const int LED_TEMP = 4;
const int LED_PIR = 5;
const int LED_DIST = 6;

// --- Sensor Thresholds ---
const float TEMP_THRESHOLD = 28.0;   // Threshold in Celsius for Temp LED
const float DISTANCE_THRESHOLD = 20.0; // Threshold in cm for Distance LED

// --- Object Initialization ---
// Initialize the LCD (0x27 is a common I2C address)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Setup a OneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS_PIN);

// Pass our OneWire reference to Dallas Temperature
DallasTemperature sensors(&oneWire);

// --- Global Variables ---
// For Temperature
float temperatureC = 0;

// For Ultrasonic
long duration;
float distanceCm = 0;

// For PIR Motion Sensor
bool isMotionDetected = false;
unsigned long motionStartTime = 0;
unsigned long inhibitEndTime = 0;
const long motionHighTime = 5000;
const long inhibitTime = 1200;

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);

  // --- ADD YOUR GITHUB INFO HERE ---
  Serial.println("===================================");
  Serial.println("Project by: aashish3808");
  Serial.println("GitHub: https://github.com/aashish3808");
  Serial.println("===================================");
  Serial.println(""); // Add a blank line

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Home Simulator");

  // Initialize DS18B20 sensor
  sensors.begin();

  // Set pin modes
  // (DS18B20 pin is handled by the libraries)
  pinMode(PIR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_TEMP, OUTPUT);
  pinMode(LED_PIR, OUTPUT);
  pinMode(LED_DIST, OUTPUT);

  // Set initial LED states to OFF
  digitalWrite(LED_TEMP, LOW);
  digitalWrite(LED_PIR, LOW);
  digitalWrite(LED_DIST, LOW);

  delay(1000); // Wait for sensors to stabilize
  lcd.clear();
}

void loop() {
  // Read from all sensors
  readTemperature();
  handlePIR();
  readDistance();

  // Update displays (Serial Monitor and LCD)
  updateDisplays();

  // Check thresholds and control LEDs
  checkThresholds();

  // Short delay
  delay(100);
}

// --- Sensor Reading Functions ---

void readTemperature() {
  // 1. Send the command to get temperatures
  sensors.requestTemperatures(); 
  
  // 2. Get the temperature in Celsius from the first sensor on the bus
  temperatureC = sensors.getTempCByIndex(0);

  // 3. Check if the reading is valid
  if (temperatureC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature");
    temperatureC = 0.0; // Set to 0 if error
  }
}

void readDistance() {
  // 1. Clear the TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // 2. Set the TRIG_PIN high for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 3. Read the ECHO_PIN
  duration = pulseIn(ECHO_PIN, HIGH);

  // 4. Calculate the distance
  distanceCm = duration * 0.0343 / 2.0;
}

void handlePIR() {
  int pirReading = digitalRead(PIR_PIN);
  unsigned long now = millis();

  // 1. Check for a new motion trigger
  if (pirReading == HIGH && !isMotionDetected && now > inhibitEndTime) {
    isMotionDetected = true;
    motionStartTime = now;
    digitalWrite(LED_PIR, HIGH);
    Serial.println("--- Motion Detected! ---");
  }

  // 2. Check if the 5-second "active" period is over
  if (isMotionDetected && (now - motionStartTime > motionHighTime)) {
    isMotionDetected = false;
    digitalWrite(LED_PIR, LOW);
    inhibitEndTime = now + inhibitTime;
    Serial.println("--- Motion ended. Inhibit 1.2s ---");
  }
}

// --- Display & Alert Functions ---

void updateDisplays() {
  // --- Update Serial Monitor ---
  Serial.print("Temp: ");
  Serial.print(temperatureC);
  Serial.print(" C | Dist: ");
  Serial.print(distanceCm);
  Serial.print(" cm | Motion: ");
  Serial.println(isMotionDetected ? "YES" : "NO");

  // --- Update LCD ---
  // Line 0: Temperature and Motion
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperatureC, 1); // 1 decimal place
  lcd.print("C ");

  lcd.setCursor(9, 0);
  lcd.print("Mot:");
  lcd.print(isMotionDetected ? "YES" : "NO ");

  // Line 1: Distance
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(distanceCm, 1); // 1 decimal place
  lcd.print(" cm   "); 
}

void checkThresholds() {
  // Control Temperature LED
  if (temperatureC > TEMP_THRESHOLD) {
    digitalWrite(LED_TEMP, HIGH);
  } else {
    digitalWrite(LED_TEMP, LOW);
  }

  // Control Distance LED
  if (distanceCm < DISTANCE_THRESHOLD && distanceCm > 0) {
    digitalWrite(LED_DIST, HIGH);
  } else {
    digitalWrite(LED_DIST, LOW);
  }
}