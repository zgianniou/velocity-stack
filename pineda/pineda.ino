#include <WiFi.h>  // Use WiFi library for ESP32
#include <WebServer.h>
#include <EEPROM.h>
#include <ArduinoJson.h>  // Include the ArduinoJson library
#include "nvs_flash.h"
#include "nvs.h"

// Access Point credentials
const char* ssid = "ESP8266_AP";
const char* password = "12345678";

// Create a web server on port 80
WebServer server(80);

// Pin configuration
#define LED_PIN 8
const int analogPin = 0;      // ADC pin
const int interruptPin = 18;  // Digital GPIO pin for signal detection
const int buttonPin = 5;      // GPIO pin for the button

// Variables for RPM simulation
volatile unsigned long t1 = 0, t2 = 0;               // Time variables
volatile float period = 0.0, freq = 1000.0, rpm = 0.0;  // Period, frequency, and RPM variables
volatile bool risingEdgeDetected = false;

unsigned long lastPrintTime = 0;  // For periodic RPM updates
bool functionStatus = true;       // Toggle status (ON/OFF)
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Mode ranges storage
const int MAX_RANGES 10
const int INITIAL_RANGES = 4;
int32_t modeRanges[MAX_RANGES] = {0,3000, 7000,10000,14000};  // Default values
int numRanges = INITIAL_RANGES;

//bool testCheck = false;  
//bool syncStatus = false; 

// Interrupt Service Routine
void IRAM_ATTR onRise() {
  static unsigned long last_rise_time = 0;
  unsigned long current_time = micros();

  // Debounce: Ignore edges that occur within 300 microseconds
  if (current_time - last_rise_time > period/2) {
    last_rise_time = current_time;

    if (risingEdgeDetected) {
      t2 = current_time;
      if (t1 < t2) {
        period = t2 - t1;  // Calculate period
      }
      risingEdgeDetected = false;
    } else {
      t1 = current_time;
      risingEdgeDetected = true;
    }
  }
}

int determineMode(float rpm) {
    for (int i = 0; i < numRanges - 1; i++) {
        if (rpm >= modeRanges[i] && rpm < modeRanges[i + 1]) return i + 1;
    }
    return 0;
}

void handleRPMData() {
    DynamicJsonDocument doc(256);
    doc["rpm"] = rpm;
    String jsonData;
    serializeJson(doc, jsonData);
    server.send(200, "application/json", jsonData);
}


void handleRanges() {
    if (server.method() == HTTP_POST) {
        String jsonData = server.arg("plain");
        DynamicJsonDocument doc(512);
        if (deserializeJson(doc, jsonData)) {
            server.send(400, "text/plain", "Invalid JSON");
            return;
        }
        if (doc.containsKey("ranges")) {
            JsonArray ranges = doc["ranges"].as<JsonArray>();
            if (ranges.size() < 4 || ranges.size() > MAX_RANGES) {
                server.send(400, "text/plain", "Invalid range size (min 4, max 12)");
                return;
            }
            numRanges = ranges.size();
            modeRanges[0] = 0;
            modeRanges[numRanges - 1] = 14000;
            for (int i = 1; i < numRanges - 1; i++) modeRanges[i] = ranges[i];
            storeRanges();
            server.send(200, "text/plain", "Ranges updated successfully");
        }
    }
}


void handleSendRanges() {
    if (server.method() == HTTP_GET) {
        DynamicJsonDocument doc(256);
        JsonArray ranges = doc.createNestedArray("ranges");
        for (int i = 0; i < numRanges; i++) {
            ranges.add(modeRanges[i]);
        }
        String jsonData;
        serializeJson(doc, jsonData);
        server.send(200, "application/json", jsonData);
    }
}

void storeRanges() {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i32(handle, "numRanges", numRanges);
        for (int i = 0; i < numRanges; i++) {
            char key[16];
            sprintf(key, "range_%d", i);
            nvs_set_i32(handle, key, modeRanges[i]);
        }
        nvs_commit(handle);
        nvs_close(handle);
    }
}

void loadRanges() {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_i32(handle, "numRanges", &numRanges);
        for (int i = 0; i < numRanges; i++) {
            char key[16];
            sprintf(key, "range_%d", i);
            nvs_get_i32(handle, key, &modeRanges[i]);
        }
        nvs_close(handle);
    }
}


// HTTP handler to send test result (if test_check is true)
void handleTestResult() {
  DynamicJsonDocument doc(256);

  // Sending mode_path as an array
  JsonArray modePath = doc.createNestedArray("mode_path");
  modePath.add(1);
  modePath.add(2);
  modePath.add(3);
  modePath.add(4);
  modePath.add(3);
  modePath.add(2);
  modePath.add(1);

  String jsonData;
  serializeJson(doc, jsonData);
  Serial.println("Sending mode path: " + jsonData);
  server.send(200, "application/json", jsonData);
}

// HTTP handler to get a boolean value called "test_check"
void handleTestCheck() {
  if (server.method() == HTTP_POST) {
    String jsonData = server.arg("plain");
    Serial.println("Received JSON data for test_check: " + jsonData);

    // Parse the JSON data
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
      Serial.println("Failed to parse JSON");
      server.send(400, "text/plain", "Invalid JSON");
      return;
    }

    // Check for test_check and update the flag
    if (doc.containsKey("test_check")) {
      testCheck = doc["test_check"];
      Serial.print("Received test_check: ");
      Serial.println(testCheck);
    }

    server.send(200, "text/plain", "Test check updated successfully");
  }
}

// HTTP handler to sync status (boolean)
void handleSync() {
  if (server.method() == HTTP_POST) {
    DynamicJsonDocument doc(256);

    // Send sync status as a boolean
    doc["sync"] = syncStatus;

    String jsonData;
    serializeJson(doc, jsonData);
    Serial.println("Sending sync status: " + jsonData);
    server.send(200, "application/json", jsonData);
  }
}

void setup() {
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(interruptPin), onRise, RISING);
  pinMode(interruptPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // Set up Wi-Fi Access Point
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);
  WiFi.setSleep(WIFI_PS_NONE);  // Disable WiFi sleep
  IPAddress IP = WiFi.softAPIP();
  Serial.println("AP IP address: " + IP.toString());

  // Configure HTTP routes
  server.on("/data", HTTP_GET, handleRPMData);          // Send RPM data
  server.on("/test_result", HTTP_GET, handleTestResult);  // Send Test Results
  server.on("/save_test_check", HTTP_POST, handleTestCheck); // Receive test_check flag as boolean value
  server.on("/save_ranges", HTTP_POST, handleRanges); // Receive ranges for modes
  server.on("/sync", HTTP_POST, handleSync); // Send sync status (boolean)
  server.on("/ranges", HTTP_GET, handleSendRanges); // Send RPM ranges
  server.begin();
  Serial.println("HTTP server started");

  // Configure pins and interrupts
  pinMode(buttonPin, INPUT_PULLUP);

  // Configure NVS storage
  nvs_flash_init();
  loadRanges();
}

void loop() {
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(buttonPin);

  // Debounce button input
  if (currentButtonState != lastButtonState) {
    if (millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();
      if (currentButtonState == LOW) {
        functionStatus = !functionStatus;
        Serial.print("Button pressed - Current status: ");
        Serial.println(functionStatus ? "ON" : "OFF");
        for (int i = 0; i < numRanges; i++) {
            Serial.print("modeRanges[");
            Serial.print(i);
            Serial.print("]: ");
            Serial.println(modeRanges[i]);
        }
      }
    }
  }
  lastButtonState = currentButtonState;

  // Handle HTTP requests
  server.handleClient();

  // Generate random RPM between 0 and 14000
  if (functionStatus) {
    if (period > 0 && (millis() - lastPrintTime >= 100)) {  // Print every 500 ms
      lastPrintTime = millis();
      freq = 1000000.0 / period ;  // Frequency in Hz
      rpm = (freq * 120) / 36;  // Calculate RPM

      Serial.print("Frequency: ");
      Serial.println(freq);
      Serial.print("RPM: ");
      Serial.println(rpm);

      int mode = determineMode(rpm);
      Serial.print("Mode: ");
      Serial.println(mode);
    }
  }

  delay(50);
}