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

bool testCheck = false;  // Flag to determine if test_result should be sent
bool syncStatus = false; // Flag to determine if ranges from esp32 to app should be sent

// Default mode ranges (to be updated via JSON)
int32_t *modeRanges = nullptr;  // Dynamic array for mode ranges
int numRanges = 0;              // Number of ranges

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

// Determine mode based on RPM and dynamic modeRanges
int determineMode(float rpm) {
  for (int i = 0; i < numRanges - 1; i++) {
    if (rpm >= modeRanges[i] && rpm < modeRanges[i + 1]) {
      return i + 1;  // Mode 1 to Mode N
    }
  }
  return 0;  // Default case
}

// HTTP handler to send RPM data
void handleRPMData() {
  DynamicJsonDocument doc(256);

  // Send RPM value
  doc["rpm"] = rpm;

  String jsonData;
  serializeJson(doc, jsonData);
  Serial.println("Sending RPM data: " + jsonData);
  server.send(200, "application/json", jsonData);
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

// HTTP handler to update mode ranges
void handleRanges() {
  if (server.method() == HTTP_POST) {
    String jsonData = server.arg("plain");
    Serial.println("Received JSON data for ranges: " + jsonData);

    // Parse the JSON data
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
      Serial.println("Failed to parse JSON");
      server.send(400, "text/plain", "Invalid JSON");
      return;
    }

    // Update mode ranges if present
    if (doc.containsKey("ranges")) {
      JsonArray ranges = doc["ranges"].as<JsonArray>();

      // Ensure the first value is 0 and the last is 14000
      if (ranges[0] != 0 || ranges[ranges.size() - 1] != 14000) {
        Serial.println("Invalid ranges: First value must be 0 and last value must be 14000");
        server.send(400, "text/plain", "Invalid ranges: First value must be 0 and last value must be 14000");
        return;
      }

      // Free existing modeRanges if any
      if (modeRanges != nullptr) {
        free(modeRanges);
      }

      // Allocate memory for new ranges
      numRanges = ranges.size();
      modeRanges = (int32_t *)malloc(numRanges * sizeof(int32_t));

      // Copy ranges to modeRanges
      for (int i = 0; i < numRanges; i++) {
        modeRanges[i] = ranges[i];
      }

      // Store the ranges in NVS
      storeRanges();

      server.send(200, "text/plain", "Ranges updated successfully");
    }
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

// HTTP handler to send RPM ranges
void handleSendRanges() {
  if (server.method() == HTTP_GET) {
    DynamicJsonDocument doc(256);

    // Send RPM ranges
    JsonArray ranges = doc.createNestedArray("ranges");
    for (int i = 0; i < numRanges; i++) {
      ranges.add(modeRanges[i]);
    }

    String jsonData;
    serializeJson(doc, jsonData);
    Serial.println("Sending RPM ranges: " + jsonData);
    server.send(200, "application/json", jsonData);
  }
}

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    // Now NVS is ready for use
}

// Store ranges in NVS
void storeRanges() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        // Store the number of ranges
        nvs_set_i32(handle, "numRanges", numRanges);

        // Store each range value
        for (int i = 0; i < numRanges; i++) {
            char key[16];
            sprintf(key, "range_%d", i);
            nvs_set_i32(handle, key, modeRanges[i]);
        }

        nvs_commit(handle);
        nvs_close(handle);
        Serial.println("Ranges stored successfully!");
    } else {
        Serial.println("Failed to store ranges!");
    }
}

// Load ranges from NVS
void loadRanges() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &handle);
    if (err == ESP_OK) {
        // Read the number of ranges
        nvs_get_i32(handle, "numRanges", &numRanges);

        // Allocate memory for modeRanges
        modeRanges = (int32_t *)malloc(numRanges * sizeof(int32_t));

        // Read each range value
        for (int i = 0; i < numRanges; i++) {
            char key[16];
            sprintf(key, "range_%d", i);
            nvs_get_i32(handle, key, &modeRanges[i]);
        }

        nvs_close(handle);
        Serial.println("Ranges loaded successfully!");
    } else {
        Serial.println("Failed to load ranges!");
    }
}

void setup() {
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(interruptPin), onRise, RISING);
  pinMode(interruptPin, INPUT);

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

  // Configure NVS
  app_main();
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