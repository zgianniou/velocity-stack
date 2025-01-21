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
#define LED_PIN 10
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
int32_t modeRanges[4][2] = {
  { 0, 3000 },     // Mode 1 range
  { 3000, 5000 },  // Mode 2 range
  { 5000, 8000 },  // Mode 3 range
  { 8000, 14000 }  // Mode 4 range
};

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
  for (int i = 0; i < 4; i++) {
    if (rpm >= modeRanges[i][0] && rpm < modeRanges[i][1]) {
      return i + 1;  // Mode 1 to Mode 4
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
  //it sends a default array value which is [1,2,3,4,3,2,1]
  //should change it when we make the mechanical parts
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

      // Ensure the array has at least 5 values (the 5 range points)
      if (ranges.size() == 5) {
        for (int i = 0; i < ranges.size(); i++) {
          float rangeValue = ranges[i];
          Serial.printf("Received range value: %.2f\n", rangeValue);

          // Assign values to modeRanges
          if (i < 4) {
            modeRanges[i][0] = rangeValue; // Store range in modeRanges
            modeRanges[i][1] = (i + 1 < ranges.size()) ? ranges[i + 1] : rangeValue;  // Set the next range as the upper bound
            Serial.printf("Updated Range for Mode %d: %.2f - %.2f\n", i + 1, modeRanges[i][0], modeRanges[i][1]);
          }
        }
      } else {
        Serial.println("Invalid number of range values");
        server.send(400, "text/plain", "Invalid number of range values");
        return;
      }
    }

    server.send(200, "text/plain", "Ranges updated successfully");
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
    for (int i = 0; i < 4; i++) {
      ranges.add(modeRanges[i][0]);
      if(i== 3){
        ranges.add(modeRanges[i][1]);
  
      }
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

void store_data_rpm(int32_t value[4][2]) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                char key[16];
                sprintf(key, "key_%d_%d", i, j);
                nvs_set_i32(handle, key, value[i][j]);
            }
        }
        if (err != ESP_OK) {
        Serial.println("Failed to commit NVS!");
    } else {
        Serial.println("Data stored successfully!");
    }
        nvs_commit(handle);
        nvs_close(handle);
    }
}

void read_data_rpm() {
  int32_t value[4][2];  // Declare the 2D array to store the values


    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &handle);
    if (err == ESP_OK) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                char key[16];
                sprintf(key, "key_%d_%d", i, j);
                nvs_get_i32(handle, key, &value[i][j]);
                if (err == ESP_OK) {
                  modeRanges[i][j] = value[i][j];
                Serial.print("Read key: ");
                Serial.print(key);
                Serial.print(" = ");
                Serial.println(value[i][j]);
            } else if (err == ESP_ERR_NVS_NOT_FOUND) {
                Serial.print("Key not found: ");
                Serial.println(key);
            } else {
                Serial.print("Error reading key: ");
                Serial.println(key);
            }
            }
            
        }
        nvs_close(handle);
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
  read_data_rpm();
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
        for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            Serial.print("modeRanges[");
            Serial.print(i);
            Serial.print("][");
            Serial.print(j);
            Serial.print("]: ");
            Serial.println(modeRanges[i][j]);
        }
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
    // rpm = random(0, 2000);  // Generate a random RPM value between 0 and 14000
    handleRPMData();

  delay(50);
}
