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

// HTTP handler to send current RPM
void handlePosition() {
  String position = String(rpm);
  // Serial.println("Sending current position: " + position);
  server.send(200, "text/plain", position);
}

// HTTP handler to send RPM and mode together
void handleData() {
  int mode = determineMode(rpm);
  String jsonData = "{";
  jsonData += "\"rpm\":" + String(rpm) + ",";                 // Send RPM value
  jsonData += "\"mode\":" + String(mode) + ",";               // Send Mode value
  jsonData += "\"system_status\":" + String(functionStatus);  // Send system status
  jsonData += "}";

  // Serial.println("Sending RPM and mode: " + jsonData);
  server.send(200, "application/json", jsonData);
}

// HTTP handler to receive and process the JSON data
void handleSaveRanges() {
  if (server.method() == HTTP_POST) {
    String jsonData = server.arg("plain");              // Get raw JSON data from the request
    Serial.println("Received JSON data: " + jsonData);  // Log the received JSON data

    // Parse the JSON data
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
      Serial.println("Failed to parse JSON");
      server.send(400, "text/plain", "Invalid JSON");
      delay(1000);
      return;
    }

    // Example: Read values from JSON (you can expand this based on your actual JSON structure)
    JsonArray ranges = doc["ranges"].as<JsonArray>();  // Assuming JSON contains "ranges" as an array
    Serial.println("Received Ranges:");                // Log the received ranges

    // Validate and update mode ranges based on received JSON
    for (int i = 0; i < ranges.size() && i < 4; i++) {
      float start = ranges[i]["start"];
      float end = ranges[i]["end"];

      // Log the received start and end values
      Serial.print("Range ");
      Serial.print(i + 1);
      Serial.print(": Start = ");
      Serial.print(start);
      Serial.print(", End = ");
      Serial.println(end);

      if (start < end) {
        modeRanges[i][0] = start;
        modeRanges[i][1] = end;
        Serial.print("Updated Range for Mode ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(start);
        Serial.print(" - ");
        Serial.println(end);
      } else {
        Serial.println("Invalid range: start should be less than end.");
      }
    }
    store_data_rpm(modeRanges);
    server.send(200, "text/plain", "Ranges saved successfully");

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
  server.on("/position", HTTP_GET, handlePosition);        // Send RPM position only
  server.on("/data", HTTP_GET, handleData);                // Send both RPM and mode as JSON
  server.on("/save_ranges", HTTP_POST, handleSaveRanges);  // Route to save ranges
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
    handlePosition();

  delay(50);
}
