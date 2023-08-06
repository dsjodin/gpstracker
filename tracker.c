#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>

#define EEPROM_SIZE 512
#define CONFIG_START 0

// Config struct to store in EEPROM
struct Config {
  char serverUrl[128];
  int sim7000gTxPin;
  int sim7000gRxPin;
  int baudRate;
  unsigned long sleepTimeMs;
  float movementThreshold;
  int minSatellites;
  unsigned long checkIntervalMs;
};

Config config;

TinyGPSPlus gps;

// Variables to store the last known location
float lastLatitude = 0.0;
float lastLongitude = 0.0;

// Moving Average Filters
MovingAverage<float> latAvg(5);  // Window size of 5
MovingAverage<float> lonAvg(5);  // Window size of 5

AsyncWebServer server(80);

void setup() {
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Load config from EEPROM
  EEPROM.get(CONFIG_START, config);

  // Setup serial communication
  SoftwareSerial ss(config.sim7000gTxPin, config.sim7000gRxPin);
  ss.begin(config.baudRate);

  // Setup SIM7000G module for GPS
  ss.println("AT+CGNSPWR=1");
  delay(100);

  // Check cellular network connection
  ss.println("AT+CREG?");
  delay(100);

  // Setup the web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Send a form to enter new config values
    String html = "<form method='POST' action='/set-config'>";
    html += "Server URL: <input type='text' name='serverUrl' value='" + String(config.serverUrl) + "'><br>";
    html += "SIM7000G TX Pin: <input type='number' name='sim7000gTxPin' value='" + String(config.sim7000gTxPin) + "'><br>";
    html += "Sleep Time in MS: <input type='number' name='sleepTimeMs' value='" + String(config.sleepTimeMs) + "'><br>";
    html += "Movement Threshold: <input type='number' step='0.01' name='movementThreshold' value='" + String(config.movementThreshold) + "'><br>";
    html += "Minimum Satellites: <input type='number' name='minSatellites' value='" + String(config.minSatellites) + "'><br>";
    html += "Check Interval MS: <input type='number' name='checkIntervalMs' value='" + String(config.checkIntervalMs) + "'><br>";
    html += "<input type='submit' value='Set Config'>";
    html += "</form>";
    request->send(200, "text/html", html);
  });

  server.on("/set-config", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Update config with new values from the form
    String serverUrl = request->getParam("serverUrl", true)->value();
    strncpy(config.serverUrl, serverUrl.c_str(), sizeof(config.serverUrl));
    config.sim7000gTxPin = request->getParam("sim7000gTxPin", true)->value().toInt();
    config.sleepTimeMs = request->getParam("sleepTimeMs", true)->value().toInt();
    config.movementThreshold = request->getParam("movementThreshold", true)->value().toFloat();
    config.minSatellites = request->getParam("minSatellites", true)->value().toInt();
    config.checkIntervalMs = request->getParam("checkIntervalMs", true)->value().toInt();

    // Save config to EEPROM
    EEPROM.put(CONFIG_START, config);
    EEPROM.commit();

    // Redirect back to the form
    request->redirect("/");
  });

  server.begin();
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      // Check if connected to at least config.minSatellites
      if (gps.satellites.value() < config.minSatellites) {
        Serial.println("Not enough satellites, waiting for better signal...");
        continue;
      }

      // Get GPS data
      float latitude = gps.location.lat();
      float longitude = gps.location.lng();
      
      // Add raw data to moving average filters
      latAvg.add(latitude);
      lonAvg.add(longitude);
      
      // Get filtered location data
      latitude = latAvg.getAverage();
      longitude = lonAvg.getAverage();

      // Calculate distance moved
      float distanceMoved = TinyGPSPlus::distanceBetween(
        latitude, longitude, lastLatitude, lastLongitude
      );

      // Only send data if movement is greater than the threshold
      if (distanceMoved > config.movementThreshold) {
        // Send GPS data to the server
        Serial.println("Sending GPS data to the server...");

        // Update last known location
        lastLatitude = latitude;
        lastLongitude = longitude;
      }

      // Put SIM7000G into sleep mode
      ss.println("AT+CSCLK=1");
      delay(100);
      
      // Sleep for config.sleepTimeMs milliseconds before getting the next GPS reading
      delay(config.sleepTimeMs);
      
      // Wake up SIM7000G
      ss.println("AT+CSCLK=0");
      delay(100);
    }
  }

  // Check cellular network connection periodically
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime > config.checkIntervalMs) {
    ss.println("AT+CREG?");
    delay(100);
    lastCheckTime = millis();
  }
}
