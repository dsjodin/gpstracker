// Configuration
#define SERVER_URL "http://yourserver.com/api/gps-data"
#define SIM7000G_TX_PIN 7
#define SIM7000G_RX_PIN 8
#define BAUD_RATE 9600
#define SLEEP_TIME_MS 60000  // Sleep for 60 seconds
#define MOVEMENT_THRESHOLD 10.0  // Movement threshold (in meters)
#define MIN_SATELLITES 4  // Minimum number of satellites for reliable GPS data
#define CHECK_INTERVAL_MS 10000  // Check cellular network connection every 10 seconds

// Instantiate SoftwareSerial and TinyGPSPlus
SoftwareSerial ss(SIM7000G_TX_PIN, SIM7000G_RX_PIN);
TinyGPSPlus gps;

// Variables to store the last known location
float lastLatitude = 0.0;
float lastLongitude = 0.0;

// Moving Average Filters
MovingAverage<float> latAvg(5);  // Window size of 5
MovingAverage<float> lonAvg(5);  // Window size of 5

void setup() {
  // Setup serial communication
  Serial.begin(BAUD_RATE);
  ss.begin(BAUD_RATE);

  // Setup SIM7000G module for GPS
  ss.println("AT+CGNSPWR=1");  // Turn on GNSS power supply
  delay(100);

  // Check cellular network connection
  ss.println("AT+CREG?");  // Check network registration status
  delay(100);
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      // Check if connected to at least MIN_SATELLITES
      if (gps.satellites.value() < MIN_SATELLITES) {
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
      if (distanceMoved > MOVEMENT_THRESHOLD) {
        // Send GPS data to the server
        Serial.println("Sending GPS data to the server...");

        // Update last known location
        lastLatitude = latitude;
        lastLongitude = longitude;
      }

      // Put SIM7000G into sleep mode
      ss.println("AT+CSCLK=1");
      delay(100);
      
      // Sleep for SLEEP_TIME_MS milliseconds before getting the next GPS reading
      delay(SLEEP_TIME_MS);
      
      // Wake up SIM7000G
      ss.println("AT+CSCLK=0");
      delay(100);
    }
  }

  // Check cellular network connection periodically
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime > CHECK_INTERVAL_MS) {
    ss.println("AT+CREG?");  // Check network registration status
    delay(100);
    lastCheckTime = millis();
  }
}
