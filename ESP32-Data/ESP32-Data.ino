#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

// AWS IoT Core certificates
const char* AWS_IOT_ENDPOINT = "something that should not be shown publicly-ats.iot.eu-west-1.amazonaws.com";
const char* AWS_IOT_CERT = R"EOF(
-----BEGIN CERTIFICATE-----
// something that should not be shown publicly
-----END CERTIFICATE-----
)EOF";

const char* AWS_IOT_PRIVATE_KEY = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
// something that should not be shown publicly
-----END RSA PRIVATE KEY-----
)EOF";

const char* AWS_IOT_ROOT_CA = R"EOF(
-----BEGIN CERTIFICATE-----
// something that should not be shown publicly
-----END CERTIFICATE-----
)EOF";

// WiFi settings
const char* WIFI_SSID = "TECNO";
const char* WIFI_PASSWORD = "abed2003";

// AWS IoT topics (these are just test topics)
const char* AWS_IOT_TOPIC_PUBLISH = "nakhnoukh/data2"; //1
const char* AWS_IOT_TOPIC_COMMAND = "nakhnoukh/command2";         //2    the topics in 1 and 2 are temporarly switched just for debugging purposes to be able to send a command from aws to the control board to test it
const char* AWS_IOT_TOPIC_CONFIG = "nakhnoukh/config";

// Device identification
const String USER_ID = "user2"; // Change to "user2" for the second prosumer

#define EEPROM_SIZE 512
#define STATE_MAGIC 0xA5A5A5A5

// Sensor calibration constants
const int ADC_SAMPLES = 150;                // Number of samples to average for each reading
const int ADC_OFFSET = 115;                 // Offset calibration for current sensors
const float VOLTAGE_SCALE = 0.0041;         // Voltage calibration scale factor
const float VOLTAGE_OFFSET = 0.57;          // Voltage calibration offset
const int MEASUREMENT_INTERVAL_MS = 200;    // Minimum time between measurements (ms)
const int STATE_SAVE_INTERVAL_MS = 60000;   // Save state every 60 seconds

struct State {
  uint32_t magic;               // =STATE_MAGIC if initialized
  char     agreementType[8];    // "sell" or "buy"
  char     agreementId[32];
  float    energy_amount;         // kWh total to trade
  float    energySoldAccumulated;     // kWh sold so far
  bool     sent25, sent50, sent75, sent100;
  float    currentSoC;          // last SoC (%)
  float    accumulatedCharge;   // Ah coulomb counter
  time_t   lastHourlyReportTime;
} state;

// Connection status
bool mqttConnected = false;
bool timeIsSynchronized = false;

// Timing constants
const unsigned long HOUR_IN_MILLIS = 3600000; // 1 hour in milliseconds
const unsigned long RECONNECT_INTERVAL = 5000; // 5 seconds

// Timing variables
unsigned long lastHourlyReportTime = 0;
unsigned long lastReconnectAttempt = 0;
unsigned long lastStateSaveTime = 0;        // Track last state save time
int currentHour = -1;

// NTP Server settings
const char* NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 10800; // UTC+3 (3 hours * 3600 seconds)
const int DAYLIGHT_OFFSET_SEC = 0;

// --------- GLOBAL VARIABLES ---------
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

#define BATTERY_VOLTAGE_PIN 36    // ADC pin for battery voltage measurement
#define BATTERY_IN_CURRENT_PIN 34 // Current between charge controller and battery
#define LOAD_CURRENT_PIN 35       // Current between battery and load
#define OUTGOING_CURRENT_PIN 32   // Current between step-up converter and other prosumer
#define SOURCE_CURRENT_PIN 33     // Current sensor to check power source (solar or prosumer)

// SoC tracking variables
float currentSoC = 87.5;        // Initial SoC (%)  
float accumulatedCharge = currentSoC * 26;  // Coulomb counter (Ah)

//====================================
// --- Variables for Energy Integration ---
float energyInAccumulated = 0.0;   // in watt-hours (Wh)
float energyOutAccumulated = 0.0;  // in watt-hours (Wh)
float energySoldAccumulated = 0.0;  // in watt-hours (Wh)
unsigned long lastEnergyUpdate = 0; // for energy integration timing
//====================================

const float VOLTAGE_DIVIDER_RATIO = 20.40/(82.4+20.4);  // Example: 20k/(80k+20k) for a 5:1 divider
const float ADC_REFERENCE = 3.3;          // Reference voltage of ESP32 ADC
const float ADC_RESOLUTION = 4096.0;      // 12-bit ADC
const float CURRENT_SENSOR_SENSITIVITY = 0.1*3.3/5; // V/A for ACS712 20A module (100mV/A)
const float CURRENT_SENSOR_OFFSET = 1.65;     // Offset voltage (when current is 0)

// Function declarations
void setupWiFi();
void setupAWS();
void setupTime();
void sendHourlyReport();
bool reconnect();
String getCurrentTimeString();
bool hourHasChanged();
float readBatteryVoltage();
float readCurrent(int pin);
//====================================
void updateEnergyMeasurements();
float getEnergyIn();
float getEnergyOut();
//====================================
void loadState();
void saveState();
void publishMilestone(int pct);

void mqttCallback(char* topic, byte* payload, unsigned int length);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  // Restore persistent state
  EEPROM.begin(EEPROM_SIZE);
  loadState();
  Serial.printf("Loaded state: agreementId='%s', sold=%.2f/%.2f kWh\n",
                state.agreementId, state.energySoldAccumulated, state.energy_amount);

  Serial.println("\n\nESP32 Hourly Energy Report System Starting...");
  Serial.println("User ID: " + USER_ID);
  
  // Setup connections in the correct order
  setupWiFi();
  setupTime();
  setupAWS();  
  // Initialize timing
  lastHourlyReportTime = millis();
  lastEnergyUpdate = millis();
  lastStateSaveTime = millis();
  
  Serial.println("Setup complete, starting main loop");
}

void loop() {
  unsigned long currentMillis = millis();

  //====================================
  // Update energy measurements only when enough time has passed
  if (currentMillis - lastEnergyUpdate >= MEASUREMENT_INTERVAL_MS) {
    updateEnergyMeasurements();
  }
  //====================================
  
  // Check and maintain connection
  if (!mqttClient.connected()) {
    // Try to reconnect every 5 seconds
    if (currentMillis - lastReconnectAttempt > RECONNECT_INTERVAL) {
      lastReconnectAttempt = currentMillis;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected, process MQTT messages
    mqttClient.loop();
    
    // Check if it's time for hourly report
    if (hourHasChanged() || (currentMillis - lastHourlyReportTime > HOUR_IN_MILLIS)) {
      sendHourlyReport();
      lastHourlyReportTime = currentMillis;
    }
  }
  // float voltage = readBatteryVoltage();
  // Serial.print("Voltage:");
  // Serial.println(voltage);
  // Serial.println(readCurrent(36));
}

void setupWiFi() {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int connectionAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connectionAttempts < 20) {
    delay(500);
    Serial.print(".");
    connectionAttempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi. Will retry in background.");
  }
}

void setupTime() {
  // Configure time with NTP - THIS IS CRITICAL FOR TLS/SSL
  Serial.println("Setting up time synchronization (UTC+3)...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  
  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  int attempts = 0;
  while (now < 24 * 3600 * 2 && attempts < 100) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
    attempts++;
  }
  Serial.println("");
  
  if (now > 8 * 3600 * 2) {
    struct tm timeinfo;
    localtime_r(&now, &timeinfo); // Use localtime_r instead of gmtime_r to apply the timezone offset
    Serial.print("Current time (UTC+3): ");
    Serial.print(asctime(&timeinfo));
    timeIsSynchronized = true;
    
    // Initialize current hour
    currentHour = timeinfo.tm_hour;
  } else {
    Serial.println("Failed to get time sync - this may cause TLS connection issues!");
  }
}

void setupAWS() {
  // Configure AWS certificates
  wifiClient.setCACert(AWS_IOT_ROOT_CA);
  wifiClient.setCertificate(AWS_IOT_CERT);
  wifiClient.setPrivateKey(AWS_IOT_PRIVATE_KEY);
  
  // Configure MQTT client
  mqttClient.setServer(AWS_IOT_ENDPOINT, 8883);
  mqttClient.setCallback(mqttCallback); // Set the callback function for receiving messages
  mqttClient.setBufferSize(512);
  mqttClient.setSocketTimeout(30);
  
  Serial.println("AWS IoT configuration complete");
}

void loadState() {
  EEPROM.get(0, state);
  if (state.magic != STATE_MAGIC) {
    // first run: zero everything
    memset(&state, 0, sizeof(state));
    state.magic = STATE_MAGIC;
    saveState();
  }
}

void saveState() {
  // Don't call EEPROM.begin() here as it's called in setup()
  EEPROM.put(0, state);
  EEPROM.commit();
  // Don't call EEPROM.end() here as we'll need EEPROM again soon
}

// MQTT callback function for receiving messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received [");
  Serial.print(topic);
  Serial.print("]: ");
  
  // Convert payload to string
  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
    Serial.print((char)payload[i]);
  }
  message[length] = '\0';
  Serial.println();
  
  // Process the message - parse JSON if needed
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  const char* type = doc["type"];
  float energy_amount = doc["energy_amount"];
  const char* agreement_id = doc["agreement_id"];
  
  Serial.print("Parsed trade - Type: ");
  Serial.print(type);
  Serial.print(", Energy: ");
  Serial.print(energy_amount);
  Serial.print(" KWH, Agreement ID: ");
  Serial.println(agreement_id);
  
    // **First‐message of a trade**:
  if (strcmp(type, "sell")==0 || strcmp(type, "buy")==0) {
    // new agreement?
    if (strcmp(state.agreementId, agreement_id) != 0) {
      strncpy(state.agreementId, agreement_id, sizeof(state.agreementId)-1);
      // reset trade counters & flags
      state.energy_amount      = energy_amount;
      state.energySoldAccumulated  = 0.0;
      state.sent25 = state.sent50 = state.sent75 = state.sent100 = false;
      Serial.printf("Starting %s trade '%s' of %.2f kWh\n",
                    type, agreement_id, energy_amount);
    }
    strncpy(state.agreementType, type, sizeof(state.agreementType)-1);
    saveState();
    lastStateSaveTime = millis(); // Reset the save timer after explicit save
  }

  // // Example: Process commands based on message content
  // if (doc.containsKey("command")) {
  //   String command = doc["command"].as<String>();
  //   Serial.print("Command received: ");
  //   Serial.println(command);
    
  //   // Handle different commands
  //   if (command == "send_report") {
  //     Serial.println("Sending report on demand");
  //     sendHourlyReport();
  //   }
  //   else if (command == "status") {
  //     // Send a status report
  //     // You can implement additional status reporting here
  //     Serial.println("Status: active");
  //   }
    
    // Add more command handlers as needed
  // }
}

void sendHourlyReport() {
  if (!mqttClient.connected()) {
    Serial.println("MQTT not connected. Cannot send hourly report.");
    return;
  }
  
  // Generate random dummy values within reasonable ranges
  float hourlyEnergyConsumption = getEnergyOut(); // 50-200 Wh
  float hourlyEnergyProduction = getEnergyIn();  // 70-300 Wh
  float soc = currentSoC;                      // 30-95%
  
  // Create simple JSON document with only the 4 requested values
  StaticJsonDocument<256> doc;
  
  //these 3 lines below are added just temporarily for debugging purposes to try sending a specific message to another device through aws
  // doc["type"] = "sell";
  // doc["energy_amount"] = 5;
  // doc["agreement_id"] = "3ammo_talal";

  //These 4 lines below are commented out just temporarily for debugging purposes to try sending a specific message to another device through aws
  doc["hourly_energy_consumption"] = hourlyEnergyConsumption;
  doc["hourly_energy_production"] = hourlyEnergyProduction;
  doc["currentTime"] = getCurrentTimeString();
  doc["SoC"] = soc;
  
  // Serialize JSON to string
  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);
  
  // Send to AWS IoT
  Serial.print("Sending hourly report to topic: ");
  Serial.println(AWS_IOT_TOPIC_PUBLISH);
  
  if (mqttClient.publish(AWS_IOT_TOPIC_PUBLISH, jsonBuffer)) {
    Serial.println("Hourly report sent successfully:");
    serializeJsonPretty(doc, Serial);
    Serial.println();
  } else {
    Serial.print("Failed to send hourly report, error: ");
    Serial.println(mqttClient.state());
  }
}

bool reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, cannot attempt MQTT connection");
    return false;
  }
  
  Serial.println("Attempting AWS IoT connection...");
  
  // Create a client ID based on user ID and a random number
  String clientId = USER_ID + "-" + String(random(0xffff), HEX);
  
  // Attempt to connect
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("Connected to AWS IoT");
    mqttConnected = true;
    
    // Subscribe to topics - only do this once
    if (mqttClient.subscribe(AWS_IOT_TOPIC_COMMAND)) {
      Serial.print("Subscribed to topic: ");
      Serial.println(AWS_IOT_TOPIC_COMMAND);
    } else {
      Serial.println("Failed to subscribe to command topic");
    }
    
    if (mqttClient.subscribe(AWS_IOT_TOPIC_CONFIG)) {
      Serial.print("Subscribed to topic: ");
      Serial.println(AWS_IOT_TOPIC_CONFIG);
    } else {
      Serial.println("Failed to subscribe to config topic");
    }
    
    // Send an initial report
    sendHourlyReport();
    
    return true;
  } else {
    int state = mqttClient.state();
    Serial.print("AWS IoT connection failed, rc=");
    Serial.println(state);
    
    mqttConnected = false;
    return false;
  }
}

String getCurrentTimeString() {
  if (!timeIsSynchronized) {
    return "time-not-synced";
  }
  
  time_t now;
  struct tm timeinfo;
  char buffer[20];
  
  time(&now);
  localtime_r(&now, &timeinfo); // Use localtime_r to apply the timezone offset
  
  // Format: YYYY-MM-DD HH
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H", &timeinfo);
  
  return String(buffer);
}

bool hourHasChanged() {
  if (!timeIsSynchronized) return false;
  
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo); // Use localtime_r to apply the timezone offset
  
  if (timeinfo.tm_hour != currentHour) {
    currentHour = timeinfo.tm_hour;
    return true;
  }
  
  return false;
}

float readBatteryVoltage() {
  long total = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    total += analogRead(BATTERY_VOLTAGE_PIN);
    delayMicroseconds(100);
  }
  // Serial.print("Bits:");
  // Serial.print((float)total / ADC_SAMPLES);
  
  // Use defined calibration constants instead of magic numbers
  float voltage = ((float)total / ADC_SAMPLES) * VOLTAGE_SCALE + VOLTAGE_OFFSET;
  return voltage;
}

float readCurrent(int pin) {
  long total = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    total += analogRead(pin) + ADC_OFFSET;
    delayMicroseconds(100);
  }
  float voltage = ((float)total / ADC_SAMPLES / ADC_RESOLUTION) * ADC_REFERENCE;
  // Convert voltage difference to current using the calibrated sensitivity
  float current = (voltage - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY;
  return current;
}

// ======================
// NEW FUNCTIONS FOR ENERGY TRACKING
// ======================

// updateEnergyMeasurements() integrates power (W) over the elapsed time period
// Energy (in Wh) = Power (W) * time (h) where Power = Voltage * Current
void updateEnergyMeasurements() {
  unsigned long now = millis();
  unsigned long dt = now - lastEnergyUpdate; // in milliseconds
  
  // Convert dt from ms to hours
  float dtHours = dt / 3600000.0;
  
  // Read the battery voltage and corresponding currents
  float voltage = readBatteryVoltage();
  float currentIn = readCurrent(BATTERY_IN_CURRENT_PIN);
  if (currentIn < 0){currentIn=0;}
  float currentOut = readCurrent(LOAD_CURRENT_PIN);
  float currentSold = readCurrent(OUTGOING_CURRENT_PIN);

  // Update accumulated charge (Coulomb counting)
  accumulatedCharge = accumulatedCharge + (currentIn * dtHours) - (currentOut * dtHours) - (currentSold * dtHours);
  currentSoC = accumulatedCharge / 26;
  // Integration: add energy (Wh) = Voltage (V) * Current (A) * dt (h)
  energyInAccumulated += voltage * currentIn * dtHours;
  energyOutAccumulated += voltage * currentOut * dtHours;
  energySoldAccumulated += voltage * currentSold * dtHours;

  // Update state values
  state.energySoldAccumulated = energySoldAccumulated;
  state.accumulatedCharge = accumulatedCharge;
  state.currentSoC = currentSoC; // Update SoC if you have a calculation for it

  // If in a sell‐trade, check percent thresholds
  bool milestoneReached = false;
  if (strcmp(state.agreementType, "sell") == 0 && state.energy_amount > 0) {
    float frac = state.energySoldAccumulated / state.energy_amount;
    if (!state.sent25 && frac >= 0.25) { 
      publishMilestone(25); 
      state.sent25 = true; 
      milestoneReached = true;
    }
    if (!state.sent50 && frac >= 0.50) { 
      publishMilestone(50); 
      state.sent50 = true; 
      milestoneReached = true;
    }
    if (!state.sent75 && frac >= 0.75) { 
      publishMilestone(75); 
      state.sent75 = true;
      milestoneReached = true;
    }
    if (!state.sent100 && frac >= 0.90) { 
      publishMilestone(100); 
      state.sent100 = true;
      milestoneReached = true;
    }
  }
  
  // Save state if a milestone was reached or every STATE_SAVE_INTERVAL_MS
  unsigned long timeSinceLastSave = now - lastStateSaveTime;
  if (milestoneReached || timeSinceLastSave >= STATE_SAVE_INTERVAL_MS) {
    saveState();
    lastStateSaveTime = now;
  }
  
  // Update for next calculation
  lastEnergyUpdate = now;
  
  // Debug output every 10 seconds
  static unsigned long lastDebugOutput = 0;
  if (now - lastDebugOutput >= 10000) {
    Serial.printf("Energy - In: %.2f Wh, Out: %.2f Wh, Sold: %.2f Wh\n", 
                 energyInAccumulated, energyOutAccumulated, energySoldAccumulated);
    lastDebugOutput = now;
  }
}

void publishMilestone(int pct) {
  StaticJsonDocument<128> out;
  out["agreement_id"] = state.agreementId;
  out["milestone_sold%"] = pct;
  char buf[128];
  serializeJson(out, buf);
  mqttClient.publish(AWS_IOT_TOPIC_PUBLISH, buf);
  Serial.printf("✓ Milestone %d%% reached for '%s'\n", pct, state.agreementId);
  // We're now saving state in updateEnergyMeasurements based on the milestoneReached flag
}

// Simple accessor functions to get the accumulated energy values
float getEnergyIn() {
  return energyInAccumulated;
}

float getEnergyOut() {
  return energyOutAccumulated;
}