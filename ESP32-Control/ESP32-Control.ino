#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <time.h>

// **************************************
// AWS IoT Certificates and Connection Info
// **************************************
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

// **************************************
// WiFi Settings & Trade Topic
// **************************************
const char* WIFI_SSID = "TECNO";
const char* WIFI_PASSWORD = "abed2003";

// Define the trade topic. (For this control ESP32, we subscribe to the trade messages.)
const char* AWS_IOT_TRADE_TOPIC = "nakhnoukh/command2";

// **************************************
// Relay and EEPROM Settings
// **************************************
// Relay pins (active LOW means HIGH keeps the relay off)
#define RELAY_SEND_PIN   16  // 1-channel relay (sending power)
#define RELAY_RECV1_PIN  17  // 2-channel relay channel 1 (e.g., solar panel)
#define RELAY_RECV2_PIN  18  // 2-channel relay channel 2 (e.g., other prosumer)

// EEPROM parameters for storing agreement_id
#define EEPROM_SIZE         512
#define AGREEMENT_ID_ADDR   0
#define MAX_ID_LENGTH       64

// **************************************
// Global Variables and Objects
// **************************************
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// For time synchronization
bool timeIsSynchronized = false;
const char* NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 10800;   // UTC+3
const int DAYLIGHT_OFFSET_SEC = 0;

// **************************************
// Function Prototypes
// **************************************
void setupWiFi();
void setupTime();
void setupAWS();
bool reconnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void processTrade(const char* type, float energy_amount, const char* agreement_id);
void controlRelayForBuy();
void controlRelayForSell();
void storeAgreementId(const char* id);

// **************************************
// WiFi Setup
// **************************************
void setupWiFi() {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 80) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi.");
  }
}

// **************************************
// Time Setup (required for TLS/SSL)
// **************************************
void setupTime() {
  // Configure time with NTP - THIS IS CRITICAL FOR TLS/SSL
  Serial.println("Setting up time synchronization (UTC+3)...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  
  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  int attempts = 0;
  while (now < 24 * 3600 * 2 && attempts < 100) {
    // esp_task_wdt_reset();
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
  } else {
    Serial.println("Failed to get time sync - this may cause TLS connection issues!");
  }
}

// **************************************
// AWS IoT Setup
// **************************************
void setupAWS() {
  // Set AWS certificates on the secure WiFi client
  wifiClient.setCACert(AWS_IOT_ROOT_CA);
  wifiClient.setCertificate(AWS_IOT_CERT);
  wifiClient.setPrivateKey(AWS_IOT_PRIVATE_KEY);
  
  // Configure MQTT
  mqttClient.setServer(AWS_IOT_ENDPOINT, 8883);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
  
  Serial.println("AWS IoT configuration complete.");
}

// **************************************
// MQTT Callback - Parse Trade JSON and Process It
// **************************************
// Expected JSON structure:
// {
//   "type": "buy" or "sell",
//   "energy_amount": <value in KWH>,
//   "agreement_id": "some_generated_id"
// }
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic [");
  Serial.print(topic);
  Serial.println("]");
  
  StaticJsonDocument<256> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, payload, length);
  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  const char* type = jsonDoc["type"];
  float energy_amount = jsonDoc["energy_amount"];
  const char* agreement_id = jsonDoc["agreement_id"];
  
  Serial.print("Parsed trade - Type: ");
  Serial.print(type);
  Serial.print(", Energy: ");
  Serial.print(energy_amount);
  Serial.print(" KWH, Agreement ID: ");
  Serial.println(agreement_id);
  
  processTrade(type, energy_amount, agreement_id);
}

// **************************************
// Process Trade Command - Store Agreement ID and Control Relays
// **************************************
void processTrade(const char* type, float energy_amount, const char* agreement_id) {
  // Store the agreement ID in EEPROM
  storeAgreementId(agreement_id);
  
  // Act on the trade type
  if (strcmp(type, "buy") == 0) {
    controlRelayForBuy();
    Serial.println("Relays set for RECEIVING power (buy).");
  } else if (strcmp(type, "sell") == 0) {
    controlRelayForSell();
    Serial.println("Relays set for SENDING power (sell).");
  } else {
    Serial.println("Unknown trade type received.");
    controlRelayForIdle();
  }
}

// **************************************
// Relay Control for "buy" (Receiving Power)
// **************************************
void controlRelayForIdle() {
  digitalWrite(RELAY_SEND_PIN, LOW);    // Sending relay ON (active LOW)
  // Optionally, you can set the 2-channel relay to a safe state:
  digitalWrite(RELAY_RECV1_PIN, LOW);
  digitalWrite(RELAY_RECV2_PIN, LOW);
}

// **************************************
// Relay Control for "buy" (Receiving Power)
// **************************************
void controlRelayForBuy() {
  // For receiving power:
  // Activate channel 2 (other prosumer) on the 2-channel relay,
  // and ensure the sending relay is OFF.
  digitalWrite(RELAY_RECV1_PIN, HIGH);  // Channel 1 off (e.g., solar off)
  digitalWrite(RELAY_RECV2_PIN, HIGH);   // Channel 2 ON (other prosumer)
  digitalWrite(RELAY_SEND_PIN, LOW);   // Sending relay OFF
}

// **************************************
// Relay Control for "sell" (Sending Power)
// **************************************
void controlRelayForSell() {
  // For sending power:
  // Activate the sending relay.
  digitalWrite(RELAY_SEND_PIN, HIGH);    // Sending relay ON (active LOW)
  // Optionally, you can set the 2-channel relay to a safe state:
  digitalWrite(RELAY_RECV1_PIN, LOW);
  digitalWrite(RELAY_RECV2_PIN, LOW);
}

// **************************************
// Store Agreement ID in EEPROM
// **************************************
void storeAgreementId(const char* id) {
  // Read existing
  char oldId[MAX_ID_LENGTH];
  for (int i = 0; i < MAX_ID_LENGTH; i++) {
    oldId[i] = EEPROM.read(AGREEMENT_ID_ADDR + i);
  }

  if (strcmp(oldId, id) == 0) {
    // Same ID, nothing to do
    return;
  }

  // Write new ID in place
  int len = strlen(id);
  int i = 0;
  for (; i < len && i < MAX_ID_LENGTH - 1; i++) {
    EEPROM.write(AGREEMENT_ID_ADDR + i, id[i]);
  }
  EEPROM.write(AGREEMENT_ID_ADDR + i, '\0');  // terminate
  EEPROM.commit();
  Serial.print("Updated agreement_id: ");
  Serial.println(id);
}


// **************************************
// Reconnect to AWS IoT MQTT Broker
// **************************************
bool reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected.");
    return false;
  }
  
  Serial.println("Attempting MQTT connection...");
  String clientId = "ESP32Client-" + String(random(0xffff), HEX);
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("Connected to AWS IoT");
    // Subscribe to the trade topic
    if (mqttClient.subscribe(AWS_IOT_TRADE_TOPIC)) {
      Serial.print("Subscribed to trade topic: ");
      Serial.println(AWS_IOT_TRADE_TOPIC);
    } else {
      Serial.println("Failed to subscribe to trade topic.");
    }
    return true;
  } else {
    Serial.print("MQTT connect failed, state=");
    Serial.println(mqttClient.state());
    return false;
  }
}

// **************************************
// Arduino Setup
// **************************************
void setup() {
  // Configure Task Watchdog Timer (TWDT)
esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 25000,    // 25-second timeout
    .trigger_panic = true   // Trigger panic on timeout
};
esp_task_wdt_init(&twdt_config);
  Serial.begin(115200);
  delay(1000);
  
  // Initialize EEPROM with the specified size
  EEPROM.begin(EEPROM_SIZE);
  
  // Set up relay pins
  pinMode(RELAY_SEND_PIN, OUTPUT);
  pinMode(RELAY_RECV1_PIN, OUTPUT);
  pinMode(RELAY_RECV2_PIN, OUTPUT);
  
  // Ensure relays are OFF (active LOW means HIGH is off)
  digitalWrite(RELAY_SEND_PIN, LOW);
  digitalWrite(RELAY_RECV1_PIN, HIGH);
  digitalWrite(RELAY_RECV2_PIN, LOW);
  
  // Initialize WiFi, time and AWS IoT connection
  setupWiFi();
  setupTime();
  setupAWS();
  
  // Attempt an initial MQTT connection
  if (!mqttClient.connected()) {
    reconnect();
  }
  
  Serial.println("Setup complete, entering main loop.");
}

// **************************************
// Main Loop
// **************************************
void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();
}
