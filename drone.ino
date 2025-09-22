// Final Working Drone - Compatible with QGroundControl
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 21
#define OLED_SDA 17
#define OLED_SCL 18
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Power and LED
#define VEXT_PIN 36
#define LED_PIN 35

// LoRa Radio pins
#define LORA_NSS 8
#define LORA_DIO1 14
#define LORA_NRST 12
#define LORA_BUSY 13

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

// Drone states
enum DroneState {
  LISTENING,
  PROCESSING,
  SENDING_RESPONSE
};

DroneState currentState = LISTENING;
String receivedMessage = "";
String currentPacketId = "";
String currentCommand = "";

// Drone status for QGroundControl
struct DroneStatus {
  bool armed = false;
  bool flying = false;
  String mode = "DISARMED";
  float battery_voltage = 12.6;
  float battery_current = 0.0;
  int8_t battery_remaining = 85;
  
  // Simulated GPS (Greenacres, FL)
  bool gps_fix = true;
  float latitude = 26.6841;
  float longitude = -80.0851;
  float altitude = 0.0;
  int satellites = 8;
  float speed = 0.0;
  float hdop = 1.2;
  
  // Attitude (simulated)
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
};

DroneStatus drone;

// Stats
int packetsReceived = 0;
int responsesSent = 0;
unsigned long lastCommandTime = 0;
unsigned long lastTelemetryTime = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== Final QGroundControl Drone ===");
  
  // Setup pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(VEXT_PIN, OUTPUT); 
  digitalWrite(VEXT_PIN, LOW);
  
  // Initialize display
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
  }
  
  showStatus("Starting...", "Radio Init");
  
  // Initialize LoRa
  SPI.begin(9, 11, 10, 8);
  int state = radio.begin(915.0);
  
  if (state == RADIOLIB_ERR_NONE) {
    radio.setSpreadingFactor(7);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setOutputPower(14);
    radio.startReceive();
    
    Serial.println("LoRa initialized successfully");
    showStatus("Systems Ready", "QGC Compatible");
  } else {
    Serial.println("LoRa failed: " + String(state));
    showStatus("LoRa FAILED", "Error: " + String(state));
  }
}

void showStatus(String line1, String line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("QGC Drone");
  display.println("=========");
  display.println("State: " + stateToString(currentState));
  display.println(line1);
  display.println(line2);
  display.println("Armed: " + String(drone.armed ? "YES" : "NO"));
  display.println("GPS: " + String(drone.satellites) + " sats");
  display.println("Alt: " + String(drone.altitude, 1) + "m");
  display.display();
}

String stateToString(DroneState state) {
  switch(state) {
    case LISTENING: return "LISTEN";
    case PROCESSING: return "PROCESS";
    case SENDING_RESPONSE: return "RESPOND";
    default: return "UNKNOWN";
  }
}

void parseCommand(String message) {
  int firstColon = message.indexOf(':');
  int secondColon = message.indexOf(':', firstColon + 1);
  
  if (secondColon > firstColon) {
    // Standard format: CMD1:ARM:MAVLINK:...
    currentPacketId = message.substring(0, firstColon);
    currentCommand = message.substring(firstColon + 1, secondColon);
  } else if (firstColon > 0) {
    // Simple format: CMD:ARM
    currentPacketId = "AUTO" + String(millis());  // Auto-generate packet ID
    currentCommand = message.substring(firstColon + 1);
  }
  
  Serial.println("Parsed - ID: " + currentPacketId + " CMD: " + currentCommand);
}

void processMAVLinkCommand() {
  Serial.println("Processing MAVLink command: " + currentCommand);
  
  if (currentCommand == "ARM") {
    if (drone.gps_fix) {
      drone.armed = true;
      drone.mode = "ARMED";
      drone.battery_current = 2.5;
      Serial.println("✓ DRONE ARMED (GPS fix available)");
      digitalWrite(LED_PIN, HIGH);
    } else {
      Serial.println("✗ ARM REJECTED - No GPS fix");
    }
    
  } else if (currentCommand == "DISARM") {
    drone.armed = false;
    drone.flying = false;
    drone.mode = "DISARMED";
    drone.altitude = 0.0;
    drone.battery_current = 0.0;
    Serial.println("✓ DRONE DISARMED");
    digitalWrite(LED_PIN, LOW);
    
  } else if (currentCommand == "TAKEOFF") {
    if (drone.armed && drone.gps_fix) {
      drone.flying = true;
      drone.mode = "TAKEOFF";
      drone.altitude = 10.0; // Simulate takeoff to 10m
      drone.battery_current = 8.5;
      Serial.println("✓ TAKEOFF INITIATED (GPS guided)");
      
      // Simulate takeoff sequence
      for(int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
      }
      digitalWrite(LED_PIN, HIGH);
    } else {
      Serial.println("✗ TAKEOFF REJECTED - Not armed or no GPS");
    }
    
  } else if (currentCommand == "LAND") {
    drone.flying = false;
    drone.mode = "LANDING";
    drone.altitude = 0.0;
    drone.battery_current = 3.0;
    Serial.println("✓ LANDING INITIATED");
    
    // Simulate landing sequence
    for(int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
    }
    
  } else if (currentCommand == "RTL") {
    if (drone.gps_fix) {
      drone.mode = "RTL";
      drone.battery_current = 6.0;
      Serial.println("✓ RETURN TO LAUNCH (GPS guided)");
    } else {
      Serial.println("✗ RTL REJECTED - No GPS fix");
    }
    
  } else {
    Serial.println("✗ Unknown MAVLink command: " + currentCommand);
  }
  
  lastCommandTime = millis();
  updateBatteryStatus();
}

void updateBatteryStatus() {
  // Simulate battery drain
  static unsigned long lastBatteryUpdate = 0;
  
  if (millis() - lastBatteryUpdate > 5000) {
    float drainRate = drone.battery_current * 0.001;
    drone.battery_voltage -= drainRate;
    
    if (drone.battery_voltage < 10.5) drone.battery_voltage = 10.5;
    
    drone.battery_remaining = map(drone.battery_voltage * 10, 105, 126, 0, 100);
    if (drone.battery_remaining > 100) drone.battery_remaining = 100;
    if (drone.battery_remaining < 0) drone.battery_remaining = 0;
    
    lastBatteryUpdate = millis();
  }
}

String createAckPacket() {
  String ack = "ACK:" + currentPacketId + ":OK:";
  ack += "ARM=" + String(drone.armed ? "1" : "0") + ",";
  ack += "FLY=" + String(drone.flying ? "1" : "0") + ",";
  ack += "MODE=" + drone.mode + ",";
  ack += "BAT=" + String(drone.battery_voltage, 2) + ",";
  ack += "GPS=" + String(drone.gps_fix ? "1" : "0");
  return ack;
}

String createTelemetryPacket() {
  String telem = "TELEM:";
  telem += "GPS=" + String(drone.gps_fix ? "1" : "0") + ",";
  telem += "LAT=" + String(drone.latitude, 7) + ",";
  telem += "LON=" + String(drone.longitude, 7) + ",";
  telem += "ALT=" + String(drone.altitude, 1) + ",";
  telem += "SATS=" + String(drone.satellites) + ",";
  telem += "ARM=" + String(drone.armed ? "1" : "0") + ",";
  telem += "FLY=" + String(drone.flying ? "1" : "0") + ",";
  telem += "MODE=" + drone.mode + ",";
  telem += "BAT=" + String(drone.battery_voltage, 2) + ",";
  telem += "CURR=" + String(drone.battery_current, 1) + ",";
  telem += "REM=" + String(drone.battery_remaining);
  
  return telem;
}

String createGPSPacket() {
  String gps = "GPS:";
  gps += "FIX=" + String(drone.gps_fix ? "1" : "0") + ",";
  gps += "LAT=" + String(drone.latitude, 7) + ",";
  gps += "LON=" + String(drone.longitude, 7) + ",";
  gps += "ALT=" + String(drone.altitude, 1) + ",";
  gps += "SPD=" + String(drone.speed, 1) + ",";
  gps += "SATS=" + String(drone.satellites) + ",";
  gps += "HDOP=" + String(drone.hdop, 1);
  
  return gps;
}

void sendResponse(String response) {
  radio.standby();
  delay(10);
  
  int result = radio.transmit(response);
  
  if (result == RADIOLIB_ERR_NONE) {
    responsesSent++;
    Serial.println("Response sent: " + response);
  } else {
    Serial.println("Response send failed: " + String(result));
  }
  
  radio.startReceive();
}

void loop() {
  switch(currentState) {
    
    case LISTENING: {
      String received;
      int state = radio.readData(received);
      
      if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Received: " + received);
        
        // Ignore ACK messages
        if (received.startsWith("ACK:")) {
          Serial.println("Ignoring ACK");
          break;
        }
        
        // Valid command received
        if (received.indexOf(':') > 0) {
          receivedMessage = received;
          packetsReceived++;
          currentState = PROCESSING;
          Serial.println("→ State: PROCESSING");
        }
      }
      
      // Send periodic telemetry for QGroundControl
      if (millis() - lastTelemetryTime > 2000) { // Every 2 seconds
        String telemetry = createTelemetryPacket();
        sendResponse(telemetry);
        
        delay(300);
        
        // Also send GPS data
        String gpsData = createGPSPacket();
        sendResponse(gpsData);
        
        lastTelemetryTime = millis();
      }
      
      // Update display
      static unsigned long lastUpdate = 0;
      if (millis() - lastUpdate > 1000) {
        unsigned long timeSince = (millis() - lastCommandTime) / 1000;
        showStatus("Listening...", "Last: " + String(timeSince) + "s ago");
        lastUpdate = millis();
      }
      break;
    }
    
    case PROCESSING: {
      showStatus("Processing...", "MAVLink CMD");
      
      parseCommand(receivedMessage);
      processMAVLinkCommand();
      
      currentState = SENDING_RESPONSE;
      Serial.println("→ State: SENDING_RESPONSE");
      break;
    }
    
    case SENDING_RESPONSE: {
      showStatus("Sending...", "ACK + Status");
      
      // Send ACK first
      String ack = createAckPacket();
      sendResponse(ack);
      
      delay(300);
      
      // Then send updated telemetry
      String telemetry = createTelemetryPacket();
      sendResponse(telemetry);
      
      delay(300);
      
      // And GPS data
      String gpsData = createGPSPacket();
      sendResponse(gpsData);
      
      showStatus("Response Sent!", "Success");
      delay(1000);
      
      // Return to listening
      currentState = LISTENING;
      Serial.println("→ State: LISTENING");
      break;
    }
  }
  
  delay(50);
}
