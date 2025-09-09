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

// Simple states
enum DroneState {
  LISTENING,
  PROCESSING,
  SENDING_ACK
};

DroneState currentState = LISTENING;
String receivedMessage = "";
String currentPacketId = "";
String currentCommand = "";

// Stats
int packetsReceived = 0;
int acksSent = 0;
unsigned long lastCommandTime = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== Drone ===");
  
  // Setup pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(VEXT_PIN, OUTPUT); 
  digitalWrite(VEXT_PIN, LOW);
  
  // Initialize display
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
    while(1);
  }
  
  showStatus("Starting...", "Radio Init");
  
  // Initialize radio
  SPI.begin(9, 11, 10, 8);
  int state = radio.begin(915.0);
  
  if (state == RADIOLIB_ERR_NONE) {
    radio.setSpreadingFactor(7);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setOutputPower(14);
    radio.startReceive();
    
    Serial.println("Radio initialized successfully");
    showStatus("Radio Ready", "Listening...");
    delay(1000);
  } else {
    Serial.println("Radio failed: " + String(state));
    showStatus("RADIO FAILED", "Error: " + String(state));
    while(1);
  }
}

void showStatus(String line1, String line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Drone");
  display.println("===========");
  display.println("State: " + stateToString(currentState));
  display.println(line1);
  display.println(line2);
  display.println("RX: " + String(packetsReceived));
  display.println("ACK: " + String(acksSent));
  display.display();
}

String stateToString(DroneState state) {
  switch(state) {
    case LISTENING: return "LISTEN";
    case PROCESSING: return "PROCESS";
    case SENDING_ACK: return "ACK";
    default: return "UNKNOWN";
  }
}

void parseCommand(String message) {
  int firstColon = message.indexOf(':');
  int secondColon = message.indexOf(':', firstColon + 1);
  
  if (firstColon > 0 && secondColon > firstColon) {
    currentPacketId = message.substring(0, firstColon);
    currentCommand = message.substring(firstColon + 1, secondColon);
  }
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
      
      // Update display periodically
      static unsigned long lastUpdate = 0;
      if (millis() - lastUpdate > 1000) {
        unsigned long timeSince = (millis() - lastCommandTime) / 1000;
        showStatus("Listening...", "Last: " + String(timeSince) + "s ago");
        lastUpdate = millis();
      }
      break;
    }
    
    case PROCESSING: {
      showStatus("Processing...", "Parsing command");
      
      parseCommand(receivedMessage);
      
      // Process the command
      Serial.println("Processing command: " + currentCommand);
      if (currentCommand == "LED_ON") {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED ON");
      } else if (currentCommand == "LED_OFF") {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED OFF");
      } else if (currentCommand == "STATUS") {
        Serial.println("Status OK");
      }
      
      lastCommandTime = millis();
      currentState = SENDING_ACK;
      Serial.println("→ State: SENDING_ACK");
      break;
    }
    
    case SENDING_ACK: {
      showStatus("Sending ACK", "ID: " + currentPacketId);
      
      radio.standby();
      delay(10);
      
      String ackMessage = "ACK:" + currentPacketId + ":OK";
      int result = radio.transmit(ackMessage);
      
      if (result == RADIOLIB_ERR_NONE) {
        acksSent++;
        Serial.println("ACK sent for: " + currentPacketId);
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        
        showStatus("ACK Sent!", "Success");
        delay(500);
      } else {
        Serial.println("ACK failed: " + String(result));
        showStatus("ACK Failed", "Error: " + String(result));
        delay(500);
      }
      
      // Return to listening
      radio.startReceive();
      currentState = LISTENING;
      Serial.println("→ State: LISTENING");
      break;
    }
  }
  
  delay(50);
}
