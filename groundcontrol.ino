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
enum GroundState {
  IDLE,
  SENDING,
  WAITING_ACK
};

GroundState currentState = IDLE;
String currentPacketId = "";
unsigned long lastSendTime = 0;
unsigned long ackTimeout = 3000;

// Command sequence
String commands[] = {"STATUS", "LED_ON", "LED_OFF", "STATUS"};
int commandIndex = 0;
int commandCounter = 0;

// Stats
int packetsSent = 0;
int acksReceived = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("===  Machine Ground Control ===");
  
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
    
    Serial.println("Radio initialized successfully");
    showStatus("Radio Ready", "Starting...");
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
  display.println("State Ground");
  display.println("============");
  display.println("State: " + stateToString(currentState));
  display.println(line1);
  display.println(line2);
  display.println("TX: " + String(packetsSent));
  display.println("ACK: " + String(acksReceived));
  display.display();
}

String stateToString(GroundState state) {
  switch(state) {
    case IDLE: return "IDLE";
    case SENDING: return "SEND";
    case WAITING_ACK: return "WAIT";
    default: return "UNKNOWN";
  }
}

String generatePacketId() {
  commandCounter++;
  return "CMD" + String(commandCounter);
}

void loop() {
  static unsigned long lastCommandTime = 0;
  
  switch(currentState) {
    
    case IDLE: {
      // Send command every 5 seconds
      if (millis() - lastCommandTime > 5000) {
        currentState = SENDING;
        Serial.println("→ State: SENDING");
      } else {
        // Show countdown
        unsigned long nextCmd = 5000 - (millis() - lastCommandTime);
        if (nextCmd > 5000) nextCmd = 0;
        showStatus("Ready", "Next: " + String(nextCmd/1000) + "s");
      }
      break;
    }
    
    case SENDING: {
      String command = commands[commandIndex % 4];
      currentPacketId = generatePacketId();
      String message = currentPacketId + ":" + command + ":";
      
      showStatus("Sending...", "CMD: " + command);
      Serial.println("Sending: " + message);
      
      radio.standby();
      delay(10);
      
      int result = radio.transmit(message);
      
      if (result == RADIOLIB_ERR_NONE) {
        packetsSent++;
        lastSendTime = millis();
        lastCommandTime = millis();
        commandIndex++;
        
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        
        // Start listening for ACK
        radio.startReceive();
        currentState = WAITING_ACK;
        Serial.println("→ State: WAITING_ACK");
      } else {
        Serial.println("Send failed: " + String(result));
        showStatus("Send Failed", "Retrying...");
        delay(1000);
        currentState = IDLE;
      }
      break;
    }
    
    case WAITING_ACK: {
      showStatus("Waiting ACK", "ID: " + currentPacketId);
      
      String received;
      int state = radio.readData(received);
      
      if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Received: " + received);
        
        // Ignore command messages
        if (received.startsWith("CMD")) {
          Serial.println("Ignoring command");
          break;
        }
        
        // Check if it's our ACK
        if (received.startsWith("ACK:" + currentPacketId)) {
          acksReceived++;
          Serial.println("ACK received!");
          
          showStatus("ACK Received!", "Success");
          delay(1000);
          
          currentState = IDLE;
          Serial.println("→ State: IDLE");
        } else {
          Serial.println("Wrong ACK received");
        }
      }
      
      // Check timeout
      if (millis() - lastSendTime > ackTimeout) {
        Serial.println("ACK timeout");
        showStatus("ACK Timeout", "No response");
        delay(1000);
        
        currentState = IDLE;
        Serial.println("→ State: IDLE");
      }
      break;
    }
  }
  
  delay(50);
}
