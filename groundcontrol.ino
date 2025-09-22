// USB MAVLink-LoRa Bridge - QGroundControl via USB
#include <RadioLib.h>

// LoRa Radio pins for Heltec V3
#define LORA_NSS 8
#define LORA_DIO1 14
#define LORA_NRST 12
#define LORA_BUSY 13

// LED pin for debug feedback
#define LED_PIN 35

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

// MAVLink message IDs
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAVLINK_MSG_ID_COMMAND_ACK 77

// MAVLink commands
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#define MAV_CMD_NAV_TAKEOFF 22
#define MAV_CMD_NAV_LAND 21
#define MAV_CMD_NAV_RETURN_TO_LAUNCH 20

// MAVLink constants
#define MAV_TYPE_QUADROTOR 2
#define MAV_AUTOPILOT_ARDUPILOTMEGA 3
#define MAV_STATE_STANDBY 3
#define MAV_STATE_ACTIVE 4
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 1
#define MAV_MODE_FLAG_SAFETY_ARMED 128
#define MAV_RESULT_ACCEPTED 0

// CRC_EXTRA values
uint8_t getCrcExtra(uint8_t msgid) {
  switch(msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: return 50;
    case MAVLINK_MSG_ID_SYS_STATUS: return 124;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return 104;
    case MAVLINK_MSG_ID_COMMAND_LONG: return 152;
    case MAVLINK_MSG_ID_COMMAND_ACK: return 143;
    default: return 0;
  }
}

// System state
struct {
  uint8_t sequence = 0;
  bool armed = false;
  float battery_voltage = 12.6;
  float latitude = 26.6841;   // Greenacres, FL
  float longitude = -80.0851;
  float altitude = 10.0;
  uint8_t satellites = 10;
} drone_state;

// Add a packet counter
uint32_t packet_counter = 0;

// Timing
uint32_t last_heartbeat = 0;
uint32_t last_status = 0;
uint32_t command_count = 0;

// MAVLink receive buffer
uint8_t mav_buffer[300];
uint16_t mav_pos = 0;
bool mav_in_packet = false;

void setup() {
  // USB Serial for MAVLink (QGroundControl connection)
  Serial.begin(57600);
  delay(2000);
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize LoRa
  SPI.begin(9, 11, 10, 8);
  int state = radio.begin(915.0);
  
  if (state == RADIOLIB_ERR_NONE) {
    radio.setSpreadingFactor(7);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setOutputPower(14);
    radio.setSyncWord(0x12);
    radio.startReceive();
  }
  
  last_heartbeat = millis();
  last_status = millis();
}

// MAVLink CRC calculation
uint16_t crc_calculate(const uint8_t* data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    uint8_t tmp = *data++ ^ (crc & 0xFF);
    tmp ^= (tmp << 4);
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
  }
  return crc;
}

uint16_t crc_accumulate(uint8_t b, uint16_t crc) {
  uint8_t tmp = b ^ (crc & 0xFF);
  tmp ^= (tmp << 4);
  return (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

// Send heartbeat to QGroundControl
void sendHeartbeat() {
  uint8_t packet[17];
  
  packet[0] = 0xFE;  // STX
  packet[1] = 9;     // Payload length
  packet[2] = drone_state.sequence++;
  packet[3] = 1;     // System ID
  packet[4] = 1;     // Component ID
  packet[5] = MAVLINK_MSG_ID_HEARTBEAT;
  
  // Payload
  packet[6] = MAV_TYPE_QUADROTOR;
  packet[7] = MAV_AUTOPILOT_ARDUPILOTMEGA;
  packet[8] = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (drone_state.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);
  packet[9] = 0;  // custom_mode
  packet[10] = 0;
  packet[11] = 0;
  packet[12] = 0;
  packet[13] = drone_state.armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
  packet[14] = 3;  // MAVLink version
  
  // Calculate checksum
  uint16_t checksum = crc_calculate(&packet[1], 14);
  checksum = crc_accumulate(getCrcExtra(MAVLINK_MSG_ID_HEARTBEAT), checksum);
  
  packet[15] = checksum & 0xFF;
  packet[16] = (checksum >> 8) & 0xFF;
  
  Serial.write(packet, 17);
  last_heartbeat = millis();
}

// Send system status
void sendSystemStatus() {
  uint8_t packet[39];
  
  packet[0] = 0xFE;
  packet[1] = 31;
  packet[2] = drone_state.sequence++;
  packet[3] = 1;
  packet[4] = 1;
  packet[5] = MAVLINK_MSG_ID_SYS_STATUS;
  
  // System status - all sensors present and healthy
  uint32_t sensors_present = 0x3FFFFFFF;  // All sensors present
  uint32_t sensors_enabled = 0x3FFFFFFF;  // All sensors enabled  
  uint32_t sensors_health = 0x3FFFFFFF;   // All sensors healthy
  memcpy(&packet[6], &sensors_present, 4);
  memcpy(&packet[10], &sensors_enabled, 4);
  memcpy(&packet[14], &sensors_health, 4);
  
  uint16_t voltage = drone_state.battery_voltage * 1000;
  memcpy(&packet[18], &voltage, 2);
  
  int16_t current = 2500;
  memcpy(&packet[20], &current, 2);
  
  packet[22] = 85;  // Battery %
  memset(&packet[23], 0, 14);
  
  // Checksum
  uint16_t checksum = crc_calculate(&packet[1], 36);
  checksum = crc_accumulate(getCrcExtra(MAVLINK_MSG_ID_SYS_STATUS), checksum);
  
  packet[37] = checksum & 0xFF;
  packet[38] = (checksum >> 8) & 0xFF;
  
  Serial.write(packet, 39);
  last_status = millis();
}

// Send GPS position
void sendGlobalPosition() {
  uint8_t packet[36];
  
  packet[0] = 0xFE;
  packet[1] = 28;
  packet[2] = drone_state.sequence++;
  packet[3] = 1;
  packet[4] = 1;
  packet[5] = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
  
  uint32_t time_boot = millis();
  int32_t lat = drone_state.latitude * 1e7;
  int32_t lon = drone_state.longitude * 1e7;
  int32_t alt = drone_state.altitude * 1000;
  
  memcpy(&packet[6], &time_boot, 4);
  memcpy(&packet[10], &lat, 4);
  memcpy(&packet[14], &lon, 4);
  memcpy(&packet[18], &alt, 4);
  memcpy(&packet[22], &alt, 4);
  memset(&packet[26], 0, 6);  // Velocities
  
  uint16_t hdg = 0;
  memcpy(&packet[32], &hdg, 2);
  
  // Checksum
  uint16_t checksum = crc_calculate(&packet[1], 33);
  checksum = crc_accumulate(getCrcExtra(MAVLINK_MSG_ID_GLOBAL_POSITION_INT), checksum);
  
  packet[34] = checksum & 0xFF;
  packet[35] = (checksum >> 8) & 0xFF;
  
  Serial.write(packet, 36);
}

// Send command ACK
void sendCommandAck(uint16_t command) {
  uint8_t packet[11];
  
  packet[0] = 0xFE;
  packet[1] = 3;
  packet[2] = drone_state.sequence++;
  packet[3] = 1;
  packet[4] = 1;
  packet[5] = MAVLINK_MSG_ID_COMMAND_ACK;
  
  packet[6] = command & 0xFF;
  packet[7] = (command >> 8) & 0xFF;
  packet[8] = MAV_RESULT_ACCEPTED;
  
  uint16_t checksum = crc_calculate(&packet[1], 8);
  checksum = crc_accumulate(getCrcExtra(MAVLINK_MSG_ID_COMMAND_ACK), checksum);
  
  packet[9] = checksum & 0xFF;
  packet[10] = (checksum >> 8) & 0xFF;
  
  Serial.write(packet, 11);
}

// Process MAVLink command
void processMAVLinkMessage(uint8_t* buffer) {
  uint8_t msgid = buffer[5];
  
  if (msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    // COMMAND_LONG payload structure:
    // param1-7 (28 bytes), command (2 bytes), target_system, target_component, confirmation
    uint16_t command = buffer[34] | (buffer[35] << 8);  // Command at offset 28 in payload
    float param1;
    memcpy(&param1, &buffer[6], 4);  // param1 at start of payload
    
    // Debug: Flash LED pattern based on command ID
    if (command == 400) {
      // ARM/DISARM - 5 rapid blinks
      for(int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
    } else {
      // Other command - 2 slow blinks
      for(int i = 0; i < 2; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
      }
    }
    
    String loraCmd;
    switch (command) {
      case MAV_CMD_COMPONENT_ARM_DISARM:
        // Don't set armed state immediately - wait for drone response
        if (param1 > 0.5) {
          loraCmd = "ARM:MAVLINK";
          Serial.println("ARM command received, sending to drone...");
        } else {
          loraCmd = "DISARM:MAVLINK";
          Serial.println("DISARM command received, sending to drone...");
        }
        
        // Log for debugging
        Serial.print("ARM/DISARM command: param1=");
        Serial.println(param1);
        break;
        
      case MAV_CMD_NAV_TAKEOFF:
        loraCmd = "TAKEOFF:MAVLINK";
        break;
        
      case MAV_CMD_NAV_LAND:
        loraCmd = "LAND:MAVLINK";
        break;
        
      case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        loraCmd = "RTL:MAVLINK";
        break;
        
      default:
        // Check if this might be an ARM command with different ID
        if (param1 > 0.5) {
          loraCmd = "ARM:MAVLINK";
        } else if (param1 < 0.5 && param1 >= 0) {
          loraCmd = "DISARM:MAVLINK";
        } else {
          loraCmd = "UNKNOWN:MAVLINK:" + String(command);
        }
        break;
    }
    
    // Send to drone via LoRa
    sendLoRaCommand(loraCmd);
    
    // Send ACK to QGC
    sendCommandAck(command);
    command_count++;
  }
}

// Send command via LoRa
void sendLoRaCommand(String cmd) {
  radio.standby();
  
  // Add packet ID to the command
  String fullCmd = "CMD" + String(packet_counter++) + ":" + cmd;
  
  int state = radio.transmit(fullCmd);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa TX: " + fullCmd);  // Debug output
  }
  
  radio.startReceive();
}

// Check for LoRa messages
void checkLoRaReceive() {
  String received;
  int state = radio.readData(received);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa RX: " + received);  // Debug output
    
    // Parse telemetry from drone
    if (received.indexOf("BAT=") >= 0) {
      int idx = received.indexOf("BAT=");
      drone_state.battery_voltage = received.substring(idx + 4).toFloat();
    }
    
    // Check for armed state changes
    bool prev_armed = drone_state.armed;
    if (received.indexOf("ARM=1") >= 0) drone_state.armed = true;
    if (received.indexOf("ARM=0") >= 0) drone_state.armed = false;
    
    // If armed state changed, immediately update QGC
    if (prev_armed != drone_state.armed) {
      Serial.println("Armed state changed to: " + String(drone_state.armed));
      sendHeartbeat();
      sendSystemStatus();
    }
  }
}

// Read MAVLink from USB
void readMAVLink() {
  while (Serial.available()) {
    uint8_t c = Serial.read();
    
    if (!mav_in_packet) {
      if (c == 0xFE) {
        mav_buffer[0] = c;
        mav_pos = 1;
        mav_in_packet = true;
      }
      continue;
    }
    
    if (mav_pos < sizeof(mav_buffer)) {
      mav_buffer[mav_pos++] = c;
    } else {
      mav_in_packet = false;
      mav_pos = 0;
      continue;
    }
    
    if (mav_pos >= 6) {
      uint8_t payload_len = mav_buffer[1];
      uint8_t packet_len = payload_len + 8;
      
      if (mav_pos >= packet_len) {
        // Verify checksum
        uint16_t received_crc = mav_buffer[packet_len-2] | (mav_buffer[packet_len-1] << 8);
        uint16_t calculated_crc = crc_calculate(&mav_buffer[1], packet_len - 3);
        
        uint8_t msgid = mav_buffer[5];
        uint8_t crc_extra = getCrcExtra(msgid);
        if (crc_extra != 0) {
          calculated_crc = crc_accumulate(crc_extra, calculated_crc);
        }
        
        if (received_crc == calculated_crc) {
          processMAVLinkMessage(mav_buffer);
        }
        
        mav_in_packet = false;
        mav_pos = 0;
      }
    }
  }
}

void loop() {
  // Read commands from QGroundControl (USB)
  readMAVLink();
  
  // Check for drone telemetry (LoRa)
  checkLoRaReceive();
  
  // Send heartbeat at 1Hz
  if (millis() - last_heartbeat > 1000) {
    sendHeartbeat();
  }
  
  // Send status at 2Hz
  if (millis() - last_status > 500) {
    sendSystemStatus();
    sendGlobalPosition();
  }
  
  delay(1);
}
