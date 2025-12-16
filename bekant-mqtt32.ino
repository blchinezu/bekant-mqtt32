#include "lin.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <string.h>
#include <EEPROM.h>

// Pins for ESP32 D1 Mini
#define UP_BTN 27
#define DOWN_BTN 23
#define MEM1_BTN 21
#define MEM2_BTN 22
#define BTN_DEBOUNCE_MS 25

#define LED LED_BUILTIN
#define LED_ON HIGH
// Serial1 with GPIO17/16 (UART0 pins 1 & 3 conflict with USB debugging)
#define TX_PIN 17
#define RX_PIN 16
HardwareSerial SerialLIN(1);

Lin lin(SerialLIN, TX_PIN, RX_PIN);

// WiFi and MQTT settings
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "YOUR_MQTT_SERVER_IP";
const int mqtt_port = 1883;
const char* mqtt_client_id = "bekant_desk";

// MQTT topics
const char* topic_height = "bekant/height";
const char* topic_height_cm = "bekant/height_cm";
const char* topic_height_percent = "bekant/height_percent";
const char* topic_status = "bekant/status";
const char* topic_command = "bekant/command";
const char* topic_log = "bekant/log";
const char* topic_availability = "bekant/availability";
const char* topic_wifi_rssi = "bekant/wifi_rssi";
const char* topic_min_height_cm = "bekant/min_height_cm";
const char* topic_max_height_cm = "bekant/max_height_cm";
const char* topic_set_min_height = "bekant/set_min_height";
const char* topic_set_max_height = "bekant/set_max_height";
// const char* topic_height_offset = "bekant/height_offset";
// const char* topic_set_height_offset = "bekant/set_height_offset";
const char* topic_child_lock = "bekant/child_lock";
const char* topic_set_child_lock = "bekant/set_child_lock";
const char* topic_memory1_height_cm = "bekant/memory1_height_cm";
const char* topic_memory2_height_cm = "bekant/memory2_height_cm";
const char* topic_memory3_height_cm = "bekant/memory3_height_cm";
const char* topic_memory4_height_cm = "bekant/memory4_height_cm";
const char* topic_set_memory1_height = "bekant/set_memory1_height";
const char* topic_set_memory2_height = "bekant/set_memory2_height";
const char* topic_set_memory3_height = "bekant/set_memory3_height";
const char* topic_set_memory4_height = "bekant/set_memory4_height";
const char* topic_memory1_recall = "bekant/memory1_recall";
const char* topic_memory2_recall = "bekant/memory2_recall";
const char* topic_memory3_recall = "bekant/memory3_recall";
const char* topic_memory4_recall = "bekant/memory4_recall";
const char* topic_restart = "bekant/restart";

// Home Assistant MQTT Discovery topics
const char* ha_discovery_prefix = "homeassistant";
const char* device_id = "bekant_desk";
const char* device_name = "Bekant Desk";

// LIN command definitions (from working megadesk)
#define LIN_CMD_IDLE            252
#define LIN_CMD_RAISE           134
#define LIN_CMD_LOWER           133
#define LIN_CMD_FINE            135
#define LIN_CMD_FINISH          132
#define LIN_CMD_PREMOVE         196
#define LIN_CMD_RECALIBRATE     189
#define LIN_CMD_RECALIBRATE_END 188

#define HYSTERESIS    137    // ignore movement requests < this distance
#define MOVE_OFFSET   159    // amount to move when moving manually

#define DANGER_MAX_HEIGHT (6777 - HYSTERESIS)
#define DANGER_MIN_HEIGHT (162 + HYSTERESIS)

#define LIN_COMM_FAILURE_THRESHOLD 20  // Restart ESP32 after this many consecutive LIN communication failures (20 cycles = ~1 second)

// Height conversion
// Encoder range: 299-6640 = 6341 units for ~60cm = ~105.7 units/cm
#define CM_TO_ENCODER_UNITS 105.7
#define ENCODER_TO_CM(units) (((units) - DANGER_MIN_HEIGHT) / CM_TO_ENCODER_UNITS + (float)minHeightCm)
#define CM_TO_ENCODER(cm) ((uint16_t)(((cm) - (float)minHeightCm) * CM_TO_ENCODER_UNITS + DANGER_MIN_HEIGHT))

// EEPROM addresses
#define EEPROM_MIN_HEIGHT_SLOT 0
#define EEPROM_MAX_HEIGHT_SLOT 2
// #define EEPROM_HEIGHT_OFFSET_SLOT 4
#define EEPROM_CHILD_LOCK_SLOT 6
#define EEPROM_MEMORY1_SLOT 8
#define EEPROM_MEMORY2_SLOT 10
#define EEPROM_MEMORY3_SLOT 12
#define EEPROM_MEMORY4_SLOT 14

WiFiClient espClient;
PubSubClient client(espClient);

// FreeRTOS queue for MQTT log messages (non-blocking communication between cores)
QueueHandle_t mqttLogQueue;
#define MAX_LOG_MESSAGE_LENGTH 128
#define MQTT_LOG_QUEUE_SIZE 20

// FreeRTOS queue for button commands (Core 0 handles timing, Core 1 executes)
QueueHandle_t buttonCommandQueue;
#define BUTTON_COMMAND_QUEUE_SIZE 10

enum class ButtonCommand {
  UP_PRESS,
  UP_RELEASE,
  DOWN_PRESS,
  DOWN_RELEASE,
  MEM1_SHORT_PRESS,  // Released before 5s
  MEM1_LONG_PRESS,   // Held for 5s
  MEM2_SHORT_PRESS,
  MEM2_LONG_PRESS,
  BOTH_PRESS,        // Both UP+DOWN pressed
  BOTH_RELEASE
};

// Also keep current button states available for reading
volatile bool button_up_pressed = false;
volatile bool button_down_pressed = false;
volatile bool button_mem1_pressed = false;
volatile bool button_mem2_pressed = false;

// User-adjustable height limits in centimeters (stored in EEPROM)
uint16_t minHeightCm = 65;
uint16_t maxHeightCm = 120;
// int16_t heightOffsetCm = 0;  // Offset to calibrate displayed height (can be negative)
bool childLockEnabled = false;  // Child lock to disable physical buttons
uint16_t memory1Height = DANGER_MIN_HEIGHT;
uint16_t memory2Height = DANGER_MAX_HEIGHT;
uint16_t memory3Height = DANGER_MIN_HEIGHT;
uint16_t memory4Height = DANGER_MAX_HEIGHT;

enum class State {
  OFF,
  STARTING,
  UP,
  DOWN,
  STOPPING1,
  STOPPING2,
  STOPPING3,
  STOPPING4,
  STARTING_RECAL,
  RECAL,
  END_RECAL,
};

State lastState = State::OFF; // Track last movement direction for stopping

// Thread-safe shared variables (accessed from both cores)
int height = 0;
State state = State::OFF;
State previous_state = State::OFF;

uint16_t targetHeight = 0; // Target position for memory moves

enum class Command {
  NONE,
  UP,
  DOWN,
  RESET,
};

Command user_cmd = Command::NONE;
bool mqtt_command_active = false; // Flag to prevent button override of MQTT commands

// Reset functionality variables
unsigned long reset_press_start = 0;
bool both_buttons_pressed = false;
const unsigned long RESET_DURATION = 10000; // 10 seconds in milliseconds
const unsigned long MEMORY_LONG_PRESS_DURATION = 5000; // 5 seconds for storing

unsigned long t = 0;

// Function declarations
void sendInitPacket(byte a1, byte a2, byte a3 = 255, byte a4 = 255);
byte recvInitPacket();
void loadSettingsFromEEPROM();
void saveHeightLimitToEEPROM(uint16_t slot, uint16_t value);
void saveMemoryHeightToEEPROM(uint16_t slot, uint16_t value);
void publishMemoryHeights(bool force = false);
void startMemoryMove(uint16_t memoryHeight, const char* label);
void processMemoryButton(bool pressed,
                         bool buttonsEnabled,
                         bool& prevPressed,
                         unsigned long& pressStart,
                         bool& longPressHandled,
                         uint16_t& memoryHeight,
                         uint16_t eepromSlot,
                         const char* label,
                         volatile bool& publishFlag);

void delay_until(unsigned long ms) {
  unsigned long end = t + (1000 * ms);
  unsigned long d = end - micros();

  // crazy long delay; probably negative wrap-around
  // just return
  if ( d > 1000000 ) {
    t = micros();
    return;
  }

  if (d > 15000) {
    unsigned long d2 = (d-15000)/1000;
    delay(d2);
    d = end - micros();
  }
  delayMicroseconds(d);
  t = end;
}

void linInit() {
  static const byte magicPacket[3] = {246, 255, 191};
  
  // Brief stabilization delay
  delay(250);
  uint8_t initA = 0;  // Start at 0 like megadesk, not -1
  
  t = micros();
  
  // Send initialization packets
  sendInitPacket(255, 7);
  recvInitPacket();
  
  sendInitPacket(255, 7);
  recvInitPacket();
  
  sendInitPacket(255, 1, 7);
  recvInitPacket();
  
  sendInitPacket(208, 2, 7);
  recvInitPacket();
  
  // Find first motor
  bool motorAFound = false;
  while (initA < 8) {
    sendInitPacket(initA, 2, 7);
    if (recvInitPacket() > 0) {
      // mqttLog(("Init: Found Motor A at address " + String(initA)).c_str());
      motorAFound = true;
      break;
    }
    initA++;
  }
  
  if (!motorAFound) {
    mqttLog("Init: ERROR - Motor A not found! Check wiring. Restarting ESP32...");
    delay(1000);
    ESP.restart();
    return;
  }
  
  // Initialize first motor
  // mqttLog(("Init: Configuring Motor A (addr=" + String(initA) + ")...").c_str());
  sendInitPacket(initA, 6, 9, 0);
  recvInitPacket();
  
  sendInitPacket(initA, 6, 12, 0);
  recvInitPacket();
  
  sendInitPacket(initA, 6, 13, 0);
  recvInitPacket();
  
  sendInitPacket(initA, 6, 10, 0);
  recvInitPacket();
  
  sendInitPacket(initA, 6, 11, 0);
  recvInitPacket();
  
  sendInitPacket(initA, 4, 0, 0);
  recvInitPacket();
  // mqttLog("Init: Motor A configured as controller 0");
  
  // Find second motor
  byte initB = initA + 1;
  bool motorBFound = false;
  while (initB < 8) {
    sendInitPacket(initB, 2, 0, 0);
    if (recvInitPacket() > 0) {
      // mqttLog(("Init: Found Motor B at address " + String(initB)).c_str());
      motorBFound = true;
      break;
    }
    initB++;
  }
  
  if (!motorBFound) {
    mqttLog("Init: ERROR - Motor B not found! Check wiring. Restarting ESP32...");
    delay(1000);
    ESP.restart();
    return;
  }
  
  // Initialize second motor
  // mqttLog(("Init: Configuring Motor B (addr=" + String(initB) + ")...").c_str());
  sendInitPacket(initB, 6, 9, 0);
  recvInitPacket();
  
  sendInitPacket(initB, 6, 12, 0);
  recvInitPacket();
  
  sendInitPacket(initB, 6, 13, 0);
  recvInitPacket();
  
  sendInitPacket(initB, 6, 10, 0);
  recvInitPacket();
  
  sendInitPacket(initB, 6, 11, 0);
  recvInitPacket();
  
  sendInitPacket(initB, 4, 1, 0);
  recvInitPacket();
  // mqttLog("Init: Motor B configured as controller 1");
  
  // Initialize remaining motors
  uint8_t initC = initB + 1;
  while (initC < 8) {
    sendInitPacket(initC, 2, 1, 0);
    recvInitPacket();
    initC++;
  }
  
  sendInitPacket(208, 1, 7, 0);
  recvInitPacket();
  
  sendInitPacket(208, 2, 7, 0);
  recvInitPacket();
  
  delay_until(15);
  lin.send(18, magicPacket, 3);
  
  // mqttLog(("Init: Complete! Motor addresses: A=" + String(initA) + " B=" + String(initB) + 
  //          " (PIDs: A=8, B=9)").c_str());
  
  delay(5);
}

void sendInitPacket(byte a1, byte a2, byte a3, byte a4) {
  static byte packet[8] = {0, 0, 0, 0, 255, 255, 255, 255};
  packet[0] = a1;
  packet[1] = a2;
  packet[2] = a3;
  packet[3] = a4;
  delay_until(10); // long frames need more time
  lin.send(60, packet, 8);
}

byte recvInitPacket() {
  static byte resp[8];
  delay_until(10); // long frames need more time
  return lin.recv(61, resp, 8);
}

void up(bool pushed) {
  if(pushed) {
    digitalWrite(LED, LED_ON);
    user_cmd = Command::UP;
  } else {
    digitalWrite(LED, !LED_ON);
    user_cmd = Command::NONE;
  }
}

void down(bool pushed) {
  if(pushed) {
    digitalWrite(LED, LED_ON);
    user_cmd = Command::DOWN;
  } else {
    digitalWrite(LED, !LED_ON);
    user_cmd = Command::NONE;
  }
}

void reset(bool triggered) {
  if(triggered) {
    digitalWrite(LED, LED_ON);
    user_cmd = Command::RESET;
  } else {
    digitalWrite(LED, !LED_ON);
    user_cmd = Command::NONE;
  }
}

// Non-blocking MQTT log function (queues message instead of publishing directly)
void mqttLog(const char* message) {
  char* msg = (char*)malloc(MAX_LOG_MESSAGE_LENGTH);
  if (msg != NULL) {
    strncpy(msg, message, MAX_LOG_MESSAGE_LENGTH - 1);
    msg[MAX_LOG_MESSAGE_LENGTH - 1] = '\0';
    xQueueSend(mqttLogQueue, &msg, 0); // Non-blocking, drop if queue full
  }
}

// MQTT callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  String topicStr(topic);

  if (topicStr == topic_set_min_height) {
    int newMin = message.toInt();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    
    if (newMin >= dangerMinCm && newMin <= dangerMaxCm && newMin < maxHeightCm) {
      minHeightCm = newMin;
      saveHeightLimitToEEPROM(EEPROM_MIN_HEIGHT_SLOT, minHeightCm);
      client.publish(topic_min_height_cm, String(minHeightCm).c_str(), true);
      // mqttLog(("Min height set to " + String(minHeightCm) + "cm").c_str());
    } else {
      mqttLog(("Invalid min height: " + String(newMin) + "cm (must be " + String((int)dangerMinCm) + "-" + String(maxHeightCm-1) + "cm)").c_str());
    }
  } else if (topicStr == topic_set_max_height) {
    int newMax = message.toInt();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    
    if (newMax >= dangerMinCm && newMax <= dangerMaxCm && newMax > minHeightCm) {
      maxHeightCm = newMax;
      saveHeightLimitToEEPROM(EEPROM_MAX_HEIGHT_SLOT, maxHeightCm);
      client.publish(topic_max_height_cm, String(maxHeightCm).c_str(), true);
      // mqttLog(("Max height set to " + String(maxHeightCm) + "cm").c_str());
    } else {
      mqttLog(("Invalid max height: " + String(newMax) + "cm (must be " + String(minHeightCm+1) + "-" + String((int)dangerMaxCm) + "cm)").c_str());
    }
  // } else if (topicStr == topic_set_height_offset) {
  //   int newOffset = message.toInt();
  //   
  //   if (newOffset >= -50 && newOffset <= 50) {
  //     heightOffsetCm = newOffset;
  //     EEPROM.writeShort(EEPROM_HEIGHT_OFFSET_SLOT, heightOffsetCm);
  //     EEPROM.commit();
  //     client.publish(topic_height_offset, String(heightOffsetCm).c_str(), true);
  //     // mqttLog(("Height offset set to " + String(heightOffsetCm) + "cm").c_str());
  //   } else {
  //     mqttLog(("Invalid height offset: " + String(newOffset) + "cm (must be -50 to +50cm)").c_str());
  //   }
  } else if (topicStr == topic_set_child_lock) {
    if (message == "ON" || message == "on" || message == "1" || message == "true") {
      childLockEnabled = true;
      EEPROM.writeBool(EEPROM_CHILD_LOCK_SLOT, childLockEnabled);
      EEPROM.commit();
      client.publish(topic_child_lock, "ON", true);
      // mqttLog("Child lock enabled");
      // Stop desk smoothly if it's currently moving
      if (state == State::STARTING || state == State::UP || state == State::DOWN || 
          state == State::STARTING_RECAL || state == State::RECAL) {
        user_cmd = Command::NONE;
        mqtt_command_active = false;
        // Set targetHeight for smooth stop based on current direction
        if (state == State::UP || lastState == State::UP) {
          targetHeight = height + MOVE_OFFSET; // smoother deceleration when going up
        } else if (state == State::DOWN || lastState == State::DOWN) {
          targetHeight = height - HYSTERESIS; // smoother deceleration when going down
        } else {
          // For STARTING or unknown state, use current height
          targetHeight = height;
        }
        // Note: state transition to STOPPING1 will be handled in main loop
        // mqttLog("Child lock: Smooth stopping desk movement");
      }
    } else if (message == "OFF" || message == "off" || message == "0" || message == "false") {
      childLockEnabled = false;
      EEPROM.writeBool(EEPROM_CHILD_LOCK_SLOT, childLockEnabled);
      EEPROM.commit();
      client.publish(topic_child_lock, "OFF", true);
      // mqttLog("Child lock disabled");
    } else {
      mqttLog(("Invalid child lock value: " + message + " (must be ON or OFF)").c_str());
    }
  } else if (topicStr == topic_set_memory1_height) {
    float newHeight = message.toFloat();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    if (newHeight >= dangerMinCm && newHeight <= dangerMaxCm) {
      uint16_t encoderValue = constrain(CM_TO_ENCODER(newHeight), (uint16_t)DANGER_MIN_HEIGHT, (uint16_t)DANGER_MAX_HEIGHT);
      memory1Height = encoderValue;
      saveMemoryHeightToEEPROM(EEPROM_MEMORY1_SLOT, memory1Height);
      publishMemoryHeights(true);
      // mqttLog(("Memory 1 height set to " + String(newHeight) + "cm").c_str());
    } else {
      mqttLog(("Invalid Memory 1 height: " + String(newHeight, 1) + "cm (must be " + String(dangerMinCm, 1) + "-" + String(dangerMaxCm, 1) + "cm)").c_str());
    }
  } else if (topicStr == topic_set_memory2_height) {
    float newHeight = message.toFloat();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    if (newHeight >= dangerMinCm && newHeight <= dangerMaxCm) {
      uint16_t encoderValue = constrain(CM_TO_ENCODER(newHeight), (uint16_t)DANGER_MIN_HEIGHT, (uint16_t)DANGER_MAX_HEIGHT);
      memory2Height = encoderValue;
      saveMemoryHeightToEEPROM(EEPROM_MEMORY2_SLOT, memory2Height);
      publishMemoryHeights(true);
      // mqttLog(("Memory 2 height set to " + String(newHeight) + "cm").c_str());
    } else {
      mqttLog(("Invalid Memory 2 height: " + String(newHeight, 1) + "cm (must be " + String(dangerMinCm, 1) + "-" + String(dangerMaxCm, 1) + "cm)").c_str());
    }
  } else if (topicStr == topic_set_memory3_height) {
    float newHeight = message.toFloat();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    if (newHeight >= dangerMinCm && newHeight <= dangerMaxCm) {
      uint16_t encoderValue = constrain(CM_TO_ENCODER(newHeight), (uint16_t)DANGER_MIN_HEIGHT, (uint16_t)DANGER_MAX_HEIGHT);
      memory3Height = encoderValue;
      saveMemoryHeightToEEPROM(EEPROM_MEMORY3_SLOT, memory3Height);
      publishMemoryHeights(true);
      // mqttLog(("Memory 3 height set to " + String(newHeight) + "cm").c_str());
    } else {
      mqttLog(("Invalid Memory 3 height: " + String(newHeight, 1) + "cm (must be " + String(dangerMinCm, 1) + "-" + String(dangerMaxCm, 1) + "cm)").c_str());
    }
  } else if (topicStr == topic_set_memory4_height) {
    float newHeight = message.toFloat();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    if (newHeight >= dangerMinCm && newHeight <= dangerMaxCm) {
      uint16_t encoderValue = constrain(CM_TO_ENCODER(newHeight), (uint16_t)DANGER_MIN_HEIGHT, (uint16_t)DANGER_MAX_HEIGHT);
      memory4Height = encoderValue;
      saveMemoryHeightToEEPROM(EEPROM_MEMORY4_SLOT, memory4Height);
      publishMemoryHeights(true);
      // mqttLog(("Memory 4 height set to " + String(newHeight) + "cm").c_str());
    } else {
      mqttLog(("Invalid Memory 4 height: " + String(newHeight, 1) + "cm (must be " + String(dangerMinCm, 1) + "-" + String(dangerMaxCm, 1) + "cm)").c_str());
    }
  } else if (topicStr == topic_memory1_recall) {
    if (message == "PRESS" || message == "press" || message == "1" || message == "true" || message == "") {
      mqttLog("MEM1 recall triggered via MQTT");
      startMemoryMove(memory1Height, "MEM1");
    }
  } else if (topicStr == topic_memory2_recall) {
    if (message == "PRESS" || message == "press" || message == "1" || message == "true" || message == "") {
      mqttLog("MEM2 recall triggered via MQTT");
      startMemoryMove(memory2Height, "MEM2");
    }
  } else if (topicStr == topic_memory3_recall) {
    if (message == "PRESS" || message == "press" || message == "1" || message == "true" || message == "") {
      mqttLog("MEM3 recall triggered via MQTT");
      startMemoryMove(memory3Height, "MEM3");
    }
  } else if (topicStr == topic_memory4_recall) {
    if (message == "PRESS" || message == "press" || message == "1" || message == "true" || message == "") {
      mqttLog("MEM4 recall triggered via MQTT");
      startMemoryMove(memory4Height, "MEM4");
    }
  } else if (topicStr == topic_restart) {
    if (message == "RESTART" || message == "restart" || message == "1" || message == "true") {
      mqttLog("MQTT: Restart command received - restarting ESP32...");
      delay(100); // Small delay to ensure log message is sent
      ESP.restart();
    } else {
      mqttLog(("Invalid restart command: " + message + " (must be RESTART)").c_str());
    }
  } else if (topicStr == topic_command) {
    if (message == "up") {
      // Simulate UP button press
      user_cmd = Command::UP;
      mqtt_command_active = true;
      digitalWrite(LED, LED_ON);
      mqttLog("MQTT: UP command received");
    } else if (message == "down") {
      // Simulate DOWN button press
      user_cmd = Command::DOWN;
      mqtt_command_active = true;
      digitalWrite(LED, LED_ON);
      mqttLog("MQTT: DOWN command received");
    } else if (message == "reset") {
      // Simulate RESET command
      user_cmd = Command::RESET;
      mqtt_command_active = true;
      digitalWrite(LED, LED_ON);
      mqttLog("MQTT: RESET command received");
    } else if (message == "stop") {
      // Smooth stop if moving (like megadesk smooth stop on button press)
      if (targetHeight > 0 && mqtt_command_active) {
        if (targetHeight > height)
          targetHeight = height + MOVE_OFFSET; // smoother deceleration when going up
        else
          targetHeight = height - HYSTERESIS; // smoother deceleration when going down
        mqttLog(("MQTT: Smooth STOP to " + String(targetHeight)).c_str());
      } else {
        mqttLog("MQTT: STOP command received");
        targetHeight = 0; // Clear any target height
      }
      user_cmd = Command::NONE;
      mqtt_command_active = false;
      digitalWrite(LED, !LED_ON);
    } else if (message.startsWith("height-cm ")) {
      // Parse target height in centimeters: "height-cm XXX"
      String heightStr = message.substring(10); // Skip "height-cm "
      int parsedCm = heightStr.toInt();
      if (parsedCm >= minHeightCm && parsedCm <= maxHeightCm) {
        uint16_t heightUnits = CM_TO_ENCODER(parsedCm);
        targetHeight = heightUnits;
        mqtt_command_active = true;
        digitalWrite(LED, LED_ON);
        mqttLog(("MQTT: Moving to " + String(parsedCm) + "cm (encoder: " + String(heightUnits) + ")").c_str());
      } else {
        mqttLog(("MQTT: Height " + String(parsedCm) + "cm outside valid range (" + String(minHeightCm) + "-" + String(maxHeightCm) + "cm)").c_str());
      }
    } else if (message.startsWith("height-percent ")) {
      // Parse percentage from Home Assistant cover slider: "height-percent XX"
      String percentStr = message.substring(15); // Skip "height-percent "
      int percent = percentStr.toInt();
      if (percent >= 0 && percent <= 100) {
        // Convert percentage to actual cm: 0% = minHeightCm, 100% = maxHeightCm
        int actualCm = minHeightCm + ((percent * (maxHeightCm - minHeightCm)) / 100);
        uint16_t heightUnits = CM_TO_ENCODER(actualCm);
        targetHeight = heightUnits;
        mqtt_command_active = true;
        digitalWrite(LED, LED_ON);
        mqttLog(("MQTT: Moving to " + String(percent) + "% (" + String(actualCm) + "cm, encoder: " + String(heightUnits) + ")").c_str());
      } else {
        mqttLog(("MQTT: Percent " + String(percent) + " outside valid range (0-100%)").c_str());
      }
    } else if (message.startsWith("height ")) {
      // Parse target height command: "height XXXX"
      String heightStr = message.substring(7); // Skip "height "
      int parsedHeight = heightStr.toInt();
      if (parsedHeight > 0) {
        // Validate target height is within safe limits
        if (parsedHeight < DANGER_MIN_HEIGHT) {
          mqttLog(("MQTT: Target height " + String(parsedHeight) + " below minimum (" + String(DANGER_MIN_HEIGHT) + ")").c_str());
        } else if (parsedHeight > DANGER_MAX_HEIGHT) {
          mqttLog(("MQTT: Target height " + String(parsedHeight) + " above maximum (" + String(DANGER_MAX_HEIGHT) + ")").c_str());
        } else {
          targetHeight = parsedHeight;
          mqtt_command_active = true;
          digitalWrite(LED, LED_ON);
          mqttLog(("MQTT: Moving to target height " + String(targetHeight)).c_str());
        }
      } else {
        mqttLog(("MQTT: Invalid height value: " + heightStr).c_str());
      }
    } else {
      mqttLog(("MQTT: Unknown command: " + message).c_str());
    }
  }
}

// WiFi connection function
void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// MQTT connection function
void connectMQTT() {
  while (!client.connected()) {
    // Connect with Last Will and Testament (LWT)
    // Will message: set availability to "offline" when connection is lost
    if (client.connect(mqtt_client_id, topic_availability, 1, true, "offline")) {
      client.subscribe(topic_command);
      client.subscribe(topic_set_min_height);
      client.subscribe(topic_set_max_height);
      // client.subscribe(topic_set_height_offset);
      client.subscribe(topic_set_child_lock);
      client.subscribe(topic_set_memory1_height);
      client.subscribe(topic_set_memory2_height);
      client.subscribe(topic_set_memory3_height);
      client.subscribe(topic_set_memory4_height);
      client.subscribe(topic_memory1_recall);
      client.subscribe(topic_memory2_recall);
      client.subscribe(topic_memory3_recall);
      client.subscribe(topic_memory4_recall);
      client.subscribe(topic_restart);
    } else {
      delay(1000);
    }
  }
}

// Publish Home Assistant MQTT Discovery configurations
void publishHomeAssistantDiscovery() {
  if (!client.connected()) return;

  // Device information JSON (shared by all entities)
  String device_json = "\"device\":{\"identifiers\":[\"" + String(device_id) + "\"],"
                      "\"name\":\"" + String(device_name) + "\","
                      "\"manufacturer\":\"IKEA/Custom\","
                      "\"model\":\"Bekant ESP32\","
                      "\"sw_version\":\"1.0\"}";

  String availability_json = "\"availability\":[{\"topic\":\"" + String(topic_availability) + "\"}]";

  // 1. Cover entity (main desk control)
  String cover_config = "{"
    "\"name\":\"Bekant Desk\","
    "\"unique_id\":\"" + String(device_id) + "_cover\","
    "\"command_topic\":\"" + String(topic_command) + "\","
    "\"position_topic\":\"" + String(topic_height_percent) + "\","
    "\"set_position_topic\":\"" + String(topic_command) + "\","
    "\"state_topic\":\"" + String(topic_status) + "\","
    "\"payload_open\":\"up\","
    "\"payload_close\":\"down\","
    "\"payload_stop\":\"stop\","
    "\"position_open\":100,"
    "\"position_closed\":0,"
    "\"state_open\":\"open\","
    "\"state_closed\":\"closed\","
    "\"state_opening\":\"opening\","
    "\"state_closing\":\"closing\","
    "\"state_stopped\":\"stopped\","
    "\"position_template\":\"{{ value | int }}\","
    "\"set_position_template\":\"height-percent {{ position | int }}\","
    "\"device_class\":\"blind\","
    "\"icon\":\"mdi:desk\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/cover/" + device_id + "/config").c_str(), 
                 cover_config.c_str(), true);

  // 2. Status sensor
  String status_config = "{"
    "\"name\":\"Status\","
    "\"unique_id\":\"" + String(device_id) + "_status\","
    "\"state_topic\":\"" + String(topic_status) + "\","
    "\"icon\":\"mdi:information-outline\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/sensor/" + device_id + "_status/config").c_str(), 
                 status_config.c_str(), true);

  // 3. Height sensor (cm)
  String height_config = "{"
    "\"name\":\"Height\","
    "\"unique_id\":\"" + String(device_id) + "_height_cm\","
    "\"state_topic\":\"" + String(topic_height_cm) + "\","
    "\"unit_of_measurement\":\"cm\","
    "\"state_class\":\"measurement\","
    "\"icon\":\"mdi:ruler\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/sensor/" + device_id + "_height/config").c_str(), 
                 height_config.c_str(), true);

  // WiFi RSSI sensor (diagnostic)
  String rssi_config = "{"
    "\"name\":\"WiFi Signal\","
    "\"unique_id\":\"" + String(device_id) + "_rssi\","
    "\"state_topic\":\"" + String(topic_wifi_rssi) + "\","
    "\"unit_of_measurement\":\"dBm\","
    "\"device_class\":\"signal_strength\","
    "\"state_class\":\"measurement\","
    "\"entity_category\":\"diagnostic\","
    "\"enabled_by_default\":false,"
    "\"icon\":\"mdi:wifi\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/sensor/" + device_id + "_rssi/config").c_str(), 
                 rssi_config.c_str(), true);

  // Encoder position sensor (diagnostic)
  String encoder_config = "{"
    "\"name\":\"Encoder Position\","
    "\"unique_id\":\"" + String(device_id) + "_encoder\","
    "\"state_topic\":\"" + String(topic_height) + "\","
    "\"state_class\":\"measurement\","
    "\"entity_category\":\"diagnostic\","
    "\"icon\":\"mdi:counter\","
    "\"enabled_by_default\":false,"
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/sensor/" + device_id + "_encoder/config").c_str(), 
                 encoder_config.c_str(), true);

  // Log sensor (diagnostic, disabled by default)
  String log_config = "{"
    "\"name\":\"Log\","
    "\"unique_id\":\"" + String(device_id) + "_log\","
    "\"state_topic\":\"" + String(topic_log) + "\","
    "\"entity_category\":\"diagnostic\","
    "\"enabled_by_default\":false,"
    "\"icon\":\"mdi:text-box-outline\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/sensor/" + device_id + "_log/config").c_str(), 
                 log_config.c_str(), true);

  // Recalibrate button
  String recalibrate_config = "{"
    "\"name\":\"Recalibrate\","
    "\"unique_id\":\"" + String(device_id) + "_recalibrate\","
    "\"command_topic\":\"" + String(topic_command) + "\","
    "\"payload_press\":\"reset\","
    "\"icon\":\"mdi:refresh\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/button/" + device_id + "_recalibrate/config").c_str(), 
                 recalibrate_config.c_str(), true);

  // 9. Restart button
  String restart_config = "{"
    "\"name\":\"Restart\","
    "\"unique_id\":\"" + String(device_id) + "_restart\","
    "\"command_topic\":\"" + String(topic_restart) + "\","
    "\"payload_press\":\"RESTART\","
    "\"icon\":\"mdi:restart\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/button/" + device_id + "_restart/config").c_str(), 
                 restart_config.c_str(), true);

  // Memory 1 recall button
  String memory1_recall_config = "{"
    "\"name\":\"Memory 1 Recall\","
    "\"unique_id\":\"" + String(device_id) + "_memory1_recall\","
    "\"command_topic\":\"" + String(topic_memory1_recall) + "\","
    "\"payload_press\":\"PRESS\","
    "\"icon\":\"mdi:numeric-1-circle\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/button/" + device_id + "_memory1_recall/config").c_str(), 
                 memory1_recall_config.c_str(), true);

  // Memory 2 recall button
  String memory2_recall_config = "{"
    "\"name\":\"Memory 2 Recall\","
    "\"unique_id\":\"" + String(device_id) + "_memory2_recall\","
    "\"command_topic\":\"" + String(topic_memory2_recall) + "\","
    "\"payload_press\":\"PRESS\","
    "\"icon\":\"mdi:numeric-2-circle\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/button/" + device_id + "_memory2_recall/config").c_str(), 
                 memory2_recall_config.c_str(), true);

  // Memory 3 recall button
  String memory3_recall_config = "{"
    "\"name\":\"Memory 3 Recall\","
    "\"unique_id\":\"" + String(device_id) + "_memory3_recall\","
    "\"command_topic\":\"" + String(topic_memory3_recall) + "\","
    "\"payload_press\":\"PRESS\","
    "\"icon\":\"mdi:numeric-3-circle\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/button/" + device_id + "_memory3_recall/config").c_str(), 
                 memory3_recall_config.c_str(), true);

  // Memory 4 recall button
  String memory4_recall_config = "{"
    "\"name\":\"Memory 4 Recall\","
    "\"unique_id\":\"" + String(device_id) + "_memory4_recall\","
    "\"command_topic\":\"" + String(topic_memory4_recall) + "\","
    "\"payload_press\":\"PRESS\","
    "\"icon\":\"mdi:numeric-4-circle\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/button/" + device_id + "_memory4_recall/config").c_str(), 
                 memory4_recall_config.c_str(), true);

  // 10. Min height number input
  String min_height_config = "{"
    "\"name\":\"Min Height\","
    "\"unique_id\":\"" + String(device_id) + "_min_height\","
    "\"state_topic\":\"" + String(topic_min_height_cm) + "\","
    "\"command_topic\":\"" + String(topic_set_min_height) + "\","
    "\"unit_of_measurement\":\"cm\","
    "\"min\":65,"
    "\"max\":120,"
    "\"step\":1,"
    "\"mode\":\"box\","
    "\"icon\":\"mdi:arrow-collapse-down\","
    "\"entity_category\":\"config\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/number/" + device_id + "_min_height/config").c_str(), 
                 min_height_config.c_str(), true);

  // 11. Max height number input
  String max_height_config = "{"
    "\"name\":\"Max Height\","
    "\"unique_id\":\"" + String(device_id) + "_max_height\","
    "\"state_topic\":\"" + String(topic_max_height_cm) + "\","
    "\"command_topic\":\"" + String(topic_set_max_height) + "\","
    "\"unit_of_measurement\":\"cm\","
    "\"min\":65,"
    "\"max\":120,"
    "\"step\":1,"
    "\"mode\":\"box\","
    "\"icon\":\"mdi:arrow-collapse-up\","
    "\"entity_category\":\"config\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/number/" + device_id + "_max_height/config").c_str(), 
                 max_height_config.c_str(), true);

  // 12. Height offset number input
  // String offset_config = "{"
  //   "\"name\":\"Height Offset\","
  //   "\"unique_id\":\"" + String(device_id) + "_height_offset\","
  //   "\"state_topic\":\"" + String(topic_height_offset) + "\","
  //   "\"command_topic\":\"" + String(topic_set_height_offset) + "\","
  //   "\"unit_of_measurement\":\"cm\","
  //   "\"min\":-50,"
  //   "\"max\":50,"
  //   "\"step\":1,"
  //   "\"mode\":\"box\","
  //   "\"icon\":\"mdi:ruler\","
  //   "\"entity_category\":\"config\","
  //   + availability_json + "," + device_json + "}";
  // client.publish(String(String(ha_discovery_prefix) + "/number/" + device_id + "_height_offset/config").c_str(), 
  //                offset_config.c_str(), true);

  // 13. Memory 1 height number input
  String memory1_config = "{"
    "\"name\":\"Memory 1 Height\","
    "\"unique_id\":\"" + String(device_id) + "_memory1_height\","
    "\"state_topic\":\"" + String(topic_memory1_height_cm) + "\","
    "\"command_topic\":\"" + String(topic_set_memory1_height) + "\","
    "\"unit_of_measurement\":\"cm\","
    "\"min\":65,"
    "\"max\":120,"
    "\"step\":0.1,"
    "\"mode\":\"box\","
    "\"icon\":\"mdi:numeric-1-box\","
    "\"entity_category\":\"config\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/number/" + device_id + "_memory1_height/config").c_str(), 
                 memory1_config.c_str(), true);

  // 14. Memory 2 height number input
  String memory2_config = "{"
    "\"name\":\"Memory 2 Height\","
    "\"unique_id\":\"" + String(device_id) + "_memory2_height\","
    "\"state_topic\":\"" + String(topic_memory2_height_cm) + "\","
    "\"command_topic\":\"" + String(topic_set_memory2_height) + "\","
    "\"unit_of_measurement\":\"cm\","
    "\"min\":65,"
    "\"max\":120,"
    "\"step\":0.1,"
    "\"mode\":\"box\","
    "\"icon\":\"mdi:numeric-2-box\","
    "\"entity_category\":\"config\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/number/" + device_id + "_memory2_height/config").c_str(), 
                 memory2_config.c_str(), true);

  // 15. Memory 3 height number input
  String memory3_config = "{"
    "\"name\":\"Memory 3 Height\","
    "\"unique_id\":\"" + String(device_id) + "_memory3_height\","
    "\"state_topic\":\"" + String(topic_memory3_height_cm) + "\","
    "\"command_topic\":\"" + String(topic_set_memory3_height) + "\","
    "\"unit_of_measurement\":\"cm\","
    "\"min\":65,"
    "\"max\":120,"
    "\"step\":0.1,"
    "\"mode\":\"box\","
    "\"icon\":\"mdi:numeric-3-box\","
    "\"entity_category\":\"config\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/number/" + device_id + "_memory3_height/config").c_str(), 
                 memory3_config.c_str(), true);

  // 16. Memory 4 height number input
  String memory4_config = "{"
    "\"name\":\"Memory 4 Height\","
    "\"unique_id\":\"" + String(device_id) + "_memory4_height\","
    "\"state_topic\":\"" + String(topic_memory4_height_cm) + "\","
    "\"command_topic\":\"" + String(topic_set_memory4_height) + "\","
    "\"unit_of_measurement\":\"cm\","
    "\"min\":65,"
    "\"max\":120,"
    "\"step\":0.1,"
    "\"mode\":\"box\","
    "\"icon\":\"mdi:numeric-4-box\","
    "\"entity_category\":\"config\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/number/" + device_id + "_memory4_height/config").c_str(), 
                 memory4_config.c_str(), true);

  // 17. Child lock switch
  String child_lock_config = "{"
    "\"name\":\"Child Lock\","
    "\"unique_id\":\"" + String(device_id) + "_child_lock\","
    "\"state_topic\":\"" + String(topic_child_lock) + "\","
    "\"command_topic\":\"" + String(topic_set_child_lock) + "\","
    "\"payload_on\":\"ON\","
    "\"payload_off\":\"OFF\","
    "\"state_on\":\"ON\","
    "\"state_off\":\"OFF\","
    "\"icon\":\"mdi:lock\","
    "\"entity_category\":\"config\","
    + availability_json + "," + device_json + "}";
  client.publish(String(String(ha_discovery_prefix) + "/switch/" + device_id + "_child_lock/config").c_str(), 
                 child_lock_config.c_str(), true);

  mqttLog("Home Assistant discovery published");
}

// MQTT state publishing function (called from WiFi task)
void publishState() {
  if (client.connected()) {
    client.publish(topic_height, String(height).c_str());
    
    // Publish height in centimeters (with offset applied)
    float heightCm = ENCODER_TO_CM(height)/*  + heightOffsetCm */;
    client.publish(topic_height_cm, String(heightCm, 1).c_str());
    
    // Publish height as percentage for HA cover (0% = minHeightCm, 100% = maxHeightCm)
    int heightPercent = (int)(((heightCm - minHeightCm) / (maxHeightCm - minHeightCm)) * 100.0 + 0.5);
    heightPercent = constrain(heightPercent, 0, 100); // Ensure it's within 0-100
    client.publish(topic_height_percent, String(heightPercent).c_str());

    String statusStr = "";
    switch (state) {
      case State::OFF:
        statusStr = "stopped";
        break;
      case State::STARTING:
        statusStr = "opening";
        break;
      case State::UP:
        statusStr = "opening";
        break;
      case State::DOWN:
        statusStr = "closing";
        break;
      case State::STOPPING1:
      case State::STOPPING2:
      case State::STOPPING3:
      case State::STOPPING4:
        statusStr = "stopped";
        break;
      case State::STARTING_RECAL:
      case State::RECAL:
      case State::END_RECAL:
        statusStr = "closing";
        break;
    }
    client.publish(topic_status, statusStr.c_str());
  }
}

bool memory1PublishPending;
bool memory2PublishPending;
bool memory3PublishPending;
bool memory4PublishPending;

void publishMemoryHeights(bool force) {
  if (!client.connected()) {
    return;
  }

  if (force || memory1PublishPending) {
    float mem1Cm = ENCODER_TO_CM(memory1Height);
    client.publish(topic_memory1_height_cm, String(mem1Cm, 1).c_str(), true);
    memory1PublishPending = false;
  }

  if (force || memory2PublishPending) {
    float mem2Cm = ENCODER_TO_CM(memory2Height);
    client.publish(topic_memory2_height_cm, String(mem2Cm, 1).c_str(), true);
    memory2PublishPending = false;
  }

  if (force || memory3PublishPending) {
    float mem3Cm = ENCODER_TO_CM(memory3Height);
    client.publish(topic_memory3_height_cm, String(mem3Cm, 1).c_str(), true);
    memory3PublishPending = false;
  }

  if (force || memory4PublishPending) {
    float mem4Cm = ENCODER_TO_CM(memory4Height);
    client.publish(topic_memory4_height_cm, String(mem4Cm, 1).c_str(), true);
    memory4PublishPending = false;
  }
}

// Button polling task running on Core 0 (handles timing and edge detection)
void buttonPollTask(void* parameter) {
  bool up_pressed = false, up_pressed_tmp = false, last_up = false;
  bool down_pressed = false, down_pressed_tmp = false, last_down = false;
  bool mem1_pressed = false, mem1_pressed_tmp = false, last_mem1 = false;
  bool mem2_pressed = false, mem2_pressed_tmp = false, last_mem2 = false;
  bool both_pressed = false, both_pressed_tmp = false, last_both = false;
  int up_pressed_count = 0, up_released_count = 0;
  int down_pressed_count = 0, down_released_count = 0;
  int mem1_pressed_count = 0, mem1_released_count = 0;
  int mem2_pressed_count = 0, mem2_released_count = 0;
  
  unsigned long mem1_press_start = 0;
  unsigned long mem2_press_start = 0;
  bool mem1_long_sent = false;
  bool mem2_long_sent = false;
  
  unsigned long both_press_start = 0;
  bool both_press_sent = false;
  
  const unsigned long LONG_PRESS_MS = 5000;
  
  while (true) {
    // Read all buttons
    up_pressed_tmp = (digitalRead(UP_BTN) == LOW);
    down_pressed_tmp = (digitalRead(DOWN_BTN) == LOW);
    mem1_pressed_tmp = (digitalRead(MEM1_BTN) == LOW);
    mem2_pressed_tmp = (digitalRead(MEM2_BTN) == LOW);

    if(up_pressed_tmp) {
      up_pressed_count++;
      if(up_pressed_count >= BTN_DEBOUNCE_MS) {
        up_pressed = true;
        up_pressed_count = 0;
        up_released_count = 0;
      }
    } else {
      up_released_count++;
      if(up_released_count >= BTN_DEBOUNCE_MS) {
        up_pressed = false;
        up_pressed_count = 0;
        up_released_count = 0;
      }
    }

    if(down_pressed_tmp) {
      down_pressed_count++;
      if(down_pressed_count >= BTN_DEBOUNCE_MS) {
        down_pressed = true;
        down_pressed_count = 0;
        down_released_count = 0;
      }
    } else {
      down_released_count++;
      if(down_released_count >= BTN_DEBOUNCE_MS) {
        down_pressed = false;
        down_pressed_count = 0;
        down_released_count = 0;
      }
    }
    
    if(mem1_pressed_tmp) {
      mem1_pressed_count++;
      if(mem1_pressed_count >= BTN_DEBOUNCE_MS) {
        mem1_pressed = true;
        mem1_pressed_count = 0;
        mem1_released_count = 0;
      }
    } else {
      mem1_released_count++;
      if(mem1_released_count >= BTN_DEBOUNCE_MS) {
        mem1_pressed = false;
        mem1_pressed_count = 0;
        mem1_released_count = 0;
      }
    }
    
    if(mem2_pressed_tmp) {
      mem2_pressed_count++;
      if(mem2_pressed_count >= BTN_DEBOUNCE_MS) {
        mem2_pressed = true;
        mem2_pressed_count = 0;
        mem2_released_count = 0;
      }
    } else {
      mem2_released_count++;
      if(mem2_released_count >= BTN_DEBOUNCE_MS) {
        mem2_pressed = false;
        mem2_pressed_count = 0;
        mem2_released_count = 0;
      }
    }
    
    // Update volatile state for direct reading by Core 1
    button_up_pressed = up_pressed;
    button_down_pressed = down_pressed;
    button_mem1_pressed = mem1_pressed;
    button_mem2_pressed = mem2_pressed;
    
    // Check for both buttons pressed (takes priority)
    both_pressed = (up_pressed && down_pressed);
    
    ButtonCommand cmd;
    
    // Both buttons logic with duration check
    if (both_pressed && !last_both) {
      // Both buttons just pressed - start timing
      both_press_start = millis();
      both_press_sent = false;
    }
    if (both_pressed && !both_press_sent) {
      // Check if RESET_DURATION has elapsed
      if (millis() - both_press_start >= RESET_DURATION) {
        cmd = ButtonCommand::BOTH_PRESS;
        xQueueSend(buttonCommandQueue, &cmd, 0);
        both_press_sent = true;
      }
    }
    if (!both_pressed && last_both) {
      // Both buttons released - reset tracking
      both_press_sent = false;
      cmd = ButtonCommand::BOTH_RELEASE;
      xQueueSend(buttonCommandQueue, &cmd, 0);
    }
    
    // Only process individual buttons if both is not active
    if (!both_pressed && !last_both) {
      // UP button
      if (up_pressed && !last_up) {
        cmd = ButtonCommand::UP_PRESS;
        xQueueSend(buttonCommandQueue, &cmd, 0);
      } else if (!up_pressed && last_up) {
        cmd = ButtonCommand::UP_RELEASE;
        xQueueSend(buttonCommandQueue, &cmd, 0);
      }
      
      // DOWN button
      if (down_pressed && !last_down) {
        cmd = ButtonCommand::DOWN_PRESS;
        xQueueSend(buttonCommandQueue, &cmd, 0);
      } else if (!down_pressed && last_down) {
        cmd = ButtonCommand::DOWN_RELEASE;
        xQueueSend(buttonCommandQueue, &cmd, 0);
      }
      
      // MEM1 button with long press detection
      if (mem1_pressed && !last_mem1) {
        mem1_press_start = millis();
        mem1_long_sent = false;
      }
      if (mem1_pressed && !mem1_long_sent) {
        if (millis() - mem1_press_start >= LONG_PRESS_MS) {
          cmd = ButtonCommand::MEM1_LONG_PRESS;
          xQueueSend(buttonCommandQueue, &cmd, 0);
          mem1_long_sent = true;
        }
      }
      if (!mem1_pressed && last_mem1 && !mem1_long_sent) {
        cmd = ButtonCommand::MEM1_SHORT_PRESS;
        xQueueSend(buttonCommandQueue, &cmd, 0);
      }
      
      // MEM2 button with long press detection
      if (mem2_pressed && !last_mem2) {
        mem2_press_start = millis();
        mem2_long_sent = false;
      }
      if (mem2_pressed && !mem2_long_sent) {
        if (millis() - mem2_press_start >= LONG_PRESS_MS) {
          cmd = ButtonCommand::MEM2_LONG_PRESS;
          xQueueSend(buttonCommandQueue, &cmd, 0);
          mem2_long_sent = true;
        }
      }
      if (!mem2_pressed && last_mem2 && !mem2_long_sent) {
        cmd = ButtonCommand::MEM2_SHORT_PRESS;
        xQueueSend(buttonCommandQueue, &cmd, 0);
      }
    }
    
    last_up = up_pressed;
    last_down = down_pressed;
    last_mem1 = mem1_pressed;
    last_mem2 = mem2_pressed;
    last_both = both_pressed;
    
    // Poll every 1 millisecond
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// WiFi/MQTT task running on Core 0 (separate from LIN protocol on Core 1)
void wifiMqttTask(void* parameter) {
  // Connect to WiFi
  connectWiFi();

  // Setup MQTT with Last Will and Testament
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  
  // Set Last Will and Testament (LWT) for availability
  client.setBufferSize(2048); // Increase buffer for discovery messages
  
  connectMQTT();
  
  // Publish availability = online
  client.publish(topic_availability, "online", true);
  
  // Publish Home Assistant discovery configs
  delay(500); // Brief delay to ensure MQTT is stable
  publishHomeAssistantDiscovery();
  
  // Publish initial config values (only published at startup/reconnect and when changed)
  client.publish(topic_min_height_cm, String(minHeightCm).c_str(), true);
  client.publish(topic_max_height_cm, String(maxHeightCm).c_str(), true);
  // client.publish(topic_height_offset, String(heightOffsetCm).c_str(), true);
  client.publish(topic_child_lock, childLockEnabled ? "ON" : "OFF", true);
  publishMemoryHeights(true);
  
  // Publish initial state immediately so HA shows values right away
  publishState();

  unsigned long lastPublish = 0;
  unsigned long lastDiagnosticPublish = 0;
  State lastPublishedState = State::OFF;

  while (true) {
    // Process MQTT callbacks and maintain connection
    if (!client.connected()) {
      connectMQTT();
      // Re-publish availability and discovery on reconnect
      client.publish(topic_availability, "online", true);
      delay(500);
      publishHomeAssistantDiscovery();
      
      // Re-publish config values after reconnection
      client.publish(topic_min_height_cm, String(minHeightCm).c_str(), true);
      client.publish(topic_max_height_cm, String(maxHeightCm).c_str(), true);
      // client.publish(topic_height_offset, String(heightOffsetCm).c_str(), true);
      client.publish(topic_child_lock, childLockEnabled ? "ON" : "OFF", true);
      publishMemoryHeights(true);
      
      // Publish state immediately after reconnection
      publishState();
    }
    client.loop();

    // Process queued log messages
    char* logMsg;
    while (xQueueReceive(mqttLogQueue, &logMsg, 0) == pdTRUE) {
      if (client.connected() && logMsg != NULL) {
        client.publish(topic_log, logMsg);
        free(logMsg);
      }
    }

    unsigned long now = millis();
    
    // Publish state on change or every 30 seconds
    bool stateChanged = (state != lastPublishedState);
    bool timeToPublish = (now - lastPublish > 30000);

    if (stateChanged || timeToPublish) {
      publishState();
      lastPublishedState = state;
      lastPublish = now;
    }

    publishMemoryHeights();

    // Publish diagnostic sensors every 30 seconds
    if (now - lastDiagnosticPublish > 30000) {
      if (client.connected()) {
        // WiFi RSSI
        int rssi = WiFi.RSSI();
        client.publish(topic_wifi_rssi, String(rssi).c_str());
      }
      lastDiagnosticPublish = now;
    }

    // Small delay to prevent task from consuming too much CPU
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  pinMode(UP_BTN, INPUT_PULLUP);
  pinMode(DOWN_BTN, INPUT_PULLUP);
  pinMode(MEM1_BTN, INPUT_PULLUP);
  pinMode(MEM2_BTN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, !LED_ON);

  // Initialize EEPROM and load all settings
  EEPROM.begin(512);
  loadSettingsFromEEPROM();

  // Create queue for MQTT log messages
  mqttLogQueue = xQueueCreate(MQTT_LOG_QUEUE_SIZE, sizeof(char*));

  // Create queue for button commands
  buttonCommandQueue = xQueueCreate(BUTTON_COMMAND_QUEUE_SIZE, sizeof(ButtonCommand));

  // Create button polling task on Core 0 (high priority for responsive input)
  xTaskCreatePinnedToCore(
    buttonPollTask,    // Task function
    "ButtonPoll",      // Task name
    2048,              // Stack size (small task)
    NULL,              // Parameters
    2,                 // Priority (higher than WiFi/MQTT)
    NULL,              // Task handle
    0                  // Core 0
  );

  // Create WiFi/MQTT task on Core 0 (leaving Core 1 free for LIN protocol)
  xTaskCreatePinnedToCore(
    wifiMqttTask,      // Task function
    "WiFiMQTT",        // Task name
    8192,              // Stack size
    NULL,              // Parameters
    1,                 // Priority
    NULL,              // Task handle
    0                  // Core 0
  );

  // Initialize LIN communication (runs on Core 1 - main loop)
  mqttLog("Starting LIN initialization...");
  lin.begin(19200);
  linInit();
  mqttLog("LIN initialization completed");
}

// Button state tracking moved to Core 0 task (buttonPollTask)

void loop() {
  uint8_t empty[] = { 0, 0, 0 };
  uint8_t node_a[3] = { 0, 0, 0 };
  uint8_t node_b[3] = { 0, 0, 0 };
  uint8_t cmd[3] = { 0, 0, 0 };
  uint8_t res = 0;

  lin.send(17, empty, 3);
  delay_until(5);

  // Recv from PID 08
  res = lin.recv(8, node_a, 3);
  static uint8_t pid8FailCount = 0;
  if (res != 4) { // Retry once if failed
    delay_until(2);
    res = lin.recv(8, node_a, 3);
    if (res != 4) {
      pid8FailCount++;
      if (pid8FailCount >= LIN_COMM_FAILURE_THRESHOLD) {
        mqttLog("LIN communication error: Motor A (PID 8) failed repeatedly - restarting ESP32...");
        delay(1000);
        ESP.restart();
      }
    } else {
      pid8FailCount = 0;
    }
  } else {
    pid8FailCount = 0;
  }
  delay_until(5);

  // Recv from PID 09
  res = lin.recv(9, node_b, 3);
  static uint8_t pid9FailCount = 0;
  if (res != 4) { // Retry once if failed
    delay_until(2);
    res = lin.recv(9, node_b, 3);
    if (res != 4) {
      pid9FailCount++;
      if (pid9FailCount >= LIN_COMM_FAILURE_THRESHOLD) {
        mqttLog("LIN communication error: Motor B (PID 9) failed repeatedly - restarting ESP32...");
        delay(1000);
        ESP.restart();
      }
    } else {
      pid9FailCount = 0;
    }
  } else {
    pid9FailCount = 0;
  }
  delay_until(5);

  // Send PID 16, 6 times
  for (uint8_t i = 0; i < 6; i++) {
    delay_until(5);
    lin.send(16, 0, 0);
  }

  delay_until(5);
  // Send PID 1
  lin.send(1, 0, 0);

  uint16_t enc_a = node_a[0] | (node_a[1] << 8);
  uint16_t enc_b = node_b[0] | (node_b[1] << 8);
  uint16_t enc_min = min(enc_a, enc_b);
  uint16_t enc_max = max(enc_a, enc_b);
  uint16_t enc_target = (enc_a + enc_b) / 2; // Use average for target
  height = (enc_a + enc_b) / 2; // Use average for height display

  // Send PID 18
  switch (state) {
    case State::OFF:
      cmd[2] = LIN_CMD_IDLE; // 252
      break;
    case State::STARTING:
      cmd[2] = LIN_CMD_PREMOVE; // 196
      break;
    case State::UP:
      enc_target = min(enc_a, enc_b); // Working megadesk uses min for UP
      cmd[2] = LIN_CMD_RAISE; // 134
      lastState = State::UP; // Track for stopping
      // Debug: Log UP command
      // if (millis() % 2000 < 100) { // Log every 2 seconds when moving up
      //   mqttLog(("UP: enc_a=" + String(enc_a) + " enc_b=" + String(enc_b) + " target=" + String(enc_target) + " cmd=" + String(cmd[2])).c_str());
      // }
      break;
    case State::DOWN:
      enc_target = max(enc_a, enc_b); // Working megadesk uses max for DOWN
      cmd[2] = LIN_CMD_LOWER; // 133
      lastState = State::DOWN; // Track for stopping
      // Debug: Log DOWN command
      // if (millis() % 2000 < 100) { // Log every 2 seconds when moving down
      //   mqttLog(("DOWN: enc_a=" + String(enc_a) + " enc_b=" + String(enc_b) + " target=" + String(enc_target) + " cmd=" + String(cmd[2])).c_str());
      // }
      break;
    case State::STOPPING1:
    case State::STOPPING2:
    case State::STOPPING3:
      enc_target = targetHeight; // More accurate stops
      cmd[2] = LIN_CMD_FINE; // 135
      break;
    case State::STOPPING4:
      // Use actual encoder position based on last direction
      if (lastState == State::UP)
        enc_target = min(enc_a, enc_b);
      else
        enc_target = max(enc_a, enc_b);
      cmd[2] = LIN_CMD_FINISH; // 132
      break;
    case State::STARTING_RECAL:
      cmd[2] = LIN_CMD_PREMOVE; // 196 - Prepare for recalibration
      break;
    case State::RECAL:
      enc_target = 0; // Drive to bottom
      cmd[2] = LIN_CMD_RECALIBRATE; // 189 - Recalibrate command
      break;
    case State::END_RECAL:
      enc_target = 99; // Set minimum height
      cmd[2] = LIN_CMD_RECALIBRATE_END; // 188 - End recalibration
      break;
  }
  cmd[0] = enc_target & 0xFF;
  cmd[1] = enc_target >> 8;
  delay_until(5);
  lin.send(18, cmd, 3);

  // Auto-move to target height if set
  if (targetHeight > 0 && mqtt_command_active) {
    // Check if we've reached the target (within hysteresis)
    if (abs((int)height - (int)targetHeight) <= HYSTERESIS) {
      // Reached target! Start smooth stopping sequence
      user_cmd = Command::NONE;
      mqtt_command_active = false;
      // DON'T clear targetHeight yet - it's needed for smooth stopping in STOPPING1/2/3 states
      // It will be cleared when stopping completes (in State::OFF)
      digitalWrite(LED, !LED_ON);
      // mqttLog(("Target height reached: " + String(height)).c_str());
    } else if (targetHeight > height + HYSTERESIS) {
      // Need to go UP
      user_cmd = Command::UP;
    } else if (targetHeight < height - HYSTERESIS) {
      // Need to go DOWN
      user_cmd = Command::DOWN;
    }
  }

  // Read current button states from volatile vars (for continuous reads like UP/DOWN hold)
  bool up_pressed = button_up_pressed;
  bool down_pressed = button_down_pressed;
  
  // Process button commands from Core 0 queue
  ButtonCommand buttonCommand;
  while (xQueueReceive(buttonCommandQueue, &buttonCommand, 0) == pdTRUE) {
    // Only process if child lock allows
    if (childLockEnabled) continue;
    
    switch (buttonCommand) {
      case ButtonCommand::UP_PRESS:
        mqttLog("UP button pressed");
        if (mqtt_command_active) {
          // Interrupt MQTT movement
          if (targetHeight > height) {
            targetHeight = height + MOVE_OFFSET;
          } else {
            targetHeight = height - HYSTERESIS;
          }
          mqtt_command_active = false;
          mqttLog(("Physical override - smooth stop to " + String(targetHeight)).c_str());
        }
        break;
        
      case ButtonCommand::UP_RELEASE:
        mqttLog("UP button released");
        break;
        
      case ButtonCommand::DOWN_PRESS:
        mqttLog("DOWN button pressed");
        if (mqtt_command_active) {
          // Interrupt MQTT movement
          if (targetHeight > height) {
            targetHeight = height + MOVE_OFFSET;
          } else {
            targetHeight = height - HYSTERESIS;
          }
          mqtt_command_active = false;
          mqttLog(("Physical override - smooth stop to " + String(targetHeight)).c_str());
        }
        break;
        
      case ButtonCommand::DOWN_RELEASE:
        mqttLog("DOWN button released");
        break;
        
      case ButtonCommand::MEM1_SHORT_PRESS:
        mqttLog("MEM1 short press - recalling position");
        startMemoryMove(memory1Height, "MEM1");
        break;
        
      case ButtonCommand::MEM1_LONG_PRESS:
        mqttLog("MEM1 long press - storing position");
        memory1Height = (uint16_t)constrain((int)height, DANGER_MIN_HEIGHT, DANGER_MAX_HEIGHT);
        saveMemoryHeightToEEPROM(EEPROM_MEMORY1_SLOT, memory1Height);
        memory1PublishPending = true;
        mqttLog(("MEM1 stored: " + String(ENCODER_TO_CM(memory1Height), 1) + "cm").c_str());
        break;
        
      case ButtonCommand::MEM2_SHORT_PRESS:
        mqttLog("MEM2 short press - recalling position");
        startMemoryMove(memory2Height, "MEM2");
        break;
        
      case ButtonCommand::MEM2_LONG_PRESS:
        mqttLog("MEM2 long press - storing position");
        memory2Height = (uint16_t)constrain((int)height, DANGER_MIN_HEIGHT, DANGER_MAX_HEIGHT);
        saveMemoryHeightToEEPROM(EEPROM_MEMORY2_SLOT, memory2Height);
        memory2PublishPending = true;
        mqttLog(("MEM2 stored: " + String(ENCODER_TO_CM(memory2Height), 1) + "cm").c_str());
        break;
        
      case ButtonCommand::BOTH_PRESS:
        mqttLog("BOTH buttons pressed - reset starting");
        both_buttons_pressed = true;
        reset_press_start = millis();
        break;
        
      case ButtonCommand::BOTH_RELEASE:
        mqttLog("BOTH buttons released");
        if (both_buttons_pressed) {
          both_buttons_pressed = false;
          reset(false);
        }
        break;
    }
  }
  
  // Handle continuous UP/DOWN movement while held
  if (!childLockEnabled && !mqtt_command_active) {
    if (up_pressed && !down_pressed) {
      user_cmd = Command::UP;
      digitalWrite(LED, LED_ON);
    } else if (down_pressed && !up_pressed) {
      user_cmd = Command::DOWN;
      digitalWrite(LED, LED_ON);
    } else if (!up_pressed && !down_pressed) {
      user_cmd = Command::NONE;
      digitalWrite(LED, !LED_ON);
    }
  }
  
  // Check if reset duration reached
  if (both_buttons_pressed && (millis() - reset_press_start >= RESET_DURATION)) {
    reset(true);
  }

  // Check if motors are idle (from working megadesk)
  bool isIdle = (node_a[2] == 0 || node_a[2] == 37 || node_a[2] == 96) && 
                (node_b[2] == 0 || node_b[2] == 37 || node_b[2] == 96);
  
  // Debug: Log motor status periodically
  // if (millis() % 5000 < 100) { // Log every 5 seconds
  //   mqttLog(("Motor status - A: " + String(node_a[2]) + " B: " + String(node_b[2]) + " Idle: " + String(isIdle) + " Height: " + String(height)).c_str());
  // }

  // If child lock is enabled and desk is moving, stop it smoothly
  if (childLockEnabled && (state == State::STARTING || state == State::UP || state == State::DOWN || 
                           state == State::STARTING_RECAL || state == State::RECAL)) {
    user_cmd = Command::NONE;
    mqtt_command_active = false;
    if (state != State::STOPPING1 && state != State::STOPPING2 && 
        state != State::STOPPING3 && state != State::STOPPING4) {
      // Set targetHeight for smooth stop based on current direction
      if (state == State::UP || lastState == State::UP) {
        targetHeight = height + MOVE_OFFSET; // smoother deceleration when going up
      } else if (state == State::DOWN || lastState == State::DOWN) {
        targetHeight = height - HYSTERESIS; // smoother deceleration when going down
      } else {
        // For STARTING or unknown state, use current height
        targetHeight = height;
      }
      state = State::STOPPING1;
      // mqttLog("Child lock: Smooth stopping desk movement");
    }
  }

  switch (state) {
    case State::OFF:
      if (user_cmd == Command::RESET) {
        mqttLog("Starting recalibration sequence...");
        state = State::STARTING_RECAL;
      } else if (user_cmd != Command::NONE) {
        if (isIdle) {
          state = State::STARTING;
        }
      }
      break;
    case State::STARTING:
      switch(user_cmd) {
        case Command::NONE:
          state = State::OFF;
          mqtt_command_active = false; // Clear MQTT flag when stopping
          targetHeight = 0; // Clear target when movement cancelled before starting
          break;
        case Command::UP:
          if (height >= DANGER_MAX_HEIGHT) {
            mqttLog(("UP blocked: height=" + String(height) + " >= max=" + String(DANGER_MAX_HEIGHT)).c_str());
            state = State::OFF;
            mqtt_command_active = false;
            targetHeight = 0; // Clear target when blocked
          } else {
            state = State::UP;
          }
          break;
        case Command::DOWN:
          if (height <= DANGER_MIN_HEIGHT) {
            mqttLog(("DOWN blocked: height=" + String(height) + " <= min=" + String(DANGER_MIN_HEIGHT)).c_str());
            state = State::OFF;
            mqtt_command_active = false;
            targetHeight = 0; // Clear target when blocked
          } else {
            state = State::DOWN;
          }
          break;
        case Command::RESET:
          state = State::STARTING_RECAL;
          break;
      }
      break;
    case State::UP:
      if (user_cmd != Command::UP || enc_max >= DANGER_MAX_HEIGHT) {
        if (enc_max >= DANGER_MAX_HEIGHT) {
          mqttLog(("UP stopped: enc_max=" + String(enc_max) + " >= max=" + String(DANGER_MAX_HEIGHT)).c_str());
        }
        state = State::STOPPING1;
      }
      break;
    case State::DOWN:
      if (user_cmd != Command::DOWN || enc_min <= DANGER_MIN_HEIGHT) {
        if (enc_min <= DANGER_MIN_HEIGHT) {
          mqttLog(("DOWN stopped: enc_min=" + String(enc_min) + " <= min=" + String(DANGER_MIN_HEIGHT)).c_str());
        }
        state = State::STOPPING1;
      }
      break;
    case State::STOPPING1:
      state = State::STOPPING2;
      break;
    case State::STOPPING2:
      state = State::STOPPING3;
      break;
    case State::STOPPING3:
      state = State::STOPPING4;
      break;
    case State::STOPPING4:
      // Check if motors are idle (like megadesk)
      if (isIdle) {
        state = State::OFF;
        mqtt_command_active = false; // Clear MQTT flag when movement stops
        targetHeight = 0; // Clear target height now that smooth stop is complete
      }
      break;
    case State::STARTING_RECAL:
      state = State::RECAL;
      mqttLog("Recalibration: Driving to bottom...");
      break;
    case State::RECAL:
      // Wait for desk to reach bottom: enc_max <= 99 and both motors at status 1
      if (enc_max <= 99 && node_a[2] == 1 && node_b[2] == 1) {
        state = State::END_RECAL;
        mqttLog(("Recalibration: Reached bottom at " + String(enc_max)).c_str());
      }
      // Safety: if user cancels
      if (user_cmd != Command::RESET) {
        state = State::OFF;
        mqtt_command_active = false;
        targetHeight = 0; // Clear target on cancellation
        mqttLog("Recalibration cancelled");
      }
      break;
    case State::END_RECAL:
      state = State::OFF;
      targetHeight = enc_max; // Prevent immediately resuming previous height
      mqtt_command_active = false;
      mqttLog("Recalibration complete!");
      break;
    default:
      state = State::OFF;
      mqtt_command_active = false; // Clear MQTT flag on error
      targetHeight = 0; // Clear target on error
      break;
  }

  previous_state = state;

  // Wait the remaining 50 ms in the cycle (like working megadesk)
  delay_until(50);
}

void startMemoryMove(uint16_t memoryHeight, const char* label) {
  if (memoryHeight < DANGER_MIN_HEIGHT || memoryHeight > DANGER_MAX_HEIGHT) {
    mqttLog((String(label) + " recall ignored: stored value out of range (" + String(memoryHeight) + ")").c_str());
    return;
  }

  targetHeight = memoryHeight;
  mqtt_command_active = true;
  digitalWrite(LED, LED_ON);
  mqttLog((String(label) + " recall: moving to " + String(ENCODER_TO_CM(memoryHeight), 1) + "cm").c_str());
}

void processMemoryButton(bool pressed,
                         bool buttonsEnabled,
                         bool& prevPressed,
                         unsigned long& pressStart,
                         bool& longPressHandled,
                         uint16_t& memoryHeight,
                         uint16_t eepromSlot,
                         const char* label,
                         volatile bool& publishFlag) {
  if (!buttonsEnabled) {
    if (!pressed) {
      longPressHandled = false;
    }
    prevPressed = pressed;
    return;
  }

  if (pressed && !prevPressed) {
    pressStart = millis();
    longPressHandled = false;
    // mqttLog((String(label) + " button pressed").c_str());
  }

  if (pressed && !longPressHandled) {
    unsigned long held = millis() - pressStart;
    if (held >= MEMORY_LONG_PRESS_DURATION) {
      memoryHeight = (uint16_t)constrain((int)height, DANGER_MIN_HEIGHT, DANGER_MAX_HEIGHT);
      saveMemoryHeightToEEPROM(eepromSlot, memoryHeight);
      publishFlag = true;
      float storedCm = ENCODER_TO_CM(memoryHeight);
      mqttLog((String(label) + " stored height: " + String(storedCm, 1) + "cm").c_str());
      longPressHandled = true;
    }
  }

  if (!pressed && prevPressed) {
    // mqttLog((String(label) + " button released").c_str());
    if (!longPressHandled) {
      startMemoryMove(memoryHeight, label);
    }
  }

  if (!pressed) {
    longPressHandled = false;
  }

  prevPressed = pressed;
}

// Load all settings from EEPROM (height limits, offset, child lock)
void loadSettingsFromEEPROM() {
  uint16_t storedMin = EEPROM.readUShort(EEPROM_MIN_HEIGHT_SLOT);
  uint16_t storedMax = EEPROM.readUShort(EEPROM_MAX_HEIGHT_SLOT);
  // int16_t storedOffset = EEPROM.readShort(EEPROM_HEIGHT_OFFSET_SLOT);
  bool storedChildLock = EEPROM.readBool(EEPROM_CHILD_LOCK_SLOT);
  uint16_t storedMem1 = EEPROM.readUShort(EEPROM_MEMORY1_SLOT);
  uint16_t storedMem2 = EEPROM.readUShort(EEPROM_MEMORY2_SLOT);
  uint16_t storedMem3 = EEPROM.readUShort(EEPROM_MEMORY3_SLOT);
  uint16_t storedMem4 = EEPROM.readUShort(EEPROM_MEMORY4_SLOT);
  
  float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
  float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
  
  // Validate stored min height
  if (storedMin >= dangerMinCm && storedMin <= dangerMaxCm && storedMin > 0) {
    minHeightCm = storedMin;
  }
  
  // Validate stored max height
  if (storedMax >= dangerMinCm && storedMax <= dangerMaxCm && storedMax > minHeightCm) {
    maxHeightCm = storedMax;
  }
  
  // Validate stored offset
  // if (storedOffset >= -50 && storedOffset <= 50) {
  //   heightOffsetCm = storedOffset;
  // }
  
  // Load child lock (no validation needed for bool)
  childLockEnabled = storedChildLock;

  if (storedMem1 >= DANGER_MIN_HEIGHT && storedMem1 <= DANGER_MAX_HEIGHT) {
    memory1Height = storedMem1;
  } else {
    memory1Height = DANGER_MIN_HEIGHT;
  }

  if (storedMem2 >= DANGER_MIN_HEIGHT && storedMem2 <= DANGER_MAX_HEIGHT) {
    memory2Height = storedMem2;
  } else {
    memory2Height = DANGER_MAX_HEIGHT;
  }

  if (storedMem3 >= DANGER_MIN_HEIGHT && storedMem3 <= DANGER_MAX_HEIGHT) {
    memory3Height = storedMem3;
  } else {
    memory3Height = DANGER_MIN_HEIGHT;
  }

  if (storedMem4 >= DANGER_MIN_HEIGHT && storedMem4 <= DANGER_MAX_HEIGHT) {
    memory4Height = storedMem4;
  } else {
    memory4Height = DANGER_MAX_HEIGHT;
  }
}

// Save height limit to EEPROM
void saveHeightLimitToEEPROM(uint16_t slot, uint16_t value) {
  EEPROM.writeUShort(slot, value);
  EEPROM.commit();
}

void saveMemoryHeightToEEPROM(uint16_t slot, uint16_t value) {
  EEPROM.writeUShort(slot, value);
  EEPROM.commit();
}
