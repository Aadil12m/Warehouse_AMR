#include <Arduino.h>

// Pins for Serial2 on ESP32
#define RX2_PIN 16
#define TX2_PIN 17

const byte slaveIDs[] = {7, 1, 2, 3};
const int numMotors = 4;

// --- Helper Functions for Modbus ASCII ---

// Calculate Longitudinal Redundancy Check (LRC)
byte calculateLRC(byte *data, byte len) {
  byte sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return (byte)((~sum) + 1);
}

// Format and send Modbus ASCII command
void sendModbusCommand(byte slave_id, byte fc, unsigned int address, unsigned int data) {
  byte frame[6];
  frame[0] = slave_id;
  frame[1] = fc;
  frame[2] = (address >> 8) & 0xFF;
  frame[3] = address & 0xFF;
  frame[4] = (data >> 8) & 0xFF;
  frame[5] = data & 0xFF;

  byte lrc = calculateLRC(frame, 6);

  char msg[30];
  snprintf(msg, sizeof(msg), ":%02X%02X%04X%04X%02X\r\n", 
           slave_id, fc, address, data, lrc);

  Serial2.print(msg);
}

// --- Motor Control Commands ---

void setSpeed(byte id, unsigned int speed) {
  sendModbusCommand(id, 6, 14, speed); // Register 14 = Speed
}

void enableDigitalMode(byte id, byte dir) {
  unsigned int data = (dir == 0) ? 257 : 265;
  sendModbusCommand(id, 6, 2, data);  // Register 2 = Mode
}

void brakeMotor(byte id, byte dir) {
  unsigned int data = (dir == 0) ? 260 : 268;
  sendModbusCommand(id, 6, 2, data);
}

void disableDigitalMode(byte id, byte dir) {
  unsigned int data = (dir == 0) ? 256 : 264;
  sendModbusCommand(id, 6, 2, data);
}

// --- Arduino Setup & Loop ---

void setup() {
  Serial.begin(115200);
  
  // Hardware Serial 2 configuration for ESP32
  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);

  Serial.println("\n=== ESP32 Native RMCS-2303 Motor Controller ===");
  delay(1000);
}

void loop() {
  for (int i = 0; i < numMotors; i++) {
    byte id = slaveIDs[i];

    Serial.print(">>> Testing Motor Slave ID: ");
    Serial.println(id);

    // 1. Forward
    Serial.println("   Running Forward...");
    setSpeed(id, 4000);
    enableDigitalMode(id, 0);
    delay(3000);

    // 2. Brake
    Serial.println("   Braking...");
    brakeMotor(id, 0);
    delay(2000);

    // 3. Reverse
    Serial.println("   Running Reverse...");
    enableDigitalMode(id, 1);
    delay(3000);

    // 4. Disable
    Serial.println("   Disabling Motor...");
    disableDigitalMode(id, 1);
    delay(2000);
  }

  Serial.println("Cycle complete. Waiting 5 seconds...\n");
  delay(5000);
}