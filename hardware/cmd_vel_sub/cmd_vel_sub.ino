// ============================================================================
// Warehouse AMR - micro-ROS cmd_vel Subscriber
// ESP32 | micro-ROS Humble | Modbus ASCII Motor Driver
// ============================================================================

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// ─── Pin Config ───────────────────────────────────────────────────────────────
#define RX2_PIN 16
#define TX2_PIN 17

// ─── Robot Parameters ─────────────────────────────────────────────────────────
const float WHEEL_RADIUS = 0.05;   // meters
const float TRACK_WIDTH  = 0.25;   // meters (left-right wheel separation)
const float WHEEL_BASE   = 0.25;   // meters (front-rear wheel separation)
const float MAX_RPM      = 3000;
const float K_MECANUM    = (WHEEL_BASE + TRACK_WIDTH) / 2.0;  // combined kinematic constant

// ─── Motor Slave IDs (Modbus) ─────────────────────────────────────────────────
const byte MOTOR_FL = 2;
const byte MOTOR_FR = 1;
const byte MOTOR_RL = 3;
const byte MOTOR_RR = 7;

// ─── Timing ───────────────────────────────────────────────────────────────────
const unsigned long CMD_VEL_TIMEOUT_MS = 500;

// ─── Motor State Caching (avoid spamming 9600 baud serial) ────────────────────
struct MotorState {
  int last_rpm = -1;
  int last_dir = -1;
};
MotorState state_FL, state_FR, state_RL, state_RR;

unsigned long last_cmd_vel_time = 0;
bool motors_stopped = false;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

// ============================================================================
// MODBUS ASCII DRIVER
// ============================================================================

byte calculateLRC(byte *data, byte len) {
  byte sum = 0;
  for (int i = 0; i < len; i++) sum += data[i];
  return (byte)((~sum) + 1);
}

void sendModbusCommand(byte slave_id, byte fc, unsigned int address, unsigned int data) {
  byte frame[6];
  frame[0] = slave_id;
  frame[1] = fc;
  frame[2] = (address >> 8) & 0xFF;
  frame[3] = address & 0xFF;
  frame[4] = (data >> 8) & 0xFF;
  frame[5] = data & 0xFF;

  byte lrc = calculateLRC(frame, 6);

  char commandString[30];
  snprintf(commandString, sizeof(commandString), ":%02X%02X%04X%04X%02X\r\n",
           slave_id, fc, address, data, lrc);

  Serial2.print(commandString);
}

// ============================================================================
// MOTOR DRIVER
// ============================================================================

void driveMotor(byte slave_id, float linear_vel_m_s, bool invert_direction, MotorState &state) {
  // Convert m/s to RPM
  float target_rpm_float = (abs(linear_vel_m_s) / (2.0 * PI * WHEEL_RADIUS)) * 60.0;
  if (target_rpm_float > MAX_RPM) target_rpm_float = MAX_RPM;
  int rpm = (int)target_rpm_float;

  // Direction
  int dir = (linear_vel_m_s >= 0) ? 0 : 1;
  if (invert_direction) dir = (dir == 0) ? 1 : 0;

  if (abs(linear_vel_m_s) < 0.001) rpm = 0;

  // ── STOP ────────────────────────────────────────────────────────────────────
  if (rpm == 0 && state.last_rpm != 0) {
    sendModbusCommand(slave_id, 6, 2, 256); // Disable
    state.last_rpm = 0;
    state.last_dir = -1;  // FIX: reset dir so re-enable works next time
    return;
  }

  // ── DRIVE ───────────────────────────────────────────────────────────────────
  if (rpm > 0) {
    // Set speed FIRST, then enable — avoids stale RPM on enable edge
    if (abs(rpm - state.last_rpm) > 10 || state.last_dir == -1) {
      sendModbusCommand(slave_id, 6, 14, (unsigned int)rpm);
      state.last_rpm = rpm;
    }
    if (dir != state.last_dir) {
      sendModbusCommand(slave_id, 6, 2, (dir == 0) ? 257 : 265);
      state.last_dir = dir;
    }
  }
}

void stopAllMotors() {
  if (motors_stopped) return;  // FIX: only send once, not every loop cycle

  sendModbusCommand(MOTOR_FL, 6, 2, 256);
  sendModbusCommand(MOTOR_FR, 6, 2, 256);
  sendModbusCommand(MOTOR_RL, 6, 2, 256);
  sendModbusCommand(MOTOR_RR, 6, 2, 256);

  state_FL = {0, -1};
  state_FR = {0, -1};
  state_RL = {0, -1};
  state_RR = {0, -1};

  motors_stopped = true;
}

// ============================================================================
// ROS 2 CALLBACK & KINEMATICS
// ============================================================================

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_vel_time = millis();
  motors_stopped = false;

  float v_x = twist->linear.x;    // forward/backward
  float v_y = twist->linear.y;    // strafe left/right
  float w_z = twist->angular.z;   // rotation

  // Mecanum inverse kinematics
  float v_fl = v_x - v_y - K_MECANUM * w_z;
  float v_fr = v_x + v_y + K_MECANUM * w_z;
  float v_rl = v_x + v_y - K_MECANUM * w_z;
  float v_rr = v_x - v_y + K_MECANUM * w_z;

  driveMotor(MOTOR_FL, v_fl, false, state_FL);
  driveMotor(MOTOR_RL, v_rl, false, state_RL);
  driveMotor(MOTOR_FR, v_fr, true,  state_FR);
  driveMotor(MOTOR_RR, v_rr, true,  state_RR);
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Serial.begin(115200);
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_cmd_vel_node", "", &support);

  rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA);

  stopAllMotors();
  motors_stopped = true;
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
    stopAllMotors();
  }
}