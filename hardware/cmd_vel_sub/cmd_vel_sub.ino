// ============================================================================
// Warehouse AMR - ESP32 micro-ROS Full Hardware Controller
// 4x RMCS-2303 Modbus ASCII Mecanum Motors + MPU6050/I2C IMU + micro-ROS Humble
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

// ─── Pin Configurations ──────────────────────────────────────────────────────
#define RX2_PIN 16
#define TX2_PIN 17
#define SDA_PIN 21
#define SCL_PIN 22

// ─── Robot Physical Parameters ───────────────────────────────────────────────
const float WHEEL_RADIUS = 0.05;   // meters (50mm radius)
const float TRACK_WIDTH  = 0.25;   // meters (distance between left and right wheels)
const float WHEEL_BASE   = 0.25;   // meters (distance between front and rear wheels)
const float MAX_RPM      = 3000.0;
const float K_MECANUM    = (WHEEL_BASE + TRACK_WIDTH) / 2.0;

// ─── Modbus Slave IDs (RMCS-2303) ────────────────────────────────────────────
const byte MOTOR_FL = 2;
const byte MOTOR_FR = 1;
const byte MOTOR_RL = 3;
const byte MOTOR_RR = 7;

// ─── Timing ──────────────────────────────────────────────────────────────────
const unsigned long CMD_VEL_TIMEOUT_MS = 500;
const unsigned long ODOM_PUB_PERIOD_MS = 50;  // 20 Hz publish rate

// ─── Motor State Caching ─────────────────────────────────────────────────────
struct MotorState {
  int last_rpm = -1;
  int last_dir = -1;
  float current_rpm = 0.0;
};
MotorState state_FL, state_FR, state_RL, state_RR;

unsigned long last_cmd_vel_time = 0;
unsigned long last_odom_pub_time = 0;
bool motors_stopped = false;

// ─── micro-ROS Objects ───────────────────────────────────────────────────────
rcl_subscription_t sub_cmd_vel;
rcl_publisher_t    pub_wheel_odom;
rcl_publisher_t    pub_imu;

geometry_msgs__msg__Twist msg_cmd_vel;
nav_msgs__msg__Odometry   msg_odom;
sensor_msgs__msg__Imu     msg_imu;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

// ─── Simple IMU Variables (MPU6050 Default Register Map) ────────────────────
const int MPU_ADDR = 0x68;
float gyro_z_offset = 0.0;
unsigned long last_imu_time = 0;
float current_yaw = 0.0;

// ============================================================================
// MODBUS ASCII PROTOCOL HELPERS
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

  char commandString[32];
  snprintf(commandString, sizeof(commandString), ":%02X%02X%04X%04X%02X\r\n",
           slave_id, fc, address, data, lrc);

  Serial2.print(commandString);
}

// ============================================================================
// MOTOR DRIVE & VELOCITY COMMANDS
// ============================================================================

void driveMotor(byte slave_id, float linear_vel_m_s, bool invert_direction, MotorState &state) {
  float target_rpm_float = (abs(linear_vel_m_s) / (2.0 * PI * WHEEL_RADIUS)) * 60.0;
  if (target_rpm_float > MAX_RPM) target_rpm_float = MAX_RPM;
  int rpm = (int)target_rpm_float;

  int dir = (linear_vel_m_s >= 0) ? 0 : 1;
  if (invert_direction) dir = (dir == 0) ? 1 : 0;

  if (abs(linear_vel_m_s) < 0.001) {
    rpm = 0;
  }

  // STOP
  if (rpm == 0 && state.last_rpm != 0) {
    sendModbusCommand(slave_id, 6, 2, 256); // Disable / Coast
    state.last_rpm = 0;
    state.last_dir = -1;
    state.current_rpm = 0.0;
    return;
  }

  // DRIVE
  if (rpm > 0) {
    if (abs(rpm - state.last_rpm) > 10 || state.last_dir == -1) {
      sendModbusCommand(slave_id, 6, 14, (unsigned int)rpm);
      state.last_rpm = rpm;
    }
    if (dir != state.last_dir) {
      sendModbusCommand(slave_id, 6, 2, (dir == 0) ? 257 : 265);
      state.last_dir = dir;
    }
    state.current_rpm = (dir == 0) ? (float)rpm : -(float)rpm;
  }
}

void stopAllMotors() {
  if (motors_stopped) return;

  sendModbusCommand(MOTOR_FL, 6, 2, 256);
  sendModbusCommand(MOTOR_FR, 6, 2, 256);
  sendModbusCommand(MOTOR_RL, 6, 2, 256);
  sendModbusCommand(MOTOR_RR, 6, 2, 256);

  state_FL = {0, -1, 0.0};
  state_FR = {0, -1, 0.0};
  state_RL = {0, -1, 0.0};
  state_RR = {0, -1, 0.0};

  motors_stopped = true;
}

// ============================================================================
// IMU INITIALIZATION & READING (MPU6050 / I2C)
// ============================================================================

void initIMU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Wake up MPU6050
  Wire.endTransmission(true);

  // Calibrate Gyro Z offset
  long gz_sum = 0;
  const int CALIB_SAMPLES = 200;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    if (Wire.available() >= 2) {
      int16_t gz_raw = (Wire.read() << 8) | Wire.read();
      gz_sum += gz_raw;
    }
    delay(5);
  }
  gyro_z_offset = (float)gz_sum / (float)CALIB_SAMPLES;
  last_imu_time = millis();
}

void readAndPublishIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() >= 14) {
    int16_t ax_raw = (Wire.read() << 8) | Wire.read();
    int16_t ay_raw = (Wire.read() << 8) | Wire.read();
    int16_t az_raw = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Temperature (skip)
    int16_t gx_raw = (Wire.read() << 8) | Wire.read();
    int16_t gy_raw = (Wire.read() << 8) | Wire.read();
    int16_t gz_raw = (Wire.read() << 8) | Wire.read();

    float dt = (millis() - last_imu_time) / 1000.0;
    last_imu_time = millis();

    // MPU6050 scales: Accel +/-2g (16384 LSB/g), Gyro +/-250 deg/s (131.0 LSB/(deg/s))
    float ax = (float)ax_raw / 16384.0 * 9.80665;
    float ay = (float)ay_raw / 16384.0 * 9.80665;
    float az = (float)az_raw / 16384.0 * 9.80665;

    float gz_rad = ((float)gz_raw - gyro_z_offset) / 131.0 * (PI / 180.0);
    current_yaw += gz_rad * dt;

    // Populate ROS 2 Imu message
    msg_imu.header.frame_id.data = (char*)"imu_link";
    msg_imu.header.stamp.sec = millis() / 1000;
    msg_imu.header.stamp.nanosec = (millis() % 1000) * 1000000;

    // Convert Yaw to Quaternion [x, y, z, w]
    msg_imu.orientation.x = 0.0;
    msg_imu.orientation.y = 0.0;
    msg_imu.orientation.z = sin(current_yaw / 2.0);
    msg_imu.orientation.w = cos(current_yaw / 2.0);

    msg_imu.angular_velocity.z = gz_rad;
    msg_imu.linear_acceleration.x = ax;
    msg_imu.linear_acceleration.y = ay;
    msg_imu.linear_acceleration.z = az;

    rcl_publish(&pub_imu, &msg_imu, NULL);
  }
}

// ============================================================================
// MECANUM FORWARD KINEMATICS & ODOMETRY PUBLISHING
// ============================================================================

void publishWheelOdometry() {
  // Convert RPM to linear wheel speeds (m/s)
  float v_fl = (state_FL.current_rpm / 60.0) * (2.0 * PI * WHEEL_RADIUS);
  float v_fr = (state_FR.current_rpm / 60.0) * (2.0 * PI * WHEEL_RADIUS);
  float v_rl = (state_RL.current_rpm / 60.0) * (2.0 * PI * WHEEL_RADIUS);
  float v_rr = (state_RR.current_rpm / 60.0) * (2.0 * PI * WHEEL_RADIUS);

  // Mecanum Forward Kinematics
  float vx = (v_fl + v_fr + v_rl + v_rr) / 4.0;
  float vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0;
  float wz = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * K_MECANUM);

  msg_odom.header.frame_id.data = (char*)"odom";
  msg_odom.child_frame_id.data = (char*)"base_footprint";
  msg_odom.header.stamp.sec = millis() / 1000;
  msg_odom.header.stamp.nanosec = (millis() % 1000) * 1000000;

  msg_odom.twist.twist.linear.x = vx;
  msg_odom.twist.twist.linear.y = vy;
  msg_odom.twist.twist.angular.z = wz;

  // Covariance defaults for EKF
  msg_odom.twist.covariance[0]  = 0.001; // vx
  msg_odom.twist.covariance[7]  = 0.001; // vy
  msg_odom.twist.covariance[35] = 0.005; // vyaw

  rcl_publish(&pub_wheel_odom, &msg_odom, NULL);
}

// ============================================================================
// ROS 2 CALLBACKS
// ============================================================================

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_vel_time = millis();
  motors_stopped = false;

  float v_x = twist->linear.x;    // Forward / Backward
  float v_y = twist->linear.y;    // Strafe Left / Right
  float w_z = twist->angular.z;   // Turn Left / Right

  // Mecanum Inverse Kinematics
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
// SETUP & MAIN LOOP
// ============================================================================

void setup() {
  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Serial.begin(115200);

  initIMU();
  set_microros_transports();
  delay(1000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_amr_hw_node", "", &support);

  // 1. Subscribe to /cmd_vel
  rclc_subscription_init_best_effort(
    &sub_cmd_vel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  // 2. Publish /wheel/odometry
  rclc_publisher_init_best_effort(
    &pub_wheel_odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "wheel/odometry"
  );

  // 3. Publish /imu/data
  rclc_publisher_init_best_effort(
    &pub_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA);

  stopAllMotors();
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Safety Timeout for Motors
  if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
    stopAllMotors();
  }

  // Periodic Telemetry Publication (20 Hz)
  if (millis() - last_odom_pub_time >= ODOM_PUB_PERIOD_MS) {
    last_odom_pub_time = millis();
    readAndPublishIMU();
    publishWheelOdometry();
  }
}
