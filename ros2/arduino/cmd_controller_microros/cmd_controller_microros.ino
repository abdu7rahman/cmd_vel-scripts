// Four-motor base driven from /cmd_vel — micro-ROS port of
// cmd_controller/cmd_controller.ino.
//
// This is the drive-only sketch: no encoders, no odometry. Use
// mecanum_odom_cmd_microros if you want /odom as well.

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

const int motor1pin1 = 34, motor1pin2 = 36, ENA = 10;
const int motor2pin1 = 38, motor2pin2 = 40, ENB = 11;
const int motor3pin1 = 26, motor3pin2 = 28, ENC = 8;
const int motor4pin1 = 30, motor4pin2 = 32, END = 9;

// Centre-to-wheel distance, metres. Scales the yaw term against the linear one.
const float robotRadius = 0.266f;

#define COMMAND_TIMEOUT_MS 500

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

unsigned long last_command_ms = 0;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void M1(int dir1, int dir2, int speed) {
  digitalWrite(motor1pin1, dir1);
  digitalWrite(motor1pin2, dir2);
  analogWrite(ENA, speed);
}

void M2(int dir1, int dir2, int speed) {
  digitalWrite(motor2pin1, dir1);
  digitalWrite(motor2pin2, dir2);
  analogWrite(ENB, speed);
}

void M3(int dir1, int dir2, int speed) {
  digitalWrite(motor3pin1, dir1);
  digitalWrite(motor3pin2, dir2);
  analogWrite(ENC, speed);
}

void M4(int dir1, int dir2, int speed) {
  digitalWrite(motor4pin1, dir1);
  digitalWrite(motor4pin2, dir2);
  analogWrite(END, speed);
}

static void stop_all() {
  M1(LOW, LOW, 0);
  M2(LOW, LOW, 0);
  M3(LOW, LOW, 0);
  M4(LOW, LOW, 0);
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *cmd =
      (const geometry_msgs__msg__Twist *)msgin;

  last_command_ms = millis();

  float vx = cmd->linear.x;
  float vy = cmd->linear.y;
  float wz = cmd->angular.z;

  // The ROS 1 sketch used an if/else chain on vx, then vy, then wz, so only one
  // axis could act at a time and a diagonal command lost its y component
  // entirely. Mixing all three lets the base move the way it was asked to.
  float w1 = vx - vy - wz * robotRadius;
  float w2 = vx + vy + wz * robotRadius;
  float w3 = vx + vy - wz * robotRadius;
  float w4 = vx - vy + wz * robotRadius;

  float peak = max(max(fabs(w1), fabs(w2)), max(fabs(w3), fabs(w4)));
  float scale = (peak > 1.0f) ? (255.0f / peak) : 255.0f;

  M1(w1 >= 0 ? HIGH : LOW, w1 >= 0 ? LOW : HIGH,
     constrain((int)(fabs(w1) * scale), 0, 255));
  M2(w2 >= 0 ? HIGH : LOW, w2 >= 0 ? LOW : HIGH,
     constrain((int)(fabs(w2) * scale), 0, 255));
  M3(w3 >= 0 ? HIGH : LOW, w3 >= 0 ? LOW : HIGH,
     constrain((int)(fabs(w3) * scale), 0, 255));
  M4(w4 >= 0 ? HIGH : LOW, w4 >= 0 ? LOW : HIGH,
     constrain((int)(fabs(w4) * scale), 0, 255));
}

void setup() {
  set_microros_transport();
  pinMode(LED_BUILTIN, OUTPUT);

  int motor_pins[] = {
    motor1pin1, motor1pin2, ENA, motor2pin1, motor2pin2, ENB,
    motor3pin1, motor3pin2, ENC, motor4pin1, motor4pin2, END,
  };
  for (unsigned int i = 0; i < sizeof(motor_pins) / sizeof(motor_pins[0]); i++) {
    pinMode(motor_pins[i], OUTPUT);
  }

  // The ROS 1 sketch opened with M1(1, 1, 255) — both inputs high at full PWM,
  // which brakes rather than stops on most drivers.
  stop_all();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "cmd_controller_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                         &cmd_vel_callback, ON_NEW_DATA));

  last_command_ms = millis();
}

void loop() {
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    stop_all();
  }
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
