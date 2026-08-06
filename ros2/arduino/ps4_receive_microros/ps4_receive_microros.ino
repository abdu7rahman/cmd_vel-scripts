// Four-motor base driven from four one-sided PWM channels — micro-ROS port of
// ps4_receive_ros/ps4_receive_ros.ino.
//
// Pairs with the joy_pwm_array node. The message carries
// [left, right, forward, back], each 0..pwm_max, and the largest channel wins.

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64_multi_array.h>

const int motor1pin1 = 34, motor1pin2 = 36, ENA = 10;
const int motor2pin1 = 38, motor2pin2 = 40, ENB = 11;
const int motor3pin1 = 26, motor3pin2 = 28, ENC = 8;
const int motor4pin1 = 30, motor4pin2 = 32, END = 9;

// Ignore channels below this, matching the ROS 1 threshold.
#define PWM_DEADBAND 40
#define COMMAND_TIMEOUT_MS 500

rcl_subscription_t cmd_sub;
std_msgs__msg__Int64MultiArray cmd_msg;
int64_t cmd_buffer[4];

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
  // The ROS 1 sketch's "stop" branch called M1(1, 1, 255) — both direction pins
  // high at full PWM. That is a brake command on most H-bridges, not a stop,
  // and it ran at boot too.
  M1(LOW, LOW, 0);
  M2(LOW, LOW, 0);
  M3(LOW, LOW, 0);
  M4(LOW, LOW, 0);
}

void cmd_callback(const void *msgin) {
  const std_msgs__msg__Int64MultiArray *msg =
      (const std_msgs__msg__Int64MultiArray *)msgin;

  if (msg->data.size < 4) {
    return;
  }

  last_command_ms = millis();

  int l = (int)msg->data.data[0];
  int r = (int)msg->data.data[1];
  int f = (int)msg->data.data[2];
  int b = (int)msg->data.data[3];

  // Pick the strongest channel rather than taking the first over threshold, so
  // a partly-deflected stick does not get overridden by whichever axis happens
  // to be tested first.
  int strongest = max(max(l, r), max(f, b));

  if (strongest < PWM_DEADBAND) {
    stop_all();
  } else if (strongest == l) {
    M1(LOW, HIGH, l); M2(LOW, HIGH, l); M3(HIGH, LOW, l); M4(HIGH, LOW, l);
  } else if (strongest == r) {
    M1(HIGH, LOW, r); M2(HIGH, LOW, r); M3(LOW, HIGH, r); M4(LOW, HIGH, r);
  } else if (strongest == f) {
    M1(HIGH, LOW, f); M2(LOW, HIGH, f); M3(LOW, HIGH, f); M4(HIGH, LOW, f);
  } else {
    M1(LOW, HIGH, b); M2(HIGH, LOW, b); M3(HIGH, LOW, b); M4(LOW, HIGH, b);
  }
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

  stop_all();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "ps4_receive_node", "", &support));

  // The ROS 1 sketch subscribed to `cmd_vel` with an Int64MultiArray, which
  // collides with the conventional Twist meaning of that topic name.
  RCCHECK(rclc_subscription_init_default(
      &cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
      "cmd_vel_pwm"));

  // Sequence storage has to be supplied before the first message arrives.
  cmd_msg.data.data = cmd_buffer;
  cmd_msg.data.size = 0;
  cmd_msg.data.capacity = 4;
  cmd_msg.layout.dim.data = NULL;
  cmd_msg.layout.dim.size = 0;
  cmd_msg.layout.dim.capacity = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg,
                                         &cmd_callback, ON_NEW_DATA));

  last_command_ms = millis();
}

void loop() {
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    stop_all();
  }
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
