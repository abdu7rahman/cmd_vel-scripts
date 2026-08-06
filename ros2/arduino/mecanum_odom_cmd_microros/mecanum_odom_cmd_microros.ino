// Mecanum base: /cmd_vel in, /odom out — micro-ROS port of
// odompub_cdmsub/odompub_cdmsub.ino and
// ros_controller___odom_publisher/ros_controller___odom_publisher.ino, which
// were the same node.
//
// Four wheels, four quadrature encoders, four H-bridge channels. Wheel order
// is front-left, front-right, rear-left, rear-right (1-4), matching the pin
// definitions carried over from the ROS 1 sketch.

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <micro_ros_utilities/string_utilities.h>

// Encoder pins.
const int encoderPin1A = 31, encoderPin1B = 33;
const int encoderPin2A = 37, encoderPin2B = 35;
const int encoderPin3A = 39, encoderPin3B = 41;
const int encoderPin4A = 45, encoderPin4B = 43;

// Motor pins: direction A, direction B, PWM enable.
const int motor1pin1 = 34, motor1pin2 = 36, ENA = 10;
const int motor2pin1 = 38, motor2pin2 = 40, ENB = 11;
const int motor3pin1 = 26, motor3pin2 = 28, ENC = 8;
const int motor4pin1 = 30, motor4pin2 = 32, END = 9;

const float wheelRadius = 0.05f;         // metres
const float robotRadius = 0.266f;        // metres, centre to wheel
const int pulsesPerRevolution = 1300;

// Stop the motors if no cmd_vel arrives within this window.
#define COMMAND_TIMEOUT_MS 500
#define ODOM_PERIOD_MS 50

volatile long encoderValue1 = 0, encoderValue2 = 0;
volatile long encoderValue3 = 0, encoderValue4 = 0;
long lastEncoderValue1 = 0, lastEncoderValue2 = 0;
long lastEncoderValue3 = 0, lastEncoderValue4 = 0;

float x = 0.0f, y = 0.0f, theta = 0.0f;

unsigned long last_command_ms = 0;

rcl_subscription_t cmd_vel_sub;
rcl_publisher_t odom_pub;
geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t odom_timer;

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

  // The ROS 1 sketch tested vx, then vy, then wz in an if/else chain, so the
  // base could only ever do one of translate-forward, strafe or spin at a time
  // and any diagonal command was silently reduced to its x component. Proper
  // mecanum mixing lets all three combine.
  float w1 = vx - vy - wz * robotRadius;  // front-left
  float w2 = vx + vy + wz * robotRadius;  // front-right
  float w3 = vx + vy - wz * robotRadius;  // rear-left
  float w4 = vx - vy + wz * robotRadius;  // rear-right

  // Normalise so the largest wheel command maps to full scale without clipping
  // the others out of proportion.
  float peak = max(max(fabs(w1), fabs(w2)), max(fabs(w3), fabs(w4)));
  float scale = 255.0f;
  if (peak > 1.0f) {
    scale = 255.0f / peak;
  }

  int s1 = constrain((int)(fabs(w1) * scale), 0, 255);
  int s2 = constrain((int)(fabs(w2) * scale), 0, 255);
  int s3 = constrain((int)(fabs(w3) * scale), 0, 255);
  int s4 = constrain((int)(fabs(w4) * scale), 0, 255);

  M1(w1 >= 0 ? HIGH : LOW, w1 >= 0 ? LOW : HIGH, s1);
  M2(w2 >= 0 ? HIGH : LOW, w2 >= 0 ? LOW : HIGH, s2);
  M3(w3 >= 0 ? HIGH : LOW, w3 >= 0 ? LOW : HIGH, s3);
  M4(w4 >= 0 ? HIGH : LOW, w4 >= 0 ? LOW : HIGH, s4);
}

// The ROS 1 sketch computed and published odometry straight out of loop() with
// a delay(1), so it flooded the link at ~1 kHz and each integration step used a
// dt of roughly one millisecond regardless of how long the pass actually took.
// A timer gives a fixed 20 Hz and a known interval.
void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) {
    return;
  }

  noInterrupts();
  long e1 = encoderValue1, e2 = encoderValue2;
  long e3 = encoderValue3, e4 = encoderValue4;
  interrupts();

  long dEncoder1 = e1 - lastEncoderValue1;
  long dEncoder2 = e2 - lastEncoderValue2;
  long dEncoder3 = e3 - lastEncoderValue3;
  long dEncoder4 = e4 - lastEncoderValue4;

  lastEncoderValue1 = e1;
  lastEncoderValue2 = e2;
  lastEncoderValue3 = e3;
  lastEncoderValue4 = e4;

  float dS1 = (dEncoder1 * 2.0f * PI * wheelRadius) / pulsesPerRevolution;
  float dS2 = (dEncoder2 * 2.0f * PI * wheelRadius) / pulsesPerRevolution;
  float dS3 = (dEncoder3 * 2.0f * PI * wheelRadius) / pulsesPerRevolution;
  float dS4 = (dEncoder4 * 2.0f * PI * wheelRadius) / pulsesPerRevolution;

  float dX = (dS1 + dS2 + dS3 + dS4) / 4.0f;
  float dY = (-dS1 + dS2 - dS3 + dS4) / 4.0f;
  float dTheta = (-dS1 + dS2 + dS3 - dS4) / (4.0f * robotRadius);

  // Integrate in the body frame at the midpoint heading, which is a closer
  // approximation over a finite step than using the start heading alone.
  float midTheta = theta + dTheta / 2.0f;
  x += dX * cos(midTheta) - dY * sin(midTheta);
  y += dX * sin(midTheta) + dY * cos(midTheta);
  theta += dTheta;

  // Keep theta bounded so it does not lose float precision over a long run.
  while (theta > PI) theta -= 2.0f * PI;
  while (theta < -PI) theta += 2.0f * PI;

  float dt = ODOM_PERIOD_MS / 1000.0f;

  int64_t now = rmw_uros_epoch_nanos();
  odom.header.stamp.sec = (int32_t)(now / 1000000000LL);
  odom.header.stamp.nanosec = (uint32_t)(now % 1000000000LL);

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  // tf::createQuaternionFromYaw has no ROS 2 equivalent on the firmware side.
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(theta / 2.0f);
  odom.pose.pose.orientation.w = cos(theta / 2.0f);

  // The ROS 1 sketch put per-step displacements in the twist fields, which are
  // specified as velocities. Divide by the step to get m/s and rad/s.
  odom.twist.twist.linear.x = dX / dt;
  odom.twist.twist.linear.y = dY / dt;
  odom.twist.twist.angular.z = dTheta / dt;

  RCSOFTCHECK(rcl_publish(&odom_pub, &odom, NULL));
}

void updateEncoder1() {
  static int lastEncoded1 = 0;
  int encoded = (digitalRead(encoderPin1A) << 1) | digitalRead(encoderPin1B);
  int sum = (lastEncoded1 << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue1++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue1--;
  lastEncoded1 = encoded;
}

void updateEncoder2() {
  static int lastEncoded2 = 0;
  int encoded = (digitalRead(encoderPin2A) << 1) | digitalRead(encoderPin2B);
  int sum = (lastEncoded2 << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2--;
  lastEncoded2 = encoded;
}

void updateEncoder3() {
  static int lastEncoded3 = 0;
  int encoded = (digitalRead(encoderPin3A) << 1) | digitalRead(encoderPin3B);
  int sum = (lastEncoded3 << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue3++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue3--;
  lastEncoded3 = encoded;
}

void updateEncoder4() {
  static int lastEncoded4 = 0;
  int encoded = (digitalRead(encoderPin4A) << 1) | digitalRead(encoderPin4B);
  int sum = (lastEncoded4 << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue4++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue4--;
  lastEncoded4 = encoded;
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

  // The ROS 1 sketch called M1(1, 1, 255) here, driving both H-bridge inputs
  // high at full PWM. On most drivers that is a brake-to-Vcc, not a stop.
  stop_all();

  pinMode(encoderPin1A, INPUT_PULLUP); pinMode(encoderPin1B, INPUT_PULLUP);
  pinMode(encoderPin2A, INPUT_PULLUP); pinMode(encoderPin2B, INPUT_PULLUP);
  pinMode(encoderPin3A, INPUT_PULLUP); pinMode(encoderPin3B, INPUT_PULLUP);
  pinMode(encoderPin4A, INPUT_PULLUP); pinMode(encoderPin4B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPin1A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1B), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2B), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3A), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3B), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin4A), updateEncoder4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin4B), updateEncoder4, CHANGE);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "mecanum_base_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
      &odom_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

  RCCHECK(rclc_timer_init_default(&odom_timer, &support,
                                  RCL_MS_TO_NS(ODOM_PERIOD_MS),
                                  odom_timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                         &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

  odom.header.frame_id =
      micro_ros_string_utilities_set(odom.header.frame_id, "odom");
  odom.child_frame_id =
      micro_ros_string_utilities_set(odom.child_frame_id, "base_link");

  last_command_ms = millis();
  rmw_uros_sync_session(1000);
}

void loop() {
  // No watchdog in the ROS 1 sketch: if the host went away the base kept
  // driving at whatever it was last told.
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    stop_all();
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
