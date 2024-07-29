// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


// ================================================================
// ===              MOTOR CONSTANTS              ===
// ================================================================
//Bonus info: Pulse/rotation = 83.5-84
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50000;
const int PWM_RESOLUTION = 8;
const float NO_LOAD_SPEED = 7.50f;
const float STALL_TORQUE = 1.05f;

//Pins
//Motor 1
const int encoderPin1 = 15;  //Yellow
const int PWMPin1 = 5;       //Blue
const int dirPin1 = 0;       //White

//Motor 2
const int encoderPin2 = 12;  //Yellow
const int PWMPin2 = 14;      //Blue
const int dirPin2 = 27;      //White


volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;

bool dir = true;

long int lastPulse1 = 0;
long int lastPulse2 = 0;
double inSpeed1 = 0;
double inSpeed2 = 0;


//For timed interrupts in loop
long int pastMillis = 0;
long int currentMillis = 0;
const unsigned period = 20;

// ================================================================
// ===               ROS DEPENDENCIES              ===
// ================================================================
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>  // ros2 client library
#include <rcl/error_handling.h>
#include <rclc/rclc.h>  // client lib for micro controllers
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/float32_multi_array.h>

// ================================================================
// ===              IMU SETUP              ===
// ================================================================

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

#define OUTPUT_READABLE_QUATERNION

//MPU CONSTATANT
#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;



// ROS CONSTANTS
#define DELAY_MS 100
#define TIMER_TIMEOUT_MS 1000



// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };




// ================================================================
// ===               ROS SETUP               ===
// ================================================================

rcl_publisher_t publisher;
rcl_publisher_t publisher2;
rcl_subscription_t test_sub;
rcl_subscription_t sub_motor_command;
sensor_msgs__msg__Imu imu_msg;
//std_msgs__msg__Float32MultiArray msg;
std_msgs__msg__Int64 msg;
sensor_msgs__msg__JointState motor_command_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;  // micro ros
rcl_node_t node;
rcl_timer_t timer;

/*
    Exectue given function, check its return value and call the error loop if the function failed
 */
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }


void error_loop() {
  // while (1) {
  //   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  //   delay(500);
  // }
}
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

int test_val = 69;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  // not used but kept for debugging purpose
  RCLC_UNUSED(last_call_time);
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    if (timer != NULL) {
      imu_msg.orientation.x = q.x;
      imu_msg.orientation.y = q.y;
      imu_msg.orientation.z = q.z;
      imu_msg.orientation.w = q.w;
      RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
    }
    // blink LED to indicate activity
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}


void subscription_callback(const void *msgin) {
  Serial.println("Q");
  //const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  const std_msgs__msg__Int64 *msg = (const std_msgs__msg__Int64 *)msgin;
  Serial.print(msg->data);
  digitalWrite(LED_PIN, (msg->data == 0)  ? LOW : HIGH);
}

// Implementation example:s
void motor_command_callback(const void *msgin) {
  Serial.println("F");
  // // Cast received message to used type
  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
  Serial.println(msg->effort.data[0]);

  // test_val = (int)msg->velocity.data[0];

  // apply_torque(msg->velocity.data[0], PWMPin1, dirPin1);
  // apply_torque(msg->velocity.data[1], PWMPin2, dirPin2);
}

void setup() {
  // ================================================================
  // ===                       IMU SETUP                       ===
  // ================================================================

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif


  Serial.begin(115200);
  while (!Serial)
    ;  // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()) // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // ================================================================
  // ===                       ROS SETUP                       ===
  // ================================================================
  //set_microros_transports();
  set_microros_wifi_transports("IEEEwireless", "iEEE2023?", "192.168.0.131", 8888);

  digitalWrite(LED_PIN, HIGH);
  delay(200);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("1");

  const char *node_name = "esp_node";
  const char *node_namespace = "";
  RCCHECK(rclc_node_init_default(&node, node_name, node_namespace, &support));
  Serial.println("2");

  //initialize publisher
  const char *publisher_topic_name = "imu_data";
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), publisher_topic_name));
  Serial.println("3");

  RCCHECK(rclc_publisher_init_default(&publisher2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "test"));
  Serial.println("4");

  //initialize subscriber
  const char *sub_name = "/motor_command";
  // Initialize a reliable subscriber
  RCCHECK(rclc_subscription_init_default(&sub_motor_command, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), sub_name));
  Serial.println("5");

  // // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &test_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "micro_ros_arduino_subscriber"));
    Serial.println("6");


  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(TIMER_TIMEOUT_MS), timer_callback));
  Serial.println("7");
  //inittial executor and add the timer and subscriber
  const unsigned int num_handles = 2;
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  Serial.println("8");
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  Serial.println("9");
  // Add subscription to the executor
  //RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor_command, &motor_command_msg, &motor_command_callback, ON_NEW_DATA));
  Serial.println("10");
  RCCHECK(rclc_executor_add_subscription(&executor, &test_sub, &msg, &subscription_callback, ON_NEW_DATA));
  Serial.println("11");

  // ================================================================
  // ===                     MOTOR SETUP                       ===
  // ================================================================

  pinMode(encoderPin1, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), countPulse1, RISING);
  pinMode(dirPin1, OUTPUT);
  digitalWrite(dirPin1, HIGH);

  pinMode(encoderPin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), countPulse2, RISING);
  pinMode(dirPin2, OUTPUT);
  digitalWrite(dirPin2, HIGH);

  ledcAttach(PWMPin1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMPin2, PWM_FREQ, PWM_RESOLUTION);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  delay(DELAY_MS);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(DELAY_MS)));
}

void apply_torque(float torque, int pwm_pin, int dir_pin) {
  float speed = NO_LOAD_SPEED * (torque) / STALL_TORQUE;
  digitalWrite(dir_pin, (speed > 0) ? LOW : HIGH);

  int pwm = (int)((abs(speed) / NO_LOAD_SPEED * 170.f) + 85.f);
  ledcWrite(pwm_pin, pwm);
  delay(5);
}


void countPulse1() {
  pulseCount1++;
}

void countPulse2() {
  pulseCount2++;
}
