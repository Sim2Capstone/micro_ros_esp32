#include "Config.h"
#define DEBUG_MOTOR
#define DEBUG_IMU

// Motor constants
float prev_yaw = .0f;
float curr_yaw = .0f;
unsigned long timer_cb_curr_time;
unsigned long timer_cb_prev_time;

unsigned long motor_cb_curr_time;
unsigned long motor_cb_prev_time;

unsigned long prev_time = 0;
unsigned long curr_time = 0;
float yaw_velocity = .0f;

struct Motor {
  int encoder;  // yellow
  int pwm;      // blue
  int dir;      // white
};

volatile int pulse_count1 = 0;
volatile int pulse_count2 = 0;

const Motor motor1 = { 18, 5, 19 };   // left
const Motor motor2 = { 12, 14, 27 };  // right

// ================================================================
// ===               ROS VARIABLES               ===
// ================================================================

rcl_publisher_t imu_pub;
rcl_subscription_t motor_cmd_sub;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState motor_cmd_msg;

rclc_executor_t executor;
rclc_executor_t act;
rclc_support_t support;
rcl_allocator_t allocator;  // micro ros
rcl_node_t node;
rcl_timer_t timer;

// ================================================================
// ===               MPU VARIABLES               ===
// ================================================================

MPU6050 mpu;
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;               // set true if DMP init was successful
uint8_t mpuIntStatus;                // holds actual interrupt status byte from MPU
uint8_t devStatus;                   // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                 // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                  // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];              // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Function prototypes
void setup_mpu();
void setup_motors();
void setup_uros();
void error_loop();
void timer_cb(rcl_timer_t *timer, int64_t last_call_time);
void motor_cmd_cb(const void *msgin);
void apply_torque(float torque, const Motor &motor);
void test_motors();

void setup() {
  // configure LED for output
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  digitalWrite(INTERNAL_LED_PIN, HIGH);
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  Serial.println(F("Initializing I2C devices..."));

  setup_mpu();
  setup_uros();
  setup_motors();
  prev_time = millis();
  timer_cb_curr_time = millis();
  motor_cb_curr_time = millis();
}

void loop() {
  delay(DELAY_MS);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(DELAY_MS)));
}

void update_imu_data() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}

void setup_mpu() {
  Serial.println("Setup : IMU");
  pinMode(INTERRUPT_PIN, INPUT);
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // TODO : IMPLEMENT ERROR  HANDLING MECHANISM

  // gyro offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // factory setting is 1688;

  // check if Initialization was successful
  if (devStatus != 0) {
    Serial.printf("DMP Init Failde code :%d", devStatus);
    //
  }
  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  // turn on the DMP, now that it's ready
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);

  // enable Arduino interrupt detection
  Serial.print("Enabling interrupt detection (Arduino external interrupt ");
  Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
  Serial.println("...");
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  // set our DMP Ready flag so the main loop() function knows it's okay to use it
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  dmpReady = true;

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
}

void setup_motors() {
  // left
  pinMode(motor1.encoder, INPUT_PULLUP);
  pinMode(motor2.encoder, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motor1.encoder), ISR_L_ENCODER_CB, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2.encoder), ISR_R_ENCODER_CB, RISING);

  ledcAttach(motor1.pwm, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motor2.pwm, PWM_FREQ, PWM_RESOLUTION);

  pinMode(motor1.dir, OUTPUT);
  pinMode(motor2.dir, OUTPUT);

  ledcWrite(motor1.pwm, 255);
  ledcWrite(motor2.pwm, 255);

  delay(1000);

  ledcWrite(motor1.pwm, 0);
  ledcWrite(motor2.pwm, 0);
}

void setup_uros() {
  // set_microros_transports();
  Serial.println("MICROROS Setup");
  set_microros_wifi_transports("IEEEwireless", "iEEE2023?", "192.168.0.135", 8888);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("1");

  const char *node_name = "esp_node";
  const char *node_namespace = "";
  RCCHECK(rclc_node_init_default(&node, node_name, node_namespace, &support));
  Serial.println("2");

  // initialize publisher
  const char *publisher_topic_name = "imu_data";
  RCCHECK(rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), publisher_topic_name));
  Serial.println("3");

  static micro_ros_utilities_memory_conf_t conf = { 0 };
  bool success = micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    &motor_cmd_msg,
    conf);
  Serial.printf("ALLOCATED MEMORY FOR SUBSCRIBER %d", success);

  // initialize subscriber
  const char *sub_name = "motor_cmd";
  // Initialize a reliable subscriber
  RCCHECK(rclc_subscription_init_default(&motor_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), sub_name));
  Serial.println("5");

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(DELAY_MS), timer_cb));
  Serial.println("6");

  // inittial executor and add the timer and subscriber
  const unsigned int num_handles = 2;
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  Serial.println("7");
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  Serial.println("8");
  // Add subscription to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_cmd_sub, &motor_cmd_msg, &motor_cmd_cb, ON_NEW_DATA));
  Serial.println("10");
  digitalWrite(INTERNAL_LED_PIN, LOW);
  Serial.println("MICROROS FINISHED");
}

void set_torque(float torque, const Motor &motor) {
  float speed = NO_LOAD_SPEED * (torque) / STALL_TORQUE;
  digitalWrite(motor.dir, (speed > 0) ? LOW : HIGH);
  int pwm = (int)((abs(speed) / NO_LOAD_SPEED * 150.f) + 95.f);
  ledcWrite(motor.pwm, pwm);
  delay(5);
}

// ================================================================
// ===               CALLBACKS                 ===
// ================================================================

void timer_cb(rcl_timer_t *timer, int64_t last_call_time) {

  if (!dmpReady || timer == NULL) {
    return;
  }

  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    return;
  }
  // timer_cb_curr_time = millis();

  // mpu.dmpGetQuaternion(&q, fifoBuffer);
  update_imu_data();
  curr_yaw = ypr[0];
  float dt = (timer_cb_curr_time - timer_cb_prev_time) * 0.001f;
  yaw_velocity = (curr_yaw - prev_yaw) / dt;

  imu_msg.orientation.x = q.x;
  imu_msg.orientation.y = q.y;
  imu_msg.orientation.z = q.z;
  imu_msg.orientation.w = q.w;
  imu_msg.angular_velocity.z = yaw_velocity;

#ifdef DEBUG_IMU
  // float time_step = (float)last_call_time * 0.000001f;
  // printf("Last callback time: %ld\n", time_step);
  Serial.printf(" IMU VALUES :\tTime step : %f\t Quaternions :%f:%f:%f:%f:%f\t%f\tAngular velocity:%f\n", dt, q.x, q.y, q.z, q.w, yaw_velocity);
#endif

  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
  prev_yaw = curr_yaw;
  // timer_cb_prev_time = timer_cb_curr_time;
}

void motor_cmd_cb(const void *msgin) {
  motor_cb_curr_time = millis();
  unsigned long dt = motor_cb_curr_time - motor_cb_prev_time * 0.001f;
  // float dt = (motor_cb_curr_time - motor_cb_prev_time) * 0.001f;

  // // Cast received message to used type
  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
  set_torque(msg->effort.data[0], motor1);
  set_torque(msg->effort.data[1], motor2);
#ifdef DEBUG_MOTOR
  Serial.printf("Motor callback : \t %lu\t%f\t%f\n", dt, msg->effort.data[0], msg->effort.data[1]);
    // Serial.println("MOTOR CALL BACK LOOP");

#endif
    motor_cb_prev_time = motor_cb_curr_time;
}

// ================================================================
// ===               ISRS               ===
// ================================================================

void dmpDataReady() {
  mpuInterrupt = true;
}
void ISR_L_ENCODER_CB() {
  pulse_count1++;
}

void ISR_R_ENCODER_CB() {
  pulse_count2;
}

void error_loop() {
  ledcWrite(motor1.pwm, 0);
  ledcWrite(motor2.pwm, 0);
  delay(50);
}

void test_imu() {

  if (!dmpReady) {
    return;
  }

  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    return;
  }
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  imu_msg.orientation.x = q.x;
  imu_msg.orientation.y = q.y;
  imu_msg.orientation.z = q.z;
  imu_msg.orientation.w = q.w;

  Serial.printf("IMU VALUES : %f\t%f\t%f\t%%f", q.x, q.y, q.z, q.w);
  // RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

void test_motors() {
  for (float i = -1.f; i < 1.f; i += 0.05f) {
    set_torque(i, motor1);
    set_torque(i, motor2);
    Serial.printf("Torque %f", i);
    delay(1000);
  }
  ledcWrite(motor1.pwm, 0);
  ledcWrite(motor2.pwm, 0);
}