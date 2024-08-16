#ifndef CONFIG_H
#define CONFIG_H
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>  // ros2 client library
#include <rcl/error_handling.h>
#include <rclc/rclc.h>  // client lib for micro controllers
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
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


// CONSTANTS
#define INTERRUPT_PIN 2
#define INTERNAL_LED_PIN 13
#define DELAY_MS 5
#define TIMER_TIMEOUT_MS 1000




// MOTOR CONSTANTS
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50000;
const int PWM_RESOLUTION = 8;
const float NO_LOAD_SPEED = 7.50f;
const float STALL_TORQUE = 1.05f;


#endif

