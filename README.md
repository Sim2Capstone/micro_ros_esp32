# Microros Esp32

## Overview


This project aims to provide a codebase that creates an abstraction layer around sensor reading and motor control using an esp32. 
By using this project, developers can avoid the complexities of an RTOS while efficiently managing sensor data and motor commands.
The main functionalities include reading IMU data, publishing sensor readings on ROS2 topics, and controlling motors based on received commands.



## Technologies

- Micro-ROS
- Arduino IDE
- Docker (not necessarly but easier to manager development)

## Hardware and Software Requirements

- ESP32 microcontroller
- IMPU (using mpu 6050)

## Installation

To install and use this project, follow these steps:

1. Clone the repository and initialize submodules:
    ```sh
    git clone --recurse-submodules https://github.com/Sim2Capstone/micro_ros_esp32.git
    cd  micro_ros_esp32    ```

2. If you've already cloned the repository and need to initialize the submodules, run:
    ```sh
    git submodule update --init --recursive
    ```

## Usage

