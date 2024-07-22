

# Customized Tea Maker Project

## Overview
The customized tea maker is an innovative device designed to enhance the tea brewing experience. By combining mechanical structures and electronic control, this system automates the tea-making process, allowing users to customize their tea preferences. Whether you're a tea enthusiast or simply looking for a fun project, this DIY tea maker is worth exploring.

## Features
1. **Customization**: The microcontroller enables users to tailor their tea according to personal preferences. With a fully automated process, there's no need to wait in queues.
2. **Heating System**: The tea maker heats water to the optimal temperature. When an order is selected, a temperature sensor ensures the water reaches the desired level.
3. **Flask System**: Prevents heat loss during the brewing process.
4. **Automatic Cup Releasing**: An IR sensor detects cup placement. If a cup is not properly positioned, the Bluetooth module signals the automated cup-releasing system to release the necessary cups.
5. **Tea Separation Mechanism**:
   - Step 1: Initial condition with closed separation blades.
   - Step 2: Upper blade opens, allowing tea/coffee/sugar powder to fall into the space between blades.
   - Step 3: Separates 1 teaspoon of powder.
   - Step 4: Lower blade opens, depositing 1 teaspoon of powder into the cup.
   - Repeat steps based on user beverage selection.
6. **Measuring System**: Servo-controlled separation blades ensure precise measurements.
7. **Filter System**: After hot water passes through the tea leaf filter, waste leaves are disposed of in a collection bin.
8. **Sensors**:
   - **IR Sensor**: Detects cup placement.
   - **Ultrasonic Sensor**: Measures water level and ensures accurate tea pouring.
   - **Temperature Sensor**: Regulates water heating.
   - **Weight Sensor**: Calculates container weights and monitors tea wastage.
   - **Real-Time Clock (RTC) Module**: Manages timing and process duration.
   - **GSM Module**: Sends tea count and refill alerts to the counter.
   - **Bluetooth Module**: Communicates cup placement status to the cup releaser.

## Installation
1. Create a clone of this repository.
2. Assemble and configure the necessary hardware (servos, sensors, etc.).
3. Programme the microcontroller with the Arduino code.
4. Enjoy your customized tea!!


Feel free to adapt and improve this project. Happy tea-making! 🍵

