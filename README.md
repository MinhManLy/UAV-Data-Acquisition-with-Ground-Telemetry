# UAV Data Acquisition with Ground Telemetry
Arduino-based project using sensors and transceivers

Project team: Nicolas Rayner, Khaileb Freeman, Gus Pfitzner & Minh Man Ly

# Project description:
The project is to create sensor device for quadcopter drones (e.g Mavic Pro DJI drone) that acts just like some of the flight instruments used in crewed aircraft. This device collects data on temperature, pressure, altitude, and drone orientation and then transmits these real-time data to ground-based telemetry. Another part in the product of the project is the ground-based telemetry. Commands and communication can be made wirelessly between drones and ground station via transceiver modules of same RF band. Examples of the commands from ground station to drone sensors are resetting altitude to 0m and calibrating drone orientation before drone takes off.

Onboard sensors and ground-based telemetry are made on PCB circuit boards and enclosed in 3D printed housings. Onboard sensors and ground-based telemetry are designed to have successful communication from at least 1000m in distance and 100m in height. The 3D housings of the completed PCB circuit board for onboard sensors must fit to the quadcopter drone and the installation must be ensured not to violate thermodynamic operation of the drone and not to affect the IMU.
The project product – drone sensors and telemetry – will be tested and demonstrated in accordance with Drone Rules established by the Australian Civil Aviation Safety Authority. The RF band of the transceivers must meet the legal RF band use for ISM purposes established by Australian Government.
The project is sponsored by Flight One Academy - Archerfield, QLD, Australia.
 
# Components:
2 devices will be made in this project: the drone sensors and the telemetry on ground station. Components used in this project include:
+ Microcontroller: Arduino Nano
+ Sensors: IMU (MPU-6050) & Barometric sensor (BMP280)
+ Transmitter & Receiver: 433/868/915MHz Transceivers (nRF905)
+ Telemetry: LCD2004 display with I2C

# Libraries:
Below is the arduino libraries used in the project:

For ground-based telemetry:
+ <Wire.h>
+ <SPI.h>
+ <LiquidCrystal_I2C.h>
+ <nRF905.h> (from Zak Kemble)

For onboard drone sensor:
+ <nRF905.h> (from Zak Kemble)
+ <SPI.h>
+ <Adafruit_BMP280.h> 
+ <Wire.h>
+ "I2Cdev.h" (from Jeff Rowberg - MPU6050)
+ "MPU6050_6Axis_MotionApps20.h" (from Jeff Rowberg - MPU6050)
