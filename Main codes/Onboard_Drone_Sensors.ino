/* ONBOARD DRONE SENSORS PROGRAM CODE
 * Project: UAV Real-time Data Acquisition with Wireless Telemetry
 * Author: Minh Man Ly
 * Co-authors: Nicolas Rayner, Khaileb Freeman & Gus Pfitzner
 * Reference: Zak Kemble & Jeff Rowberg
 * Sponsor: Flight One Academy - Archerfield, QLD, Australia
 * Circuit schematic: Available on the website below (Github)
 * Web link: https://github.com/MinhManLy/UAV-Data-Acquisition-with-Ground-Telemetry
 * This program code is not the low power consumption version. Yet, the code is still 
in accordance with legal RF band for ISM purpose in Australia - 433MHz & less than 25mW power output 
 */

// =============================================================================================== //
// ===                                        LIBRARIES                                        === //
// =============================================================================================== //

#include <nRF905.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h"              // Not necessary if using MotionApps include file

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE  // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
    #include "Wire.h"
#endif

// =============================================================================================== //
// ===                              PINS & VARIABLES DECLARATION                               === //
// =============================================================================================== //

#define RXADDR 0xE7E7E7E7           // Address of this device (drone sensors)
#define TXADDR 0xE7E7E7E7           // Address of device to send to (ground station)
#define NODE_ID        78           // No actual use

#define TIMEOUT 200                 // Define refresh rate/transmission rate for drone sensor receive (recommmend: 100ms -> 500ms ping timeout)

#define PACKET_NONE		0             // No data detected
#define PACKET_OK		1               // Data detected
#define PACKET_INVALID	2           // Bad/invalid data detected

#define PAYLOAD_SIZE	NRF905_MAX_PAYLOAD  // Max size of data packet (buffer) = 32

#define initialiseLED A0            // LED to signal the completion of setup (not use)

float pressure, temp, altitude;
float ground_pressure, total_pressure;

nRF905 transceiver = nRF905();      // Define transceiver used - nRF905
Adafruit_BMP280 bmp; // I2C         // Define barometric sensor used - BMP280

MPU6050 mpu;                        // Class default I2C address is 0x68
//MPU6050 mpu(0x69);                // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL

//#define INTERRUPT_PIN 2           // Use pin 2 on Arduino Uno & most boards //NOT USED FOR MPU6050 as nRF905 is using it

float pitch, roll, yaw;             // Declare int/float orientation vars for 1m/1.00m resolution

static volatile uint8_t packetStatus;

// ===                                MPU CONTROL/STATUS VARS                                  === //
bool dmpReady = false;              // Set true if DMP init was successful
uint8_t mpuIntStatus;               // Holds actual interrupt status byte from MPU
uint8_t devStatus;                  // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                 // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];             // FIFO storage buffer

// ===                                ORIENTATION/MOTION VARS                                  === //
Quaternion q;                       // [w, x, y, z]         quaternion container
VectorInt16 aa;                     // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;                 // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;                // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;                // [x, y, z]            gravity vector
float euler[3];                     // [psi, theta, phi]    Euler angle container
float ypr[3];                       // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// =============================================================================================== //
// ===                                    INTERRUPT SETUPS                                     === //
// =============================================================================================== //

// NOT USED IN THIS PROGRAM
//volatile bool mpuInterrupt = false;     // Indicates whether MPU interrupt pin has gone high
//void dmpDataReady() {
    //mpuInterrupt = true;
//}

// Don't modify these 2 functions. They just pass the DR/AM interrupt to the correct nRF905 instance.
void nRF905_int_dr(){transceiver.interrupt_dr();}
void nRF905_int_am(){transceiver.interrupt_am();}

// =============================================================================================== //
// ===                                  FUNCTIONS DEFINITION                                   === //
// =============================================================================================== //

// Event function for RX complete
void nRF905_onRxComplete(nRF905* device)
{
	packetStatus = PACKET_OK;
	transceiver.standby();
}

// Event function for RX invalid
void nRF905_onRxInvalid(nRF905* device)
{
	packetStatus = PACKET_INVALID;
	transceiver.standby();
}

// =============================================================================================== //
// ===                                    INITIAL SETUPS                                       === //
// =============================================================================================== //

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE            // Join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000);                                    // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  pinMode(initialiseLED, OUTPUT);
  digitalWrite(initialiseLED, LOW);
  
	Serial.begin(115200);
  while (!Serial);                                            // Wait for Leonardo enumeration, others continue immediately
  
  //Serial.println(F("Initializing I2C devices..."));         // For troubleshooting
  mpu.initialize();                                           // Initialize device
  //pinMode(INTERRUPT_PIN, INPUT); //NOT USED FOR MPU6050 as nRF905 is using it
  Serial.println(F("Testing device connections..."));         // Verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read());                // Empty buffer
  //while (!Serial.available());                              // Wait for data
  //delay(1000);
  while (Serial.available() && Serial.read());                // Empty buffer again
  
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();                            // Load and configure the DMP
  
  // Supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);                                  // 1688 factory default for Jeff Rowberg's test chip

  if (devStatus == 0) {                                       // Make sure it worked (returns 0 if so)
    mpu.CalibrateAccel(6);                                    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);                                  // Turn on the DMP, now that it's ready

    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING); //INTERRUPT NOT USED FOR MPU6050 as nRF905 is using it
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }
  
  /* For debugging Barometric sensor
	if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  */
	
	// Standby off
	//pinMode(7, OUTPUT); 
	//digitalWrite(7, HIGH);
	
	SPI.begin();                                                  // This must be called first
  
	transceiver.begin(                                            // All wires, maximum functionality for nRF905
		SPI,      // SPI bus to use (SPI, SPI1, SPI2 etc)
		10000000, // SPI Clock speed (10MHz)
		10, // SPI SS
		7,  // CE (standby)
		9,  // TRX (RX/TX mode)
		8,  // PWR (power down)
		4,  // CD (collision avoid)
		3,  // DR (data ready)
		2,  // AM (address match)
		nRF905_int_dr,  // Interrupt function for DR
		nRF905_int_am   // Interrupt function for AM
	);

/*
	// Minimal wires (polling)
	// Up to 5 wires can be disconnected, however this will reduce functionality and will put the library into polling mode instead of interrupt mode
	// In polling mode the .poll() method must be called as often as possible. If .poll() is not called often enough then events may be missed. (Search for .poll() in the loop() function below)
	transceiver.begin(
		SPI,
		10000000,
		6,
		NRF905_PIN_UNUSED, // CE (standby) pin must be connected to VCC (3.3V) - Will always be in RX or TX mode
		9, // TRX (RX/TX mode)
		NRF905_PIN_UNUSED, // PWR pin must be connected to VCC (3.3V) - Will always be powered up
		NRF905_PIN_UNUSED, // Without the CD pin collision avoidance will be disabled
		NRF905_PIN_UNUSED, // Without the DR pin the library will run in polling mode and poll the status register over SPI. This also means the nRF905 can not wake the MCU up from sleep mode
		NRF905_PIN_UNUSED, // Without the AM pin the library the library must poll the status register over SPI.
		NULL, // No interrupt function
		NULL // No interrupt function
	);
*/

/*
	// Minimal wires (interrupt)
	// Up to 4 wires can be disconnected, however this will reduce functionality.
	// onAddrMatch and onRxInvalid events will not work with this configuration as they rely on the AM interrupt.
	transceiver.begin(
		SPI,
		10000000,
		6,
		NRF905_PIN_UNUSED, // CE (standby) pin must be connected to VCC (3.3V) - Will always be in RX or TX mode
		9, // TRX (RX/TX mode)
		NRF905_PIN_UNUSED, // PWR pin must be connected to VCC (3.3V) - Will always be powered up
		NRF905_PIN_UNUSED, // Without the CD pin collision avoidance will be disabled
		3, // DR (data ready)
		NRF905_PIN_UNUSED, // Without the AM pin the library must poll the status register over SPI.
		nRF905_int_dr,
		NULL // No interrupt function
	);
*/

	transceiver.events(                                   // Register event functions
		nRF905_onRxComplete,
		nRF905_onRxInvalid,
		NULL,
		NULL
	);

	transceiver.setListenAddress(TXADDR);                 // Set address of this device
 
  // Set channel and band for legal ISM use in Australia
  // NOTE: we can either use 433 or 915MHz as 433MHz output power is 
//10dBm (10mW). Prefer to nRF905_config -> NRF905_PWR = NRF905_PWR_10 (10dBm = 10mW)
  transceiver.setBand(NRF905_BAND_433);
  transceiver.setChannel(10);

  bmp.begin();                                          // Set up the Barometric sensor
  
  // Default settings from datasheet
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,         // Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,         // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,        // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,          // Filtering.
                  Adafruit_BMP280::STANDBY_MS_500);     // Standby time.
                  
  // Getting average temperature and pressure for ground station
  // Altitude at ground station is approximately 0 meter
  total_pressure = 0;
  for (int i = 1; i <= 50; i++) {   //50 measurements
    pressure = bmp.readPressure();
    total_pressure += pressure;
    //Serial.print("    ");
    //Serial.println(total_pressure);
    delay(50);                                          // Total delay time = 50 * 50 = 2.5 secs
  }
  ground_pressure = total_pressure/50.;
  
	Serial.println(F("Onboard drone sensor - setup completed"));
  //digitalWrite(initialiseLED, HIGH);
}

// =============================================================================================== //
// ===                                    MAIN PROGRAM LOOP                                    === //
// =============================================================================================== //

void loop()
{
	static uint8_t counter;                                   // Not use
  
  temp = bmp.readTemperature();                             // Store float value of temperature (*C) in 'temp'
  pressure = bmp.readPressure();                            // Store float value of pressure (Pa) in 'pressure'
  altitude = bmp.readAltitude(ground_pressure/100.);        // Store float value of altitude (m) in 'altitude'. Altitude is calculated by divided the average ground pressure taken at the end of set ups by 100 (100 is to convert Pa -> hPa)
  int temp_int = temp*100;                                  // Move decimal point 2 places to the right (e.g temp = 25.23*C -> temp_int = 2523*C) => This is the number later be sent from this device to ground statino telemetry
  long int pressure_int = pressure;                         // Float-type pressure (Pa) is converted to long int-type pressure (Pa) (e.g pressure = 101748.21Pa - > pressure_int = 101748Pa)
  int altitude_int = altitude*100;                          // Move decimal point 2 places to the right (e.g altitude = 3.25m -> altitude_int = 325m) => This is the number later be sent from this device to ground statino telemetry

  Orientation_Reader();                                     // Read data from MPU6050 IMU

  int yaw_int = yaw*100;                                    // Move decimal point 2 places to the right (e.g yaw = 12.25 -> yaw_int = 1225) => This is the number later be sent from this device to ground statino telemetry
  int pitch_int = pitch*100;                                // Move decimal point 2 places to the right (e.g pitch = 12.25 -> pitch_int = 1225) => This is the number later be sent from this device to ground statino telemetry
  int roll_int = roll*100;                                  // Move decimal point 2 places to the right (e.g roll = 12.25 -> roll_int = 1225) => This is the number later be sent from this device to ground statino telemetry
  
	int32_t buffer[PAYLOAD_SIZE];                             // Make data 
	//memset(buffer, counter, PAYLOAD_SIZE);
	//counter++;
  packetStatus = PACKET_NONE;

  // BMP280 data
  buffer[1] = temp_int>>8;
  buffer[2] = temp_int;
  buffer[3] = pressure_int>>8;
  buffer[4] = pressure_int;
  buffer[5] = altitude_int>>8;
  buffer[6] = altitude_int;
  // MPU6050 data
  buffer[0] = pitch_int>>0;
  buffer[7] = roll_int>>0;

  // Need more testings
  //buffer[8] = yaw_int>>0;
  //buffer[9] = pitch_int>>0;
  //buffer[10] = pitch_int;
  //buffer[11] = roll_int>>0;
  //buffer[12] = roll_int;

  Serial.print(F("Barometric sensor values: "));            // Show data
  Serial.print(temp_int);
  Serial.print(F(" "));
  Serial.print(pressure_int);
  Serial.print(F(" "));
  Serial.println(altitude_int);
  Serial.print(F("IMU values: "));
  Serial.print("yaw not work");
  Serial.print(F(" "));
  Serial.print(buffer[0]);
  Serial.print(F(" "));
  Serial.println(buffer[7]);
  //packetStatus = PACKET_NONE;
  
//                                   Transceiver in Transmit mode                                  //
	Serial.println(F("Sending data... "));
	// Write reply data and destination address to radio IC
	transceiver.write(TXADDR, buffer, sizeof(buffer));
  
  //packetStatus = PACKET_NONE;
  uint32_t startTime = millis();

//                                    Transceiver in Receive mode                                  //
	while(!transceiver.TX(NRF905_NEXTMODE_RX, true));         // Send the data (send fails if other transmissions are going on, keep trying until success) and enter RX mode on completion
	
	Serial.println(F("Waiting for reply..."));                // Receive reply (replyBuffer) from ground station

  uint8_t success;

  uint32_t sendStartTime = millis();                        // Wait for reply with timeout
  while(1)  {
    //transceiver.poll();                                   // Uncomment this line if the library is running in polling mode

    success = packetStatus;
    if(success != PACKET_NONE)
      break;
    else if(millis() - sendStartTime > TIMEOUT)
      break;
  }

  if(success == PACKET_NONE)
  {
    Serial.println(F("Receiving ground data timed out > 200ms"));
    //timeouts++;
  }
  else if(success == PACKET_INVALID)
  {
    Serial.println(F("Invalid packet!"));
    //invalids++;
  }
  else
  {
    uint16_t totalTime = millis() - startTime;
    //replies++;
  
    // If success send ping time over serial
    Serial.print(F("Ping time: "));
    Serial.print(totalTime);
    Serial.println(F("ms"));
    
		// Get the reply data // NOT WORK YET
		uint8_t replyBuffer[PAYLOAD_SIZE];
		transceiver.read(replyBuffer, sizeof(replyBuffer));
    
		// Show received data
    Serial.print(replyBuffer[0], DEC);
    Serial.print(F(" "));
    Serial.print(replyBuffer[1], DEC);
    Serial.print(F(" "));
    Serial.print(replyBuffer[2], DEC);
    Serial.println();
    Serial.print(replyBuffer[3], DEC);
    Serial.println();

    // Reset ground altitude (1 1 0 0)
    if (replyBuffer[0] == 1 && replyBuffer[1] == 1)  {
      Serial.println("RESET GROUND ALTITUDE");
      Serial.print("GROUND PRESSURE RESET TO ");
      Reset_ground_pressure();
      Serial.println(ground_pressure);
    }
    // Reset pitch and roll (0 0 1 1)
    if (replyBuffer[2] == 1 && replyBuffer[3] == 1) {
      Serial.println("RESET MPU6050");
      Reset_MPU_DMP();
      Serial.print("Inertial Measurement Unit has been reset");
    }
  }
  Serial.flush();
    
  // Delay is for debugging
	//delay(500);
}

// =============================================================================================== //
// ===                                  FUNCTIONS DEFINITION                                   === //
// =============================================================================================== //

void Orientation_Reader() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;                                  // Read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {          // Get the Latest packet 
            mpu.dmpGetQuaternion(&q, fifoBuffer);           // Display Euler angles in degrees
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("Yaw = ");
            yaw = ypr[0] * 180/M_PI;
            Serial.print(yaw);
            Serial.print(" | Pitch = ");
            pitch = ypr[1] * 180/M_PI;
            Serial.print(pitch);
            Serial.print(" | Roll = ");
            roll = ypr[2] * 180/M_PI;
            Serial.println(roll);
    }
}

void Reset_ground_pressure() {
  // Getting average temperature and pressure for ground station
  // Altitude at ground station is approximately 0 meter
  float total_pressure = 0;
  float temporary_pressure;
  for (int j = 1; j <= 50; j++) {             //50 measurements
    temporary_pressure = bmp.readPressure();
    total_pressure += temporary_pressure;
    delay(50);
  }
  ground_pressure = total_pressure/50.;
}

void Reset_MPU_DMP()  {
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();                          // Load and configure the DMP
    
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);                                // 1688 factory default Jeff Rowberg's test chip

  if (devStatus == 0) {                                     // Make sure it worked (returns 0 if so)
    mpu.CalibrateAccel(6);                                  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);                                // Turn on the DMP, now that it's ready

    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING); //NOT USED FOR MPU6050 as nRF905 is using it
    mpuIntStatus = mpu.getIntStatus();

    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;                                        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
    
    packetSize = mpu.dmpGetFIFOPacketSize();                // get expected DMP packet size for later comparison
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

// =============================================================================================== //
// ===                                          END                                            === //
// =============================================================================================== //
