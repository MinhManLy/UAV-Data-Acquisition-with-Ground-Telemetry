/* GROUND-BASED TELEMETRY PROGRAM CODE
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

#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <nRF905.h>

// =============================================================================================== //
// ===                              PINS & VARIABLES DECLARATION                               === //
// =============================================================================================== //

#define RXADDR 0xE7E7E7E7   // Address of this device (ground station)
#define TXADDR 0xE7E7E7E7   // Address of device to send to (drone sensors)

#define PACKET_NONE		0     // No data detected
#define PACKET_OK		1       // Data detected
#define PACKET_INVALID	2   // Bad/invalid data detected

#define PAYLOAD_SIZE	NRF905_MAX_PAYLOAD  // Max size of data packet (buffer) = 32

#define button1 A3          // Pushbutton to change display mode, reset altitude & re-calibrate pitch/roll (connect to Arduino A3 and GND)
int button1_current;        // Current pushbutton state to change display mode
int button1_previous = 0;   // Previous pushbutton state to change display mode
int lcd_state;              // State of LCD display of different modes (TPA reader or Orientation mode)

int count;                  // Count the duration of pushbutton being pushed to change display modes - Can use millis() function instead of counter
//long time_count;          // (For testing) The last time the output pin was toggled
//long debounce = 200;      // (For testing) The debounce time, increase if the output flickers

nRF905 transceiver = nRF905();            // Define transceiver used - nRF905
LiquidCrystal_I2C lcd(0x27,20,4);         // Define lcd display used - LCD2004_I2C

static volatile uint8_t packetStatus;     // Status of data packet
uint8_t replyBuffer[PAYLOAD_SIZE];        // Reply data buffer from this device (ground station) to receiver device (drone sensors) of same address\

// =============================================================================================== //
// ===                                    INTERRUPT SETUPS                                     === //
// =============================================================================================== //

// Don't modify these 2 functions. They just pass the DR/AM interrupt to the correct nRF905 instance.
void nRF905_int_dr(){transceiver.interrupt_dr();}   // Receive and transmit ready interrupt
void nRF905_int_am(){transceiver.interrupt_am();}   // Address matched interrupt

// =============================================================================================== //
// ===                                  FUNCTIONS DEFINITION                                   === //
// =============================================================================================== //

// Event function for RX complete
void nRF905_onRxComplete(nRF905* device)  {
	packetStatus = PACKET_OK;
	transceiver.standby();
}

// Event function for RX invalid
void nRF905_onRxInvalid(nRF905* device) {
	packetStatus = PACKET_INVALID;
	transceiver.standby();
}

// Make buffer for data
int32_t buffer[PAYLOAD_SIZE];     // Data buffer from other device (drone sensors) sent to this device (ground station)

// =============================================================================================== //
// ===                                     INITIAL SETUPS                                      === //
// =============================================================================================== //

void setup()  {
	Serial.begin(115200);           // Set up serial monitor for troubleshooting

  lcd.init();                     // Initialise LCD2004_I2C
  lcd.backlight();                // Turn on backlight for LCD2004_I2C (consume more current)
  //lcd.noBacklight;              // Turn off backlight
  lcd.setCursor(0,1);             // Display project team
  lcd.print("Designed by");
  lcd.setCursor(6,2);
  lcd.print("Octaltech Team");    // Madeup name of the project team (no trademark)
  delay(3000);                    // Delay is unecessary but is here to wait for drone sensor to finish initialisation
  
  pinMode(button1, INPUT_PULLUP); // Set up push button for changing mode
  
	//Serial.println(F("Server starting..."));    // For troubleshooting
  
  // This must be called first
	SPI.begin();                    // Set up SPI

	transceiver.begin(
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
	
	transceiver.events(                       // Register event functions
		nRF905_onRxComplete,
		nRF905_onRxInvalid,
		NULL,
		NULL
	);
	
	transceiver.setListenAddress(RXADDR);     // Set address of this device

  // Set channel and band for legal ISM use in Australia
  // NOTE: we can either use 433 or 915MHz as 433MHz output power is 
//10dBm (10mW) (legal for ISM use in Australia). Prefer to nRF905_config -> NRF905_PWR = NRF905_PWR_10 (10dBm = 10mW)
  transceiver.setBand(NRF905_BAND_433);
  transceiver.setChannel(10);

	transceiver.RX();         // Put into receive mode

  lcd_state = 0;            // Declare first display (TPA reader) that the lcd screen shows
  button1_previous = 0;     // Declare previous button state = 0 to toggle display when pushbutton is pressed

	//Serial.println(F("Server started"));    // For troubleshooting
  lcd.clear();
  lcd.setCursor(2,1);
  lcd.print("Waiting for data");
  lcd.setCursor(9,2);
  lcd.print("...");
  delay(1000);              // Delay is unecessary but is here to wait for data from drone sensor
}

// =============================================================================================== //
// ===                                    MAIN PROGRAM LOOP                                    === //
// =============================================================================================== //

// NEED A FUNCTION TO KNOW WHEN THE SENSOR STABILIZATION IS FINISHED
// The interaction between display and pushbutton is quite glitchy but works good enough for the project
  
void loop() {
  toggle_button();                                    // To toggle LCD display to change modes

  if ((button1_previous == 0 && button1_current == 1))  {         // TPA Reader display mode on
    lcd.clear();

// =============================  TEMP, PRESSURE & ALTITUDE DISPLAY  ============================= //
    while (lcd_state == 0)  {                                     // TPA Reader display mode on, loop for continous data acquisition from BMP280
      if (digitalRead(button1) == 0)  {                           // If the pushbutton is pressed while in the TPA Reader screen, the display will exit TPA Reader mode and change to Orientation mode by setting the state of lcd = 1
        lcd_state = 1;
      }
      
//                                    Transceiver in Receive mode                                  //
      while(packetStatus == PACKET_NONE);                         // Wait for data from Arduino onboard
      if(packetStatus != PACKET_OK) {                             // If good data packet has not been received by this device, status of packet is 'no packet' and the transceiver is put into Receiver mode for any incoming data packet
        packetStatus = PACKET_NONE;
        //Serial.println(F("Invalid packet!"));
        transceiver.RX();                                         // Put into receive mode
      }
      else {                                                      // If good data packet has been received, carry the following operations
        if (digitalRead(button1) == 0)  {                         // If the pushbutton is pressed while in the TPA Reader screen, the display will exit TPA Reader mode and change to Orientation mode by setting the state of lcd = 1
          lcd_state = 1;
        }
        packetStatus = PACKET_NONE;                               // Set packet status to none
        transceiver.read(buffer, sizeof(buffer));                 // Read payload data packet from on board Arduino transceiver
        TPA_Reader();                                             // Take and show received data

        // Reset ground pressure and altitude (ground altitude is reset to approx 0m) - NOTE: the altitude can be reset at any time even when the drone is in flight
        count = 0;                                                // Set counter = 0 to measure the duration of pushbutton being pressed without released
        while (digitalRead(button1) == 0) {                       // When the pushbutton is pressed while in TPA Reader dispay mode
          count++;                                                // Start counting
          if (count >= 5)  {                                      // If the pushbutton is being pressed and hold long enough (1 second)
            // SEND REPLY AND COMMAND TO DRONE SENSOR
            //Serial.println("SENT REPLY AND COMMAND TO DRONE SENSOR");
            lcd.clear();                                          // Temporarily display 'Reset altitude' on the lcd screen
            lcd.setCursor(1,1);
            lcd.print("Reset altitude ...");

//                                    Transceiver in Transmit mode                                 //
            // Reply back to onboard Arduino
            replyBuffer[0] = 1;                                   // Create buffer for reply with pattern '1 1 0 0' for resetting altitude
            replyBuffer[1] = 1;
            replyBuffer[2] = 0;
            replyBuffer[3] = 0;
            
            transceiver.write(TXADDR, replyBuffer, sizeof(replyBuffer));  // Write reply data and destination address to radio
            
            while(!transceiver.TX(NRF905_NEXTMODE_RX, true));     // Send the reply data, once the transmission has completed go into receive mode
            
            Serial.print(F("Reply data:"));                       // For troubleshooting purpose on serial monitor
            for(uint8_t i=0;i<4;i++)
            {
              Serial.print(F(" "));
              Serial.print(replyBuffer[i], DEC);
            }
            Serial.println();
            Serial.flush();
            
            //delay(100);
            
            lcd_state = 0;                                        // Set state of lcd = 0 to ramain in the TPA Reader display without changing to Orientation display mode
            delay(500);
          }
          else if (count < 5) {                                   // If the pushbutton is not pressed and hold long enough (<1 sec), set state of lcd = 1 and change display mode to Orientation
            lcd_state = 1;
            delay(200);                                           // Pushbutton duration to change display mode = count * delay time = 5 * 200 = 1 second (count >= 5)
          }
        }

//                                    Transceiver in Receive mode                                  //
        //memset(replyBuffer, 0, PAYLOAD_SIZE);                   // Reset reply buffer (not work yet)
        transceiver.RX();                                         // Put into receive mode (with no reply to sensor)
        if (digitalRead(button1) == 0)  {                         
          lcd_state = 1;
        }
    	}
    }
    delay(100);
  }

// ====================================  PITCH & ROLL DISPLAY  =================================== //
  else {                                                          // Orientation display mode on
    //delay(1000);
    lcd.clear();
    
//                                    Transceiver in Receive mode                                  //
    while (lcd_state == 1)  {                                     // Orientation display mode on, loop for continous data acquisition from MPU6050
      if (digitalRead(button1) == 0)  {
          lcd_state = 0;
      }
      while(packetStatus == PACKET_NONE);                         // Wait for data from onboard sensor
      if(packetStatus != PACKET_OK) {                             // If good data packet has not been received by this device, status of packet is 'no packet' and the transceiver is put into Receiver mode for any incoming data packet
        packetStatus = PACKET_NONE;
        //Serial.println(F("Invalid packet!"));
        transceiver.RX();                                         // Put into receive mode
      }
      else {                                                      // If good data packet has been received, carry the following operations
        packetStatus = PACKET_NONE;
        transceiver.read(buffer, sizeof(buffer));                 // Read payload
        Orientation_Reader();                                     // Take and show received data
        
        //transceiver.RX();                                       // Put into receive mode (with no reply to sensor)
        
        // Reset MPU6050 by resetting Digital Motion Processor 
        count = 0;                                                // Set counter = 0 to measure the duration of pushbutton being pressed without released
        while (digitalRead(button1) == 0) {                       // When the pushbutton is pressed while in Orientation dispay mode
          count++;                                                // Start counting
          if (count >= 5)  {                                      // If the pushbutton is being pressed and hold long enough (1 second)
            // SEND REPLY AND COMMAND TO DRONE SENSOR
            Serial.println("SENT REPLY AND COMMAND TO DRONE SENSOR");
            lcd.clear();
            lcd.setCursor(2,1);
            lcd.print("Reset MPU6050 ...");

//                                   Transceiver in Transmit mode                                  //
            //uint8_t replyBuffer[PAYLOAD_SIZE];                  // Copy data into new buffer for modifying // NOT WORK YET
            // Reply back to onboard Arduino
            replyBuffer[0] = 0;                                   // Create buffer for reply with pattern '0 0 1 1' for calibrating pitch and roll
            replyBuffer[1] = 0;
            replyBuffer[2] = 1;
            replyBuffer[3] = 1;

            transceiver.write(TXADDR, replyBuffer, sizeof(replyBuffer));  // Write reply data and destination address to radio
            
            while(!transceiver.TX(NRF905_NEXTMODE_RX, true));     // Send the reply data, once the transmission has completed go into receive mode
            
            Serial.print(F("Reply data:"));                       // For troubleshooting purpose on serial monitor
            for(uint8_t i=0;i<4;i++)
            {
              Serial.print(F(" "));
              Serial.print(replyBuffer[i], DEC);
            }
            Serial.println();
            Serial.flush();
            
            lcd_state = 0;                                        // Set state of lcd = 0 to ramain in the Orientation display without changing to TPA Reader display mdoe
            delay(100);
            // Reset reply buffer
            //memset(replyBuffer, 0, PAYLOAD_SIZE);
          }
          else if (count < 5) {                                   // If the pushbutton is not pressed and hold long enough (<1 sec), set state of lcd = 1 and change display mode to Orientation
            lcd_state = 0;
            delay(200);                                           // Pushbutton duration to change display mode = count * delay time = 5 * 200 = 1 second (count >= 5)
          }
        }

//                                    Transceiver in Receive mode                                  //
        //memset(replyBuffer, 0, PAYLOAD_SIZE);                   // Reset reply buffer (not work yet)
        //delay(100);                                             // To slow dowN transmission speed/rate
        transceiver.RX();                                         // Put into receive mode (with no reply to sensor)
        
        //if (digitalRead(button1) == 0)  {
          //lcd_state = 0;
        //}
      }
    }
  }
  if ((button1_previous != 0 || button1_current != 1)) {          // This if statement is to partly prevent the glitch when the display is changed from Orienation back to TPA Reader
      lcd.clear();
      lcd.setCursor(2,1);
      lcd.print("PRESS THE BUTTON");
      delay(100);
  }
}

// =============================================================================================== //
// ===                                  FUNCTIONS DEFINITION                                   === //
// =============================================================================================== //

void toggle_button()  {
  button1_previous = button1_current;           // To toggle LCD display to change modes
  button1_current = digitalRead(button1);       
  
  //Serial.print(button1_previous);             // For troubleshooting
  //Serial.print("  ");
  //Serial.println(button1_current);
}

void TPA_Reader() {
  Serial.print(F("Barometric sensor values: "));                  // Show received data in serial monitor
  Serial.print(buffer[1]<<8 | buffer[2]);
  Serial.print(F(" "));
  Serial.print(buffer[3]<<8 | buffer[4]);
  Serial.print(F(" "));
  Serial.println(buffer[5]<<8 | buffer[6]);
  int32_t temp = (buffer[1]<<8 | buffer[2]);                      // Put BMMP280 data into variables to read on the lcd screen
  int32_t pressure = (buffer[3]<<8 | buffer[4]);
  int32_t altitude = (buffer[5]<<8 | buffer[6]);
  lcd.setCursor(5,0); lcd.print("TPA Reader");                    // Print data in lcd screen
  lcd.setCursor(0,1); lcd.print("Temp(*C)  = "); lcd.print(temp/100.); lcd.print("   ");
  lcd.setCursor(0,2); lcd.print("Pres(hPa) = "); lcd.print(pressure/100.); lcd.print(" ");
  lcd.setCursor(0,3); lcd.print("Alti(m)   = "); lcd.print(altitude/100.); lcd.print(" ");
}

void Orientation_Reader() {   // Need more testings
    Serial.print(F("IMU values: "));                              // Show received data in serial monitor
    //Serial.print(buffer[8]<<0); //yaw - not work
    //Serial.print(F(" "));
    Serial.print(buffer[0]<<0); //pitch
    Serial.print(F(" "));
    Serial.print(buffer[7]<<0); //roll
    Serial.println(F(" "));
    int16_t pitch = (buffer[0]<<0);                               // Put MPU6050 data into variables to read on the lcd screen
    int16_t roll = (buffer[7]<<0);
    //int16_t yaw = (buffer[11]<<0);
    //Serial.println(yaw);
    //Serial.println(pitch);
    //Serial.println(roll);
    lcd.setCursor(5, 0);  lcd.print("Orientation");               // Print data in lcd screen
    //lcd.setCursor(0, 1);  lcd.print("Yaw   = "); lcd.print(yaw/100.); lcd.print(" ");
    lcd.setCursor(0, 1);  lcd.print("Pitch = "); lcd.print(pitch/100.); lcd.print(" ");
    lcd.setCursor(0, 2);  lcd.print("Roll  = "); lcd.print(roll/100.); lcd.print(" ");
    lcd.setCursor(0, 3);  lcd.print("Only reset if needed");
}

// =============================================================================================== //
// ===                                          END                                            === //
// =============================================================================================== //

/* // Posible solution to glitch screen after Orientation display
void menuSwitch() {
  if (!(digitalRead(sw))){
    modeMenu = modeMenu + 1;
    if (modeMenu > 3) {
      modeMenu = 1;
    }
    Serial.print("modeMenu = ");
    Serial.println(modeMenu);
    lcd.clear();
    arrowMenu();
    displayMenu();
    delay(200);
  }
}
*/
