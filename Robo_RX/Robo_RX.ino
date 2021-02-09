// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration

#include <Servo.h>

#include <SPI.h>
#include <RH_RF69.h>



/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           13
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
#define RFM69_CS      2    // "E"
#define RFM69_IRQ     15   // "B"
#define RFM69_RST     16   // "D"
#define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
#define RFM69_RST     13   // same as LED
#define RFM69_CS      33   // "B"
#define RFM69_INT     27   // "A"
#define LED           13
#endif

/* Teensy 3.x w/wing
  #define RFM69_RST     9   // "A"
  #define RFM69_CS      10   // "B"
  #define RFM69_IRQ     4    // "C"
  #define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/

/* WICED Feather w/wing
  #define RFM69_RST     PA4     // "A"
  #define RFM69_CS      PB4     // "B"
  #define RFM69_IRQ     PA15    // "C"
  #define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission



typedef struct
{
  uint16_t   dataitemx;
  uint16_t   dataitemy;
  uint16_t   dataitemz;

} ControlDataStruct;


// HEAD 

Servo tiltServo;  // create servo object to control a servo
Servo panServo;  // create servo object to control a servo

int pos = 90;    // variable to store the servo position

int x = 0;

int y = 0;

int z = 0;

int driving = 1; // Set to 1 so will initate stop at turn on.


////DRIVE

#include <SoftwareSerial.h>
#include <Sabertooth.h>

SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.

// Simplifierd serial limits for each motor
#define SBT_MOTOR1_FULL_FORWARD 10
#define SBT_MOTOR1_FULL_REVERSE -10

#define SBT_MOTOR2_FULL_FORWARD 10
#define SBT_MOTOR2_FULL_REVERSE -10

void setup()
{
  Serial.begin(9600);
  SWSerial.begin(9600);

  ST.autobaud();
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);

  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");


  tiltServo.attach(10);  // attaches the servo on pin 9 to the servo object
  panServo.attach(9);  // attaches the servo on pin 9 to the servo object

}


void loop() {

  // Engage Head position 
  tiltServo.write(pos);

  if (rf69.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);


    ControlDataStruct data;
    uint8_t datalen = sizeof(data);
    
    if (   rf69.recv((uint8_t*)&data, &datalen)
           && datalen == sizeof(data))
    {
      // Have the data, so do something with it
      uint16_t x = data.dataitemx;
      uint16_t y = data.dataitemy;
      uint16_t z = data.dataitemz;

      if (!datalen) return;
      buf[len] = 0;
      // Some debug
      //      Serial.print("Received [");
      //      Serial.print(datalen);
      //      Serial.print("]: ");
      //  Serial.println(x);
      //      Serial.println(y);
      //      Serial.print("RSSI: ");
      //      Serial.println(rf69.lastRssi(), DEC);
      //
      if (z == 1) {
        driveCommand(x, y);
      }
      else {

        if (x != 5) {
          panHead(x);
        }
        else {
          panServo.write(90);
        }

        if (y != -1) {
          tiltHead(y);
        }
        else {
          tiltServo.write(pos);
        }
        killMotors();

      }



    }
    else {
      Serial.println("Receive failed");
    }
  }
}

// Main drive function
void driveCommand(int x, int y) {
  Serial.println("Steer ing " + x);
  //127 right
  //-127 LEFT
  if (x < -10 || x > 20 ) {
    driving = 1;
    if (x > 10) {
      turnRight();
    }
    if (x < -1) {
      turnLeft();
    }
  }
  else if (y < -10 || y > 10 ) {
    driving = 1;
    //127 full forWARD
    //-128 FULL BACK
    if (y > 10) {
      driveForward();
    }
    if (y < -10) {
      driveReverse();
    }
  }
  else {
    killMotors();
  }
}

void driveForward() { //motors fast forward
  ST.motor(1, SBT_MOTOR1_FULL_FORWARD);
  ST.motor(2, SBT_MOTOR2_FULL_FORWARD);
  Serial.println("motors fast forward");
}


void driveReverse() { //motors fast reverse
  ST.motor(1, SBT_MOTOR1_FULL_REVERSE);
  ST.motor(2, SBT_MOTOR2_FULL_REVERSE);
  Serial.println("motors fast reverse");
}

void turnLeft() { //motor 1 full reverse and motor 2 full forward to turn left
  ST.motor(1, SBT_MOTOR1_FULL_REVERSE);
  ST.motor(2, SBT_MOTOR2_FULL_FORWARD);
  Serial.println("motor 1 full reverse and motor 2 full forward to turn left");
}

void turnRight() { //motor 1 full forward and motor 2 full reverse to turn right
  ST.motor(1, SBT_MOTOR1_FULL_FORWARD);
  ST.motor(2, SBT_MOTOR2_FULL_REVERSE);
  Serial.println("motor 1 full forward and motor 2 full reverse to turn right");
}

// STOP
void killMotors() {
  if (driving == 1) {
    ST.motor(1, 0);   //kill motors for 0.5 second
    ST.motor(2, 0);   //kill motors for 0.5 second
    driving = 0;
  }
  Serial.println("kill motors ");

}


void tiltHead(int y)
{
  if (y < 179 || y > 0 ) {
    if ( y < -2) {
      pos = pos - 2;
      tiltServo.write(pos);
      Serial.println("tilt -- write ");
      Serial.println("\r\n");
      //tiltServo.write(70);              // tell servo to go to position in variable 'pos'
      delay(15);
    }
    if (y > 1 ) {
      pos = pos + 2;
      tiltServo.write(pos);
      Serial.println("tilt ++ write ");
      Serial.println("\r\n");
      // tell servo to go to position in variable 'pos'
      delay(15);
    }
  }
  Serial.println("\r\n ----" + pos);
  tiltServo.write(pos);
}


void panHead(int x)
{
  if ( x < 4) {

    Serial.println("y write less than5 ");
    Serial.println("\r\n");
    panServo.write(110);              // tell servo to go to position in variable 'pos'
    delay(15);
  }
  else if (x > 6 ) {

    Serial.println("y write ");
    Serial.println("\r\n");
    panServo.write(70);              // tell servo to go to position in variable 'pos'
    delay(15);
  }
  else {
    Serial.println("no write 5 ");
    // panServo.write(90);              // tell servo to go to position in variable 'pos'
  }
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}
