#include <SoftwareSerial.h>
SoftwareSerial Esp(4,5);

int count=0;
int resetPin = 12;
int outTemps1=A1;
int TempRoom;
float mvRoom;
float celRoom;
int outTemps2=A2;
/*float latitude=100.000;
float longitude=2000.000;
*/
#include<dht.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

dht DHT;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// if you require to change the pin number, Edit the pin with your arduino pin.
#define DHT11_PIN A0
#define DHT11_PIN2 A2


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Esp.begin(9600);
  count=0;
  while (!Esp) {};
  
// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

Serial.begin(9600);

Serial.println("Welcome to the Testing module of C-BAND");

while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

// load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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

}

void loop() 
{ // READ DATA


   // if programming failed, don't try to do anything
   // if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            delay(100);
            Serial.print("Moving for/back ");
            Serial.print(ypr[0] * 180/M_PI);
            Esp.print(ypr[0] * 180/M_PI);
            delay(100);
            Serial.print("\t");
            Esp.print(",");
            Serial.print("Moving up/down");
            Serial.print(ypr[1] * 180/M_PI);
            Esp.print(ypr[1] * 180/M_PI);
            delay(100);
            Serial.print("\t");
            Esp.print(",");
            Serial.print("Tilting");
            Serial.print(ypr[2] * 180/M_PI);
            Esp.print(ypr[2] * 180/M_PI);
            delay(50);
        #endif
         // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

int chk = DHT.read11(DHT11_PIN);

Serial.print("\t");
Esp.print(",");
Serial.print(" Humidity " );

Serial.print(DHT.humidity, 1);
Esp.print(DHT.humidity, 1);
Serial.print("\t");
Esp.print(",");
delay(50);
Serial.print(" Temparature ");

Serial.print(DHT.temperature, 1);
Esp.print(DHT.temperature, 1);
Serial.print("\t");
Esp.print(",");
delay(50);
tempSensor(outTemps1);
delay(50);
int chk2 = DHT.read11(DHT11_PIN2);

Serial.print(" Humidity Room " );

Serial.print(DHT.humidity, 1);
Esp.print(DHT.humidity, 1);
Serial.print("\t");
Esp.print(",");
delay(50);
Serial.print(" Temparature Room ");

Serial.print(DHT.temperature, 1);
Esp.println(DHT.temperature, 1);
delay(50);
Serial.print("\n");

    }
}



void tempSensor(int outTemps)
{
  int x=0;
  float cel=0;
  while (x<10)
  {
 TempRoom = analogRead(outTemps);
 mvRoom = ( TempRoom/1024.0)*5000; 
 celRoom = mvRoom/10;
 cel=celRoom+cel;
 x++;
  }
  cel=cel/10;
  Serial.print("TEMPRATURE Body  ");
  Serial.print(cel);
  Esp.print(cel);
  Serial.print("*C");
  Serial.print("\t");
  Esp.print(",");
  delay(500);
}





