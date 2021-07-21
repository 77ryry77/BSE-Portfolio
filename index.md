# Gesture Controlled Car
The car is controlled by the way the person moves their arm.  If the person moves their arm forward or backwards it will make the car move.  If the person turns their arm it will make the car turn in the same direction.

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Ryan Dalal | Edgemont Highschool | Computer Science | Incoming Sophmore

![Car](https://github.com/77ryry77/BSE_Portfolio/blob/gh-pages/IMG_0779.jpeg)

![Gyro](https://github.com/77ryry77/BSE_Portfolio/blob/gh-pages/gyro.png)

## Parts
- ESP32 Devkit V1 (2x)
- GY521 Gyroscope Acceleromoter
- L928n motor driver
- 12v DC motor (4x)
- 4x AA battery pack (x2)
- Arduino Uno (not neccessary if another 3.3v power source is available)
- Portable Charger (not neccessary if another power source for gyro is available)

# Final Milestone
  
The car is completed now.  It can move in all four directions and now it also has a speed control.  Depending on how far the person turns their wrist or twists forward and backwards, the car will move at different speeds.  This and the faster response time makes it much easier to control and manever around tight spaces.  The car and the remote also now both have indicator lights.  The light on the remote turns on when the gyroscope is ready and the car light turns on when the car is receiving information from the gyroscope.  The remote is also now battery powered and more portable.  This is the final stage of the car. 

![Contruction](https://t3.ftcdn.net/jpg/03/44/17/18/360_F_344171869_h3nxznW93zBoOLuMeIJ3Q3xzanFSN8vu.jpg)

```c++
//================
//Code for car esp
//================
#include "WiFi.h"
#include <Wire.h>
#include <esp_now.h>

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x9C, 0x9C, 0x1F, 0xE3, 0x73, 0xF8};

// Define variables to store incoming readings
float incomingYaw;
float incomingPitch;
float incomingRoll;

//first number in comment is the esp pinout --- second is arduino
//assigns the pins to control the left motors
int motor1pin1 = 26;//26 2
int motor1pin2 = 25;//25 3 

//assigns the pins to control the right motors
int motor2pin1 = 33;//33 4
int motor2pin2 = 32;//32 5

//pins for controlling speed of left and right motors
int motor1speed = 13;
int motor2speed = 14;

int dir = 0;//prevents switches in direction when moving the arm unit

const int freq = 30000;
const int pwmChannel = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 200;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float y;
    float p;
    float r;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message Readings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.println(len);
  incomingYaw = incomingReadings.y;
  incomingPitch = incomingReadings.p;
  incomingRoll = incomingReadings.r;
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

void setup() {
  // put your setup code here, to run once:

  //sets all the pins to control the motor to be output pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor1speed, OUTPUT);
  pinMode(motor2speed, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(motor1speed, pwmChannel);
  // configure LED PWM functionalitites
  
  ledcSetup(pwmChannel2, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(motor2speed, pwmChannel2);

  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // put your main code here, to run repeatedly:

  //this loop will decide which way to turn eventually

  //for now just moves car depending on what method is called
  if(incomingReadings.p > 40 && (dir == 0 || dir == 1))
  {
    dir = 1;
    //gets a number between 0 and 255 that is = the percentage of a full movement
    setSpeed((incomingReadings.p-40)/140*80+175);
    backward();
  }
  else if(incomingReadings.p < -40 && (dir == 0 || dir == 2))
  {
    dir = 2;
    setSpeed((-incomingReadings.p-40)/140*80+175);
    forward();
  }
  else if(incomingReadings.r > 40 && (dir == 0 || dir == 3))
  {
    dir = 3;
    setSpeed((incomingReadings.r-40)/140*80+175);
    left();
  }
  else if(incomingReadings.r < -40 && (dir == 0 || dir == 4))
  {
    dir = 4;
    setSpeed((-incomingReadings.r-40)/140*80+175);
    right();
  }
  else
  {
    dir = 0;
    off();
  }
}
//sets the speed of the motors
void setSpeed(int s){
  ledcWrite(pwmChannel, s);
  ledcWrite(pwmChannel2, s); 
}

//forward backward left right move set the direction of the wheels to control movmenet
void forward(){
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}
void backward(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
void left(){
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
void right(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}
//turns all the motors off
void off(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}
//=====================
//code for arm unit esp
//=====================
#include "I2Cdev.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;

#define INTERRUPT_PIN 33  // use pin 2 on Arduino Uno & most boards -- 33 on esp

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//esp data
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xAA, 0xE8, 0x50};

// Define variables to store BME280 readings to be sent
float yaw;
float pitch;
float roll;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float y;
    float p;
    float r;
} struct_message;

// Create a struct_message called readings to hold sensor readings
struct_message Readings;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

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
        pinMode(2, OUTPUT);
        digitalWrite(2, HIGH);
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

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
  
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
  
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
}

void loop() {
    getReadings();
   
    // Set values to send
    Readings.y = yaw;
    Readings.p = pitch;
    Readings.r = roll;
  
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));
     
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
}
void getReadings(){
  // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[0] * 180/M_PI;
            pitch = ypr[1] * 180/M_PI;
            roll = ypr[2] * 180/M_PI;
    }
}
```

# Second Milestone
  
The second milestone was getting basic gestures to make the car move.  Right now the first esp in someones hand sends data from the gyroscope, to the esp on the car which then controls the wheels of the car.  The four gesutres right now are moving forward and backward, and turning you wrist left and right.  The esp on the car is currently being powered by an unused arduino, and the esp in the persons hand is powered by a usb cable plugged into the wall.  The car and gyroscope are in a basic functioning stage right now.

[![Milestone2](https://res.cloudinary.com/marcomontalbano/image/upload/v1626873924/video_to_markdown/images/youtube--l1Im7CuR5S8-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=l1Im7CuR5S8 "Milestone2")

>To connect esps I followed a tutorial and used code from [here](https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/)

>To calculate the yaw pitch and roll values I used code from [here](https://www.instructables.com/Arduino-MPU6050-GY521-6-Axis-Accelerometer-Gyro-3D/)

```c++
//=====================================
//code for the esp connected to the car
//esp connection code based on tutorial at https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/ 
//=====================================
#include "WiFi.h"
#include <Wire.h>
#include <esp_now.h>

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x9C, 0x9C, 0x1F, 0xE3, 0x73, 0xF8};//replace with mac adress of other esp

// Define variables to store incoming readings
float incomingYaw;
float incomingPitch;
float incomingRoll;

//first number in comment is the esp pinout --- second is arduino
//assigns the pins to control the left motors
int motor1pin1 = 26;//26 2
int motor1pin2 = 25;//25 3 

//assigns the pins to control the right motors
int motor2pin1 = 33;//33 4
int motor2pin2 = 32;//32 5

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float y;
    float p;
    float r;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message Readings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.println(len);
  incomingYaw = incomingReadings.y;
  incomingPitch = incomingReadings.p;
  incomingRoll = incomingReadings.r;
}

void setup() {
  // put your setup code here, to run once:

  //sets all the pins to control the motor to be output pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  // Init Serial Monitor
  Serial.begin(115200);

  // Init OLED display
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // put your main code here, to run repeatedly:

  //this loop will decide which way to turn eventually

  //for now just moves car depending on what method is called
  if(incomingReadings.p > 40)
  {
    backward();
  }
  else if(incomingReadings.p < -40)
  {
    forward();
  }
  else if(incomingReadings.r > 40)
  {
    left();
  }
  else if(incomingReadings.r < -40)
  {
    right();
  }
  else
  {
    off();
  }
}


//forward backward left right move set the direction of the wheels to control movmenet
void forward(){
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}
void backward(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
void left(){
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
void right(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}
//turns all the motors off
void off(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

//===================================
//code for esp connected to gyroscope
//esp connection code based on tutorial at https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/ 
//gyroscope calculations are cut down version of code from https://www.instructables.com/Arduino-MPU6050-GY521-6-Axis-Accelerometer-Gyro-3D/
//===================================
#include "I2Cdev.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;

#define INTERRUPT_PIN 33  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 26 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//esp data
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xAA, 0xE8, 0x50};//remeber to change to your mac adress

// Define variables to store BME280 readings to be sent
float yaw;
float pitch;
float roll;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float y;
    float p;
    float r;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message Readings;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

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

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
  
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
  
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
}

void loop() {
    getReadings();
   
    // Set values to send
    Readings.y = yaw;
    Readings.p = pitch;
    Readings.r = roll;
    Serial.println(yaw);
  
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));
     
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    delay(400);
}
void getReadings(){
  // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[0] * 180/M_PI;
            pitch = ypr[1] * 180/M_PI;
            roll = ypr[2] * 180/M_PI;
    }
}
```
  
# First Milestone
  

The first milestone was setting up and getting the car to move.  I had to test all the motors and switch the positive and negative wires in order to get them all the spin the right way.  Instead of using an esp as the main controller, I used the arduino uno because I didn't have a way to power the esp since it requires 3.3v not 5v.  I had to use a motor control board which supplies the correct voltage to the motors.  It also allows you to easily control the direction of the motor.  In order to get the full potential out of the motors I had to use two battery packs (each with 4x AA batteries), which supplied a total voltage of 12v.  Now the car just follows some preprogrammed instructions to move in all the different directions.

[![Milestone1](https://res.cloudinary.com/marcomontalbano/image/upload/v1626441353/video_to_markdown/images/youtube--K7e2xXXm9xE-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=K7e2xXXm9xE "Milestone1"){:target="_blank" rel="noopener"}

```c++

#include "WiFi.h"

//assigns the pins to control the left motors
int motor1pin1 = 2;//26
int motor1pin2 = 3;//25

//assigns the pins to control the right motors
int motor2pin1 = 4;//33
int motor2pin2 = 5;//32


void setup() {
  // put your setup code here, to run once:

  //sets all the pins to control the motor to be output pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  //this loop will decide which way to turn eventually

  //for now just moves car depending on what method is called
  delay(10000);
  forward();
  delay(1500);
  off();
  delay(100);
  backward();
  delay(1000);
  off();
  delay(100);
  left();
  delay(1500);
  off();
  delay(100);
  right();
  delay(1000);
  off();
  delay(100);
  off();
}


//forward backward left right move set the direction of the wheels to control movmenet
void forward(){
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}
void backward(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
void left(){
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
void right(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}
//turns all the motors off
void off(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}
```
