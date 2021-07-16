# Gesture Controlled Car
The car is controlled by the way the person moves their arm.  If the person moves their arm forward or backwards it will make the car move.  If the person turns their arm it will make the car turn in the same direction.

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Ryan Dalal | Edgemont Highschool | Computer Science | Incoming Sophmore

# Final Milestone
  
Incomplete

![Contruction](https://t3.ftcdn.net/jpg/03/44/17/18/360_F_344171869_h3nxznW93zBoOLuMeIJ3Q3xzanFSN8vu.jpg)

# Second Milestone
  
The second milestone was getting the two esp modules to connect and be able to send data back and forth.  This will allow for wirless communication from the persons arm and gyroscope to the motors.

![Contruction](https://t3.ftcdn.net/jpg/03/44/17/18/360_F_344171869_h3nxznW93zBoOLuMeIJ3Q3xzanFSN8vu.jpg)
  
# First Milestone
  

The first milestone was setting up and getting the car to move.  I had to test all the motors and switch the positive and negative wires in order to get them all the spin the right way.  Instead of using an esp as the main controller, I used the arduino uno because I didn't have a way to power the esp since it requires 3.3v not 5v.  I had to use a motor control board which supplies the correct voltage to the motors.  It also allows you to easily control the direction of the motor.  In order to get the full potential out of the motors I had to use two battery packs (each with 4x AA batteries), which supplied a total voltage of 12v.  Now the car just follows some preprogrammed instructions to move in all the different directions.

![Contruction](https://vcunited.club/wp-content/uploads/2020/01/No-image-available-2.jpg)

```


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
