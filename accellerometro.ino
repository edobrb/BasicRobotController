#include "DigitalButton.h"
#include "HBridge.h"
#include "MPU6050.h"
#include "Led.h"

#define USE_SERIAL
#define BAUD_RATE 9600


#define LEFT_MOTOR_PIN 5
#define RIGHT_MOTOR_PIN 6
#define LEFT_MOTOR_INV_PIN 11
#define RIGHT_MOTOR_INV_PIN 10
#define BUTTON1_PIN 7
#define BUTTON2_PIN 8
#define BUTTON3_PIN 9


//States
#define NONE 0
#define FORWARD 1
#define FORWARDING 2
#define FORWARD2 3
#define FORWARDING2 4
#define ROTATE360 5
#define ROTATING360 6
#define SHUTDOWN_MOTORS 7
#define ROTATING360_FINALIZING 6
#define FORWADING_TIME 4000
#define PDI_PROPORTIONAL_FACTOR 0.002f //Higher means more aggresive correction





float fullRotationX = 11385;

Button *btn1, *btn2, *btn3;
HBridge *motors;
MPU6050* mpu6050;
Led* debugLed;

int state = 0;
long forwarding_start = 0;
float target_angle = 0;


void setup(){
  #ifdef USE_SERIAL
  Serial.begin(BAUD_RATE);
  Serial.println("Ready!");
  #endif
  debugLed = new Led(13);
  debugLed->pulse(0.5);
  
  motors = new HBridge(LEFT_MOTOR_PIN, LEFT_MOTOR_INV_PIN, RIGHT_MOTOR_PIN, RIGHT_MOTOR_INV_PIN);
  
  btn1 = new DigitalButton(BUTTON1_PIN);
  btn1->addHandler(&btn1_handler);
  btn2 = new DigitalButton(BUTTON2_PIN);
  btn2->addHandler(&btn2_handler);
  btn3 = new DigitalButton(BUTTON3_PIN);
  btn3->addHandler(&btn3_handler);
  
  mpu6050 = new MPU6050();
  mpu6050->autocalibration(true);
}

void loop() {
  debugLed->update();
  checkButtons();
  mpu6050->update();
  handleCommands();
  updateStateMachine();
  
  
  #ifdef USE_SERIAL
  Serial.flush();
  #endif
}

void updateStateMachine() {
  switch(state) {
    case FORWARD:
      forwarding_start = millis();
      target_angle = 0;
      state = FORWARDING;
    break;
    case FORWARDING:
     debugLed->pulse(10);
     if((millis() - forwarding_start) < FORWADING_TIME) {
       float x = (mpu6050->angleX + (target_angle * fullRotationX / 360)) * PDI_PROPORTIONAL_FACTOR;
       motors->leftPower(1 + x);
       motors->rightPower(1 - x);
     } else {
       state = SHUTDOWN_MOTORS;
     }
    break;
    case ROTATE360:
      mpu6050->resetAngles();
      state = ROTATING360;
    break;
    case ROTATING360:
      debugLed->pulse(20);
      if(abs(mpu6050->angleX) < fullRotationX) {
         motors->leftPower(-1);
         motors->rightPower(1);
      } else {
        state = SHUTDOWN_MOTORS;
      }
    break;
    case SHUTDOWN_MOTORS:
      debugLed->pulse(0.5);
      motors->stop();
      state = NONE;
    break;
    case NONE:
    break;
  }
}
void handleCommands() {
  #ifdef USE_SERIAL
  if(Serial.available() > 0) {
    char v = (char)Serial.read();
    if(v == 'k') {
      mpu6050->resetAngles();
      Serial.println("Resetted X angle reference");
    } else if(v == 'm') {
      Serial.print("Angle X: "); Serial.println(mpu6050->angleX);
      Serial.print("Avg GyX: "); Serial.println(mpu6050->getAvgGyX());
    } else if(v == 'j') {
      fullRotationX = mpu6050->angleX;
      Serial.print("360 rotation set to: "); Serial.println(abs(fullRotationX));
    }


    if(v == 'h') {
      state = SHUTDOWN_MOTORS;
      Serial.println("SHUTDOWN_MOTORS");
    } else if(v == 'w') {
      state = FORWARD;
      Serial.println("FORWARD");
    }
  }
  #endif
}
void checkButtons() {
  btn1->check();
  btn2->check();
  btn3->check();
}
void btn1_handler(ButtonState s) {
  if(s == ButtonPressed) {
    mpu6050->recalibrate(1.0f);
    mpu6050->resetAngles();
    mpu6050->resetAvgs();
    state = SHUTDOWN_MOTORS;
  }
}
void btn2_handler(ButtonState s) {
  if(s == ButtonPressed) {
    mpu6050->resetAngles();
    state = FORWARD;
  }
}
void btn3_handler(ButtonState s) {
  if(s == ButtonPressed) {
    state = ROTATE360;
  }
}
