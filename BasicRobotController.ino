
#include "DigitalButton.h"
#include "HBridge.h"
#include "MPU6050.h"
#include "Led.h"


#define USE_SERIAL
#define BAUD_RATE 9600

#define USE_DEBUG_BUTTONS
#define BUTTON1_PIN 7
#define BUTTON2_PIN 8
#define BUTTON3_PIN 9


//HBridge pins
#define LEFT_MOTOR_PIN 5
#define RIGHT_MOTOR_PIN 6
#define LEFT_MOTOR_INV_PIN 11
#define RIGHT_MOTOR_INV_PIN 10


//States
#define NONE 0
#define FORWARD 1
#define FORWARDING 2
#define ROTATE 10
#define ROTATING 11
#define SHUTDOWN_MOTORS 20


#define FORWADING_TIME 4000
#define PDI_PROPORTIONAL_FACTOR 0.002f //Higher means more aggresive correction





float fullRotationX = 11385;

#ifdef USE_DEBUG_BUTTONS
Button *btn1, *btn2, *btn3;
#endif
HBridge *motors;
MPU6050* mpu6050;
Led* debugLed;

int state = 0;
long forwarding_start = 0;
float forward_target_angle = 0;
float forward_direction = 1; //1 = forward, -1 = backward
float rotate_target_angle = 0;
float rotate_direction = 1;
long rotating_ok_micros = -1;
void setup(){
#ifdef USE_SERIAL
  Serial.begin(BAUD_RATE);
  Serial.println("Initializing...");
#endif

  debugLed = new Led(13);
  debugLed->pulse(0.5);
  
  motors = new HBridge(LEFT_MOTOR_PIN, LEFT_MOTOR_INV_PIN, RIGHT_MOTOR_PIN, RIGHT_MOTOR_INV_PIN);

#ifdef USE_DEBUG_BUTTONS
  btn1 = new DigitalButton(BUTTON1_PIN);
  btn1->addHandler(&btn1_handler);
  btn2 = new DigitalButton(BUTTON2_PIN);
  btn2->addHandler(&btn2_handler);
  btn3 = new DigitalButton(BUTTON3_PIN);
  btn3->addHandler(&btn3_handler);
#endif
  
  mpu6050 = new MPU6050();
  mpu6050->autocalibration(true);

#ifdef USE_SERIAL
  Serial.println("Ready!");
#endif
}

void loop() {
  debugLed->update();
#ifdef USE_DEBUG_BUTTONS
  checkButtons();
#endif
  mpu6050->update();
  handleCommands();
  updateStateMachine();
  
  
  #ifdef USE_SERIAL
  Serial.flush();
  #endif
}
float sign1(float v) {
  return v>0?1:(v<0?-1:0);
}
void updateStateMachine() {
  switch(state) {
    case SHUTDOWN_MOTORS:
      debugLed->pulse(0.5);
      motors->stop();
      state = NONE;
    break;
    case FORWARD:
      forwarding_start = millis();
      forward_target_angle = 0;
      state = FORWARDING;
    break;
    case FORWARDING:
     debugLed->pulse(10);
     if((millis() - forwarding_start) < FORWADING_TIME) {
       float x = (mpu6050->angleX + (forward_target_angle * fullRotationX / 360)) * PDI_PROPORTIONAL_FACTOR;
       motors->leftPower(forward_direction + x);
       motors->rightPower(forward_direction - x);
     } else {
       state = SHUTDOWN_MOTORS;
     }
    break;
    case ROTATE:
      mpu6050->resetAngles();
      state = ROTATING;
      rotating_ok_micros = -1;
    break;
    case ROTATING:
      debugLed->pulse(20);
      float factor = rotate_direction * mpu6050->angleX / ((rotate_target_angle / 360.0f) * fullRotationX) - 1;
      float power = max(0.2f, min(1, abs(factor))); //Need to throw a PID control
      motors->leftPower(rotate_direction * sign1(factor) * power);
      motors->rightPower(-rotate_direction * sign1(factor) * power);
      if(abs(factor) < 0.02 && rotating_ok_micros == -1) {
        rotating_ok_micros = micros();
      }
      if(rotating_ok_micros != -1 && (micros() - rotating_ok_micros) / 1000000.0 > 0.5) {
        state = SHUTDOWN_MOTORS;
      }
      if(abs(factor) > 0.02) {
        rotating_ok_micros = -1;
      }
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
      Serial.println("h");
    } else if(v == 'w') {
      state = FORWARD;
      forward_direction = 1;
      Serial.println("w");
    } else if(v == 's') {
      state = FORWARD;
      forward_direction = -1;
      Serial.println("s");
    } else if(v == 'd') {
      state = ROTATE;
      rotate_target_angle = 90;
      rotate_direction = -1;
      Serial.println("d");
    } else if(v == 'a') {
      state = ROTATE;
      rotate_target_angle = 90;
      rotate_direction = 1;
      Serial.println("a");
    }
  }
  #endif
}

#ifdef USE_DEBUG_BUTTONS
void checkButtons() {
  btn1->check();
  btn2->check();
  btn3->check();
}
bool btn1s = true;
void btn1_handler(ButtonState s) {
  if(s == ButtonPressed) {
    if(btn1s) {
       mpu6050->resetAngles();
    } else {
      fullRotationX = mpu6050->angleX;
    }
    btn1s = !btn1s;
    state = SHUTDOWN_MOTORS;
  }
}
void btn2_handler(ButtonState s) {
  if(s == ButtonPressed) {
    mpu6050->resetAngles();
    forward_direction = 1;
    state = FORWARD;
  }
}
void btn3_handler(ButtonState s) {
  if(s == ButtonPressed) {
    state = ROTATE;
    rotate_direction = 1;
    rotate_target_angle = 90;
  }
}
#endif
