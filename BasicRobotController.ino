
#include "DigitalButton.h"
#include "HBridge.h"
#include "MPU6050.h"
#include "Led.h"


#define USE_SERIAL
#define BAUD_RATE 9600

//#define USE_DEBUG_BUTTONS
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
#define KEEP_POSITION 30
#define KEEPING_POSITION 31

#define PDI_PROPORTIONAL_FACTOR 0.002f //Higher means more aggresive correction





float fullRotationX = 11385;

#ifdef USE_DEBUG_BUTTONS
Button *btn1, *btn2, *btn3;
#endif
HBridge *motors;
MPU6050* mpu6050;
Led* debugLed;

int state = NONE;
long forwarding_start = 0, keep_position_start = 0;
float forward_target_angle = 0;
float forward_direction = 1; //1 = forward, -1 = backward
float rotate_target_angle = 0;
float rotate_direction = 1;
long rotating_ok_micros = -1;
char lastCmd = '?';
long forwarding_time = 4 * 1000000, keep_position_time = 500000 /*0.5s*/;
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
  if(state == NONE) {

    
    
  } else if(state == SHUTDOWN_MOTORS) {

    debugLed->pulse(0.5);
    motors->stop();
    state = NONE;
    
  } else if(state == FORWARD) {

    forwarding_start = micros();
    state = FORWARDING;
    
  } else if(state == FORWARDING) {
    
    debugLed->pulse(10);
    if((micros() - forwarding_start) < forwarding_time) {
      float x = (mpu6050->angleX + (forward_target_angle * fullRotationX / 360)) * PDI_PROPORTIONAL_FACTOR;
      motors->leftPower(forward_direction + x);
      motors->rightPower(forward_direction - x);
    } else {
      state = SHUTDOWN_MOTORS;
    }
   
  }  if(state == ROTATE) {

    state = ROTATING;
    rotating_ok_micros = -1;
    
  } else if(state == ROTATING) {

    debugLed->pulse(20);
    float factor = rotate_direction * mpu6050->angleX / ((rotate_target_angle / 360.0f) * fullRotationX) - 1;
    float power = max(0.4f, min(1, abs(factor))); //Need to throw a PID control
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
    
  } else if(state == KEEP_POSITION) {
    state = KEEPING_POSITION;
    keep_position_start = micros(); 
  } else if(state == KEEPING_POSITION) {

    if((micros() - keep_position_start) < keep_position_time) {
      debugLed->pulse(1);
      float x = (mpu6050->angleX) * PDI_PROPORTIONAL_FACTOR * 2;
      if(x > 0) {
        motors->leftPower(x);
        motors->rightPower(-x);
      } else {
        motors->leftPower(x);
        motors->rightPower(-x);
      }
    } else {
      state = SHUTDOWN_MOTORS;
    }
    
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
      state = KEEP_POSITION;
      mpu6050->resetAngles();
      Serial.println("h");
    } else if(v == 'w') {
      state = FORWARD;
      forward_direction = 1;
      forward_target_angle = 0;
      if(lastCmd == 'a' || lastCmd == 'd') {
         mpu6050->resetAngles();
      }
      forwarding_time = 60 * 1000000;
      Serial.println("w");
    } else if(v == 's') {
      state = FORWARD;
      forward_direction = -1;
      forward_target_angle = 0;
      if(lastCmd == 'a' || lastCmd == 'd') {
         mpu6050->resetAngles();
      }
      forwarding_time = 60 * 1000000;
      Serial.println("s");
    } else if(v == 'd') {
      state = ROTATE;
      rotate_target_angle = 90;
      rotate_direction = -1;
      mpu6050->resetAngles();
      Serial.println("d");
    } else if(v == 'a') {
      state = ROTATE;
      rotate_target_angle = 90;
      rotate_direction = 1;
      mpu6050->resetAngles();
      Serial.println("a");
    }

    if(v == 'w' || v == 's' || v == 'a' || v == 'd') {
      lastCmd = v;
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
    forwarding_time = 4 * 1000000;
    forward_target_angle = 0;
  }
}
void btn3_handler(ButtonState s) {
  if(s == ButtonPressed) {
    state = ROTATE;
    rotate_direction = 1;
    rotate_target_angle = 90;
    mpu6050->resetAngles();
  }
}
#endif
