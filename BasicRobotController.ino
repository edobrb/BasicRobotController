#include "HBridge.h"
#include "MPU6050.h"
#include "Led.h"
#include "Utils.h"

#define USE_SERIAL
#define BAUD_RATE 9600


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


#define AHEAD 1
#define BACKWARD -1
#define LEFT -1
#define RIGHT 1


float fullRotationX = 11385;

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
char lastMovementCmd = '?';
long forwarding_time = seconds(4.0), keep_position_time = seconds(0.5);
void setup(){
#ifdef USE_SERIAL
  Serial.begin(BAUD_RATE);
  Serial.println("Initializing...");
#endif

  debugLed = new Led(13);
  debugLed->pulse(0.5);
  
  motors = new HBridge(LEFT_MOTOR_PIN, LEFT_MOTOR_INV_PIN, RIGHT_MOTOR_PIN, RIGHT_MOTOR_INV_PIN);
  
  mpu6050 = new MPU6050();
  mpu6050->autocalibration(true);

#ifdef USE_SERIAL
  Serial.println("Ready!");
#endif
}

void loop() {
  debugLed->update();
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
  if(state == NONE) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    
    
  } else if(state == SHUTDOWN_MOTORS) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    debugLed->pulse(0.5);
    motors->stop();
    state = NONE;
    
  } else if(state == FORWARD) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    forwarding_start = now();
    state = FORWARDING;
    
  } else if(state == FORWARDING) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    debugLed->pulse(10);
    if(!isExpired(forwarding_start, forwarding_time)) {
      float x = (mpu6050->angleX + (forward_target_angle * fullRotationX / 360)) * PDI_PROPORTIONAL_FACTOR;
      motors->leftPower(forward_direction + x);
      motors->rightPower(forward_direction - x);
    } else {
      state = SHUTDOWN_MOTORS;
    }
   
  }  if(state == ROTATE) {

    state = ROTATING;
    rotating_ok_micros = -1;
    
  } else if(state == ROTATING) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    
  } else if(state == KEEP_POSITION) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    state = KEEPING_POSITION;
    keep_position_start = now();
     
  } else if(state == KEEPING_POSITION) { /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(!isExpired(keep_position_start, keep_position_time)) {
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
    
  } ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
      if(state == FORWARDING) {          //keep goin straight while decelerating
        keep_position(seconds(0.5));
      } else {                           //no need to counter rotate
        shutdown_motors();
      }
      Serial.println("h");
    } else if(v == 'w') {
      forward(AHEAD, 0, seconds(60));
      Serial.println("w");
    } else if(v == 's') {
      forward(BACKWARD, 0, seconds(60));
      Serial.println("s");
    } else if(v == 'd') {
      rotate(LEFT, 90);
      Serial.println("d");
    } else if(v == 'a') {
      rotate(RIGHT, 90);
      Serial.println("a");
    }

    if(v == 'w' || v == 's' || v == 'a' || v == 'd') {
      lastMovementCmd = v;
    }
  }
  #endif
}

/* ACTIONS */
void forward(float direction, float target_angle, long duration) {
  state = FORWARD;
  forward_direction = direction;
  forward_target_angle = target_angle;
  if(lastMovementCmd == 'a' || lastMovementCmd == 'd') {
    mpu6050->resetAngles();
  }
  forwarding_time = duration;
}
void rotate(float direction, float target_angle) {
  state = ROTATE;
  rotate_target_angle = target_angle;
  rotate_direction = direction;
  mpu6050->resetAngles();
}
void keep_position(long duration) {
  keep_position_time = duration;
  state = KEEP_POSITION;
}
void shutdown_motors() {
  state = SHUTDOWN_MOTORS;
}
