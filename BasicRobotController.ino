#include "HBridge.h"
#include "MPU6050.h"
#include "Led.h"
#include "Utils.h"
#include "PID.h"

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

#define AHEAD 1
#define BACKWARD -1
#define LEFT 1
#define RIGHT -1


float fullRotationX = 11385;

HBridge *motors;
MPU6050* mpu6050;
Led* debugLed;
PID* pid;
PID* forwardPid;

int state = NONE;
long forwarding_start = 0;
float forward_target_angle = 0;
float forward_direction = 1; //1 = forward, -1 = backward
float rotate_target_angle = 0;
long rotating_start = 0;
char lastMovementCmd = '?';
long forwarding_time = seconds(4.0), rotating_time = seconds(2);
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

  pid = new PID(50, 100, 2, 1);
  forwardPid = new PID(1, 0.1, 0.1, 2);
  
#ifdef USE_SERIAL
  Serial.println("Ready!");
#endif

forward(AHEAD, 0, seconds(60));
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
void updateStateMachine() {
  if(state == NONE) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  } else if(state == SHUTDOWN_MOTORS) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    debugLed->pulse(0.5);
    motors->stop();
    state = NONE;
    
  } else if(state == FORWARD) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    state = FORWARDING;
    forwarding_start = now();
    forwardPid->reset();
    
  } else if(state == FORWARDING) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    debugLed->pulse(10);
    if(!isExpired(forwarding_start, forwarding_time)) {
      float error = (mpu6050->angleX - ((forward_target_angle / 360.0f) * fullRotationX)) / 360.0;
      float attenuation = forwardPid->update(error);
      motors->leftPower(forward_direction - attenuation);
      motors->rightPower(forward_direction + attenuation);
    } else {
      state = SHUTDOWN_MOTORS;
    }
   
  }  if(state == ROTATE) {

    state = ROTATING;
    rotating_start = now();
    pid->reset();
    
  } else if(state == ROTATING) { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    debugLed->pulse(20);
    if(!isExpired(rotating_start, rotating_time)) {
      float error = mpu6050->angleX - ((rotate_target_angle / 360.0f) * fullRotationX);
      float power = pid->update(error/fullRotationX);
      motors->leftPower(-power);
      motors->rightPower(power);
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
    } else if(v == 'p') {
      float p = Serial.parseInt();
      float i = Serial.parseInt();
      float d = Serial.parseInt();
      forwardPid->setParameters(p / 10.0, i / 10.0, d / 10.0);
      Serial.print(p); Serial.print(" ,"); Serial.print(i); Serial.print(" ,"); Serial.print(d);
      Serial.println("");
    }

    if(v == 'h') {
      if(state == FORWARDING) {          //keep goin straight while decelerating
        keep_position(seconds(1));
        //shutdown_motors();
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
      rotate(RIGHT, 90, seconds(2), true);
      Serial.println("d");
    } else if(v == 'a') {
      rotate(LEFT, 90, seconds(2), true);
      Serial.println("a");
    } else if(v == 'o') {
      keep_position(seconds(60));
      Serial.println("o");
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
void rotate(float direction, float target_angle, long duration, bool resetAngles) {
  state = ROTATE;
  rotate_target_angle = direction * target_angle;
  rotating_time = duration;
  if(resetAngles) {
    mpu6050->resetAngles();
  }
}
void keep_position(long duration) {
  rotate(LEFT, 0, duration, false);
}
void shutdown_motors() {
  state = SHUTDOWN_MOTORS;
}
