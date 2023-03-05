#include <Servo.h>

const int PWM_PINS[] = {3, 5, 6, 9, 10, 11};

struct Vector {
  double x, y, z;

  Vector(const double& _x, const double& _y, const double& _z)
    : x(_x), y(_y), z(_z) {}

  Vector operator- (const Vector& vect) const {
    return {x - vect.x, y - vect.y, z - vect.z};
  }
};

Vector target_direction(0, 0, 0);
Vector true_direction(0, 0, 0);

void ReadSensors();  // read accelerometer/gyro and depth
void UpdateVars();   // calculate and update variables 
void UpdateMotors(); // write to all the motors

struct Motor {
  double mag;
  Servo controller;
};

const int N_MOTORS = 4;
union motors_u {
  struct motors_s {
    Motor forward, aft, left, right;
  };
  Motor motors_a[N_MOTORS];
} motors;

void setup() {
  for(int i = 0; i < N_MOTORS; i++) { motors.motors_a[i].controller.attach(PWM_PINS[i]); }
}

void loop() {
  // put your main code here, to run repeatedly:

  // take in data
  // process data
  UpdateMotors();

  for (const Motor& motor : motors.motors_a) { motor.controller.write(motor.mag); }
}

void UpdateMotors() {
  Vector accel_direction = target_direction - true_direction;
  
  if (accel_direction.x > 0) {       // yaw right
  }
  else if (accel_direction.x < 0) {  // yaw left
  }
  else {                             // go forward
  }

  if (accel_direction.y > 0) {       // yaw right and go forward
  }
  else if (accel_direction.y < 0) {  // yaw left and go forward
  }
  else {                             // go forward
  }
  
  if (accel_direction.z > 0) {       // go up 
  }
  else if (accel_direction.z < 0) {  // go down
  }
  else {                             // go forward
  }
}