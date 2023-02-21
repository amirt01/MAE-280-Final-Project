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

struct Motor {
  Vector pos = {0, 0, 0};
  Vector dir = {0, 0, 0};
  double mag = 0;
  Servo controller;

  Motor() = default;

  Motor(const double& pos_x, const double& pos_y, const double& poz_z,
        const double& dir_x, const double& dir_y, const double& dir_z,
        const double& _mag)
    : pos(pos_x, pos_y, poz_z), dir(dir_x, dir_y, dir_z), mag(_mag) {}

  Motor(const Vector& _pos, const Vector& _dir, const double& _mag)
    : pos(_pos), dir(_dir), mag(_mag) {}
};

Vector Moment(const Motor&);

void ReadSensors();  // read accelerometer/gyro and depth
void UpdateVars();   // calculate and update variables 
void UpdateMotors(); // write to all the motors

const int N_MOTORS = 4;
Motor motors[N_MOTORS] = {
  {},
  {},
  {},
  {}
};

Vector target_direction(0, 0, 0);
Vector true_direction(0, 0, 0);

void setup() {
  for(int i = 0; i < N_MOTORS; i++) { motors[i].controller.attach(PWM_PINS[i]); }
}

void loop() {
  // put your main code here, to run repeatedly:

  // take in data
  // process data
  UpdateMotors();

  for (const Motor& motor : motors) { motor.controller.write(motor.mag); }
}

Vector Moment(const Motor& m) {
  // TODO: multiply by mag somehow
  return {m.pos.y * m.dir.z - m.pos.z * m.dir.y,
          -(m.pos.x * m.dir.z - m.pos.z * m.dir.x),
          m.pos.x * m.dir.y - m.pos.y * m.dir.x};
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