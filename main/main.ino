struct Vector {
  double x, y, z;

  Vector(const double& _x, const double& _y, const double& _z)
    : x(_x), y(_y), z(_z) {}
};

struct Motor {
  Vector pos;
  Vector dir;
  double mag;
  const int pin;

  Motor(const double& pos_x = 0, const double& pos_y = 0, const double& poz_z = 0,
        const double& dir_x = 0, const double& dir_y = 0, const double& dir_z = 0,
        const double& _mag = 0, const int& _pin = 0)
    : pos(pos_x, pos_y, poz_z), dir(dir_x, dir_y, dir_z), mag(_mag), pin(_pin) {}

  Motor(const Vector& _pos, const Vector& _dir, const double& _mag, const int& _pin)
    : pos(_pos), dir(_dir), mag(_mag), pin(_pin) {}
};

Vector Moment(const Motor&);

const int N_MOTORS = 6;
Motor Motors[N_MOTORS] = {
  {},
  {},
  {},
  {},
  {},
  {}
};

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

Vector Moment(const Motor& m) {
  // TODO: multiply by mag somehow
  return {m.pos.y * m.dir.z - m.pos.z * m.dir.y,
          -(m.pos.x * m.dir.z - m.pos.z * m.dir.x),
          m.pos.x * m.dir.y - m.pos.y * m.dir.x};
}
