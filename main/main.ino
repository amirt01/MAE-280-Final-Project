struct Vector {
  double x, y, z;

  Vector(const double& _x, const double& _y, const double& _z)
    : x(_x), y(_y), z(_z) {}
};

struct Motor {
  Vector pos;
  Vector mag;

  Motor(const double& pos_x = 0, const double& pos_y = 0, const double& poz_z = 0,
        const double& mag_x = 0, const double& mag_y = 0, const double& mag_z = 0)
    : pos(pos_x, pos_y, poz_z), mag(mag_x, mag_y, mag_z) {}

  Motor(Vector pos, Vector mag) : pos(pos), mag(mag) {}
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
  return {m.pos.y * m.mag.z - m.pos.z * m.mag.y,
          -(m.pos.x * m.mag.z - m.pos.z * m.mag.x),
          m.pos.x * m.mag.y - m.pos.y * m.mag.x};
}
