struct Vector {
  double x, y, z;

  Vector(const double& _x, const double& _y, const double& _z)
    : x(_x), y(_y), z(_z) {}
};

struct Motor {
  Vector position;
  Vector magnitude;

  Motor(const double& pos_x = 0, const double& pos_y = 0, const double& poz_z = 0,
        const double& mag_x = 0, const double& mag_y = 0, const double& mag_z = 0)
    : position(pos_x, pos_y, poz_z), magnitude(mag_x, mag_y, mag_z) {}

  Motor(Vector pos, Vector mag) : position(pos), magnitude(mag) {}
};

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
