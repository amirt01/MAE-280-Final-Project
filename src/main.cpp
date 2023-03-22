#include <Arduino.h>
#include <Servo.h>
#include <Pixy2.h>

#include <Adafruit_LPS35HW.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ================================================================
// ===                     Sensor VARIABLES                     ===
// ================================================================

Adafruit_LPS35HW lps = Adafruit_LPS35HW();
const float target_pressure = 62.4 * 2;  // 62.4 lbs/ft3 * 2 ft deep
float pressure = 0.f;
const float LPS_KP = 0.f;

Adafruit_MPU6050 mpu;
float gyro_x;
float gyro_y;
float gyro_z;
const float MPU_KD_X = 0.f;
const float MPU_KD_Y = 0.f;

Pixy2 pixy;
const int16_t CENTER_X = 157;
const int16_t CENTER_Y = 103;
int16_t target_x;
int16_t target_y;
const float PIXY_KP_X = 1.f;
const float PIXY_KP_Y = 1.f;

// ================================================================
// ===                    RECIEVER VARIABLES                    ===
// ================================================================

int CH1_sig;
int CH2_sig;
int CH3_sig;
int CH4_sig;

#define CH1 2
#define CH2 4
#define CH3 7
#define CH5 8

unsigned long duration1;
unsigned long duration2;
unsigned long duration3;
unsigned long duration5;

// ================================================================
// ===                     Motor VARIABLES                      ===
// ================================================================

#define ESC1_PIN 9
#define ESC2_PIN 6
#define ESC3_PIN 5
#define ESC4_PIN 3

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

const short NEUTRAL = 90;
const short MAX_NEG = 0;
const short MAX_POS = 180;
const short MIN_FREQ = 1000;
const short MAX_FREQ = 2000;

// ================================================================
// ===                       OPERATIONS                         ===
// ================================================================

const unsigned int LAUNCH_DELAY = 3000;

#define LED_PIN 13
bool blinkState = false;
unsigned long toggle_time = 0;
constexpr unsigned int LED_INCREMENT = 300;

float acceleration_vector[2];
bool arrived = false;

// ================================================================
// ===                  FUNCTION DECLERATIONS                   ===
// ================================================================

void MPU_Setup();
void LPS_Setup();
void Servo_Setup();
void Operations_Setup();
void Channel_Setup();

void Get_RC_Values();
void Update_Servo_Manual();

void Get_LPS_Data();
void Get_MPU_Data();
void Get_Pixy_Data();

void Calculate_Acceleration_Vector();
void Update_Servos();
void Surface();

void Update_Blink();

// ================================================================
// ===                          SETUP                           ===
// ================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // will pause until serial console opens
  
  MPU_Setup();
  LPS_Setup();
  const unsigned int INITIAL_PRESSURE = lps.readPressure();
  Servo_Setup();
  Operations_Setup();
  Channel_Setup();

  pixy.init();
  pixy.setLamp(1,1);

  Serial.println();

  delay(LAUNCH_DELAY);
}

// ================================================================
// ===                           RUN                            ===
// ================================================================

void loop() {
  duration5 = pulseIn(CH5, HIGH);
  
  if (duration5 > 1800 || duration5 == 0) {
    Serial.println("AUTONOMOUS");

    Get_LPS_Data();
    Get_MPU_Data();
    Get_Pixy_Data();
    
    if (arrived) {
      Surface();
      return;
    }

    Calculate_Acceleration_Vector();
    Update_Servos();
  } else if (duration5 < 1100) {
    Serial.println("MANUAL");

    Get_RC_Values();
    Update_Servo_Manual();
  } else {
    Serial.println("WTF u doin'...");
  }
  
  Update_Blink();
}

// ================================================================
// ===                   FUNCTION DEFINITIONS                   ===
// ================================================================

void MPU_Setup() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (true) delay(10);
  }
  
  // TODO: find the best values for these
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU Initialized");
}

void LPS_Setup() {
  if (!lps.begin_I2C()) {
    Serial.println("Failed to find LPS33HW chip");
    while (true) delay(10);
  }

  Serial.println("Found LPS33HW chip");
}

void Servo_Setup() {
  esc1.attach(ESC1_PIN, MIN_FREQ, MAX_FREQ);
  esc2.attach(ESC2_PIN, MIN_FREQ, MAX_FREQ);
  esc3.attach(ESC3_PIN, MIN_FREQ, MAX_FREQ);
  esc4.attach(ESC4_PIN, MIN_FREQ, MAX_FREQ);

  esc1.write(NEUTRAL);
  esc2.write(NEUTRAL);
  esc3.write(NEUTRAL);
  esc4.write(NEUTRAL);

  Serial.println("Servos Initialized");
}

void Operations_Setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, blinkState);
  toggle_time = millis() + 1000;

  Serial.println("Operations Initialized");
}

void Channel_Setup() {
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  // pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);

  Serial.println("Channels Initialized");
}

void Get_RC_Values() {
  duration1 = pulseIn(CH1, HIGH);
  duration2 = pulseIn(CH2, HIGH);
  duration3 = pulseIn(CH3, HIGH);

  Serial.print("RC Values: (");
  Serial.print(duration1);
  Serial.print(", ");
  Serial.print(duration2);
  Serial.print(", ");
  Serial.print(duration3);
  Serial.println(")");
}

void Update_Servo_Manual() {
  // Depth
  CH3_sig = map(duration3, MIN_FREQ, MAX_FREQ, MAX_NEG, MAX_POS);
  esc1.write(CH3_sig);
  esc3.write(CH3_sig);

  // Thrust
  CH2_sig = map(duration2, MIN_FREQ, MAX_FREQ, MAX_NEG, MAX_POS);
  esc2.write(CH2_sig);
  esc4.write(CH2_sig);

  // Yaw
  CH1_sig = map(duration1, MIN_FREQ, MAX_FREQ, MAX_NEG, MAX_POS);
  if (CH1_sig > 90) {
    esc1.write(CH1_sig);
    esc3.write(MAX_POS - CH1_sig);
  } else {
    esc1.write(MAX_POS - CH1_sig);
    esc3.write(CH1_sig);
  }
}

void Get_LPS_Data() {
  pressure = lps.readPressure();

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");
}

void Get_MPU_Data() {
  sensors_event_t g;
  mpu.getGyroSensor()->getEvent(&g);

  gyro_x = g.gyro.x;
  gyro_y = g.gyro.x;
  gyro_z = g.gyro.x;

  Serial.print("Rotation (");
  Serial.print(gyro_x);
  Serial.print(", ");
  Serial.print(gyro_y);
  Serial.print(", ");
  Serial.print(gyro_z);
  Serial.println(") rad/s");
}

void Get_Pixy_Data() {
  pixy.ccc.getBlocks();

  if (!pixy.ccc.numBlocks) {
    target_x = 2 / 3 * CENTER_X;
    target_y = CENTER_Y;
  } else {
    target_x = pixy.ccc.blocks[0].m_x;
    target_y = pixy.ccc.blocks[0].m_y;

    if (pixy.ccc.blocks[0].m_width > 300)
      arrived = true;
  }

  Serial.print("Pixy Data: (");
  Serial.print(target_x);
  Serial.print(", ");
  Serial.print(target_y);
  Serial.println(")");
}

void Calculate_Acceleration_Vector() {
  // initially set to pixy x, y
  acceleration_vector[0] = (target_x - CENTER_X) * PIXY_KP_X;
  acceleration_vector[1] = (target_y - CENTER_Y) * PIXY_KP_Y;

  // if too deep, pitch up | if too shallow, pitch down
  if (pressure > target_pressure) acceleration_vector[1] += (pressure - target_pressure) * LPS_KP;
  else if (pressure < target_pressure) acceleration_vector[1] += (target_pressure - pressure) * LPS_KP;

  // limit based on gyro
  if (acceleration_vector[0] > NEUTRAL && gyro_x > 10)
    acceleration_vector[0] -= (acceleration_vector[0] - NEUTRAL) * MPU_KD_X;
  else if (acceleration_vector[1] > NEUTRAL && gyro_x > 10)
    acceleration_vector[1] -= (acceleration_vector[1] - NEUTRAL) * MPU_KD_Y;

  Serial.print("Acceleration Vector: (");
  Serial.print(acceleration_vector[0]);
  Serial.print(", ");
  Serial.print(acceleration_vector[1]);
  Serial.println(")");
}

void Update_Servos() {
  int esc1_val = map(acceleration_vector[0], -CENTER_X, CENTER_X, 180, 0) + 45;
  int esc2_val = map(acceleration_vector[0], -CENTER_X, CENTER_X, 0, 180) + 45;
  int esc3_val = map(acceleration_vector[1], -CENTER_Y, CENTER_Y, 180, 0);
  int esc4_val = map(acceleration_vector[1], -CENTER_Y, CENTER_Y, 180, 0);

  // Write to the servos
  esc1.write(esc1_val);
  esc3.write(esc2_val);
  esc2.write(esc3_val);
  esc4.write(esc4_val);
}

// Return to the surface
void Surface() {
  esc1.write(90);
  esc3.write(90);
  esc2.write(120);
  esc4.write(120);
}

// Blink every second
void Update_Blink() {
  if (millis() < toggle_time) return;

  // Update LED
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  toggle_time += LED_INCREMENT;
}
