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
const float MPU_KD_X = 0.1f;
const float MPU_KD_Y = 0.1f;

Pixy2 pixy;
const uint16_t center_x = 157;
const uint16_t center_y = 103;
uint16_t target_x;
uint16_t target_y;
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
#define CH4 10
#define CH5 8

unsigned long duration1;
unsigned long duration2;
unsigned long duration3;
unsigned long duration4;
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

// ================================================================
// ===                       OPERATIONS                         ===
// ================================================================

const unsigned int LAUNCH_DELAY = 3000;

#define LED_PIN 13
bool blinkState = false;
unsigned long toggle_time = 0;
constexpr unsigned int LED_INCREMENT = 300;

float acceleration_vector[2];

enum Control_Mode { manual, autonomous } control_mode;

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

void Update_Blink();

// ================================================================
// ===                          SETUP                           ===
// ================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // will pause until serial console opens
  
  MPU_Setup();
  LPS_Setup();
  Servo_Setup();
  Operations_Setup();
  Channel_Setup();

  Serial.println();

  pixy.init();

  delay(LAUNCH_DELAY);
}

// ================================================================
// ===                           RUN                            ===
// ================================================================

void loop() {
  switch (control_mode)
  {
  case Control_Mode::manual:
    Get_RC_Values();
    Update_Servo_Manual();
    break;
  case Control_Mode::autonomous:
    Get_LPS_Data();
    Get_MPU_Data();
    Get_Pixy_Data();

    Calculate_Acceleration_Vector();
    Update_Servos();
    break;
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
  esc1.attach(ESC1_PIN);
  esc2.attach(ESC2_PIN);
  esc3.attach(ESC3_PIN);
  esc4.attach(ESC4_PIN);

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

  control_mode = Control_Mode::manual;

  Serial.println("Operations Initialized");
}

void Channel_Setup(){
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  //pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
}

void Get_RC_Values() {
  duration1 = pulseIn(CH1, HIGH);
  duration2 = pulseIn(CH2, HIGH);
  duration3 = pulseIn(CH3, HIGH);
  duration4 = pulseIn(CH4, HIGH, 3000);
  duration5 = pulseIn(CH5, HIGH, 3000);
}

void Update_Servo_Manual() {
  // Pitch
  if (duration2 >= 1350 && duration2 <= 1600) {
    esc1.write(NEUTRAL);
    esc2.write(NEUTRAL);
    esc3.write(NEUTRAL);
    esc4.write(NEUTRAL);
  } else {
    CH2_sig = map(duration2, 1000, 2000, 0, 180);
    esc1.write(NEUTRAL);
    esc2.write(CH2_sig);
    esc3.write(CH2_sig);
    esc4.write(NEUTRAL);
  }

  // Yaw
  CH1_sig = map(duration1,1000,2000,0,180);
  esc3.write(CH2_sig);
  esc4.write(180 - CH2_sig);

  // Thrust
  CH3_sig = map(duration3, 1000, 2000, 0, 180);
  esc1.write(CH3_sig);
  esc4.write(CH3_sig);

  CH2_sig = map(duration2, 1000, 2000, 0, 180);
  esc2.write(CH2_sig);
  esc3.write(CH2_sig);
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

  Serial.print("Rotation X: ");
  Serial.print(gyro_x);
  Serial.print(", Y: ");
  Serial.print(gyro_y);
  Serial.print(", Z: ");
  Serial.print(gyro_z);
  Serial.println(" rad/s");
}

void Get_Pixy_Data() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    target_x = pixy.ccc.blocks[0].m_x;
    target_y = pixy.ccc.blocks[0].m_y;
  }

  Serial.print(target_x);
  Serial.print(" | ");
  Serial.println(target_y);
}

void Calculate_Acceleration_Vector() {
  // initially set to pixy x, y
  acceleration_vector[0] = (target_x - center_x) * PIXY_KP_X;
  acceleration_vector[1] = (target_y - center_y) * PIXY_KP_Y;

  // if too deep, pitch up | if too shallow, pitch down
  if (pressure > target_pressure) acceleration_vector[1] += (pressure - target_pressure) * LPS_KP;
  else if (pressure < target_pressure) acceleration_vector[1] += (target_pressure - pressure) * LPS_KP;
}

void Update_Servos() {
  // calculate new servos
  long x_servo_power = map(0, 315, MAX_NEG, MAX_POS, acceleration_vector[0]);
  long y_servo_power = map(0, 207, MAX_NEG, MAX_POS, acceleration_vector[1]);

  // limit based on gyro
  if (x_servo_power > NEUTRAL && gyro_x > 10)
    x_servo_power -= (x_servo_power - NEUTRAL) * MPU_KD_X + NEUTRAL;
  else if (y_servo_power > NEUTRAL && gyro_x > 10)
    y_servo_power -= (y_servo_power - NEUTRAL) * MPU_KD_Y + NEUTRAL;
  
  // Write to the servos
  esc1.write(NEUTRAL);
  esc2.write(NEUTRAL);
  esc3.write(NEUTRAL);
  esc4.write(NEUTRAL);
}

// Blink every second
void Update_Blink() {
  if (millis() < toggle_time) return;

  // Update LED
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  toggle_time += LED_INCREMENT;
}
