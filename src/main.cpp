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
float lps_temp = 0;
float pressure = 0;

Adafruit_MPU6050 mpu;
float mpu_temp = 0;
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;

Pixy2 pixy;
uint16_t target_x = 0;
uint16_t target_y = 0;
uint16_t target_width = 0;
uint16_t target_width = 0;

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
int del = 1000;
int test = 95;

// ================================================================
// ===                       OPERATIONS                         ===
// ================================================================

const unsigned int LAUNCH_DELAY = 3000;

#define LED_PIN 13
bool blinkState = false;
unsigned long toggle_time = 0;
constexpr unsigned int LED_INCREMENT = 300;

enum class State { StandBye, Running, Resting } state;

// ================================================================
// ===                  FUNCTION DECLERATIONS                   ===
// ================================================================

void MPU_Setup();
void LPS_Setup();
void Servo_Setup();
void Operations_Setup();

void Get_LPS_Data();
void Get_MPU_Data();
void Get_Pixy_Data();

void Calclate_Acceleration_Vector();
void Update_Servos();

void Update_Blink();

// this is a test comment

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

  Serial.println();

  pixy.init();

  delay(LAUNCH_DELAY);
}

// ================================================================
// ===                           RUN                            ===
// ================================================================

void loop() {
  Get_LPS_Data();
  Get_MPU_Data();
  Get_Pixy_Data();
  
  Calculate_Acceleration_Vector();

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

  state = State::StandBye;

  Serial.println("Operations Initialized");
}

void Get_LPS_Data() {
  lps_temp = lps.readTemperature();
  pressure = lps.readPressure();

  /* The temperature that is measured here is usually the temperature
  in the sensor which is used for calibration purposes.*/
  Serial.print("Temperature: ");
  Serial.print(lps_temp);
  Serial.println(" C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");
}

void Get_MPU_Data() {
  // TODO: We most likely only need data from the gyroscope so we can
  //       par this down a lot.
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel_x = a.acceleration.x;
  accel_y = a.acceleration.x;
  accel_z = a.acceleration.x;
  
  gyro_x = g.gyro.x;
  gyro_y = g.gyro.x;
  gyro_z = g.gyro.x;
  
  mpu_temp = temp.temperature;

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(accel_x);
  Serial.print(", Y: ");
  Serial.print(accel_y);
  Serial.print(", Z: ");
  Serial.print(accel_z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(gyro_x);
  Serial.print(", Y: ");
  Serial.print(gyro_y);
  Serial.print(", Z: ");
  Serial.print(gyro_z);
  Serial.println(" rad/s");

  /* The temperature that is measured here is usually the temperature
  in the sensor which is used for calibration purposes.*/
  Serial.print("Temperature: ");
  Serial.print(mpu_temp);
  Serial.println(" degC");
}

void Get_Pixy_Data() {
  // grab blocks!
  pixy.ccc.getBlocks();

  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks) {
    Serial.println("EYOOOOOOOOOO");
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);

    target_x = pixy.ccc.blocks[0].m_x;
    target_y = pixy.ccc.blocks[0].m_y;
  }
}

void Calculate_Acceleration_Vector() {
  
}

// Blink every second
void Update_Blink() {
  if (millis() < toggle_time) return;

  // Update LED
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  toggle_time += LED_INCREMENT;
}
