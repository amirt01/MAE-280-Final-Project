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

Adafruit_MPU6050 mpu;
Adafruit_LPS35HW lps = Adafruit_LPS35HW();

Pixy2 pixy;

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

constexpr unsigned int LAUNCH_DELAY = 3000;

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

  Serial.println();

  pixy.init();

  delay(LAUNCH_DELAY);
}

// ================================================================
// ===                           RUN                            ===
// ================================================================

void loop() {
  Serial.print("Temperature: ");
  Serial.print(lps.readTemperature());
  Serial.println(" C");

  Serial.print("Pressure: ");
  Serial.print(lps.readPressure());
  Serial.println(" hPa");

  Serial.println();
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  /* esc1.write(test);
  delay(del);
  esc1.write(90);
  delay(del);


  esc2.write(test);
  delay(del);
  esc2.write(90);
  delay(del);

  esc3.write(test);
  delay(del);
  esc3.write(90);
  delay(del);

  esc4.write(test);
  delay(del);
  esc4.write(90);
  delay(del); */

  int i;
  // grab blocks!
  pixy.ccc.getBlocks();

  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks) {
    Serial.println("EYOOOOOOOOOO");
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i = 0; i < pixy.ccc.numBlocks; i++) {
      //      Serial.print("  block ");
      //    Serial.print(i);
      //  Serial.print(": ");
      Serial.println(pixy.ccc.blocks[i].m_x);
      esc1.write(100);
      esc2.write(100);
      esc3.write(110);
      esc4.write(100);
      delay(300);
    }
  }

  /* esc1.write(test);
  esc2.write(test);
  esc3.write(test);
  esc4.write(test);
  delay(del);
  */

  esc1.write(NEUTRAL);
  esc2.write(NEUTRAL);
  esc3.write(NEUTRAL);
  esc4.write(NEUTRAL);
  //delay(del);
  
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

// Blink every second
void Update_Blink() {
  if (millis() < toggle_time) return;

  // Update LED
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  toggle_time = millis() + LED_INCREMENT;
}