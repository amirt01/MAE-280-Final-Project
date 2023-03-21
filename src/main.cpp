#include <Arduino.h>
#include <Servo.h>
#include <Pixy2.h>

#include <Adafruit_LPS35HW.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();
Adafruit_MPU6050 mpu;

Pixy2 pixy;

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int esc1pin = 9;
int esc2pin = 6;
int esc3pin = 5;
int esc4pin = 3;

int pre = 90;
int del = 1000;
int test = 95;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("Adafruit LPS35HW Test");
  if (!lps35hw.begin_I2C()) {
    //if (!lps35hw.begin_SPI(LPS_CS)) {
    //if (!lps35hw.begin_SPI(LPS_CS, LPS_SCK, LPS_MISO, LPS_MOSI)) {
    Serial.println("Couldn't find LPS35HW chip");
    while (1)
      ;
  }

  Serial.println("Found LPS35HW chip");
  Serial.println("");

  pixy.init();

  esc1.attach(esc1pin);
  esc2.attach(esc2pin);
  esc3.attach(esc3pin);
  esc4.attach(esc4pin);

  esc1.write(90);
  esc2.write(90);
  esc3.write(90);
  esc4.write(90);
  delay(3000);
}

void loop() {

  Serial.print("Temperature: ");
  Serial.print(lps35hw.readTemperature());
  Serial.println(" C");

  Serial.print("Pressure: ");
  Serial.print(lps35hw.readPressure());
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

  esc1.write(90);
  esc2.write(90);
  esc3.write(90);
  esc4.write(90);
  //delay(del);
}