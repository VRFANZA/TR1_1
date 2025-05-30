#include <bluefruit.h>
#include "Adafruit_TinyUSB.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <math.h>  // floor()を使うため
#define VBATPIN A6

BLEHidAdafruit blehid;
BLEUart bleuart;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define GYRO_THRESHOLD 10.0
#define PITCH_CHANGE_MIN 10.0
#define ROLL_MAX 15.0
#define HEADING_MAX 15.0

float prev_pitch = 0;
float prev_roll = 0;
float prev_heading = 0;
float last_pitch_change = 0;
bool down_detected = false;

float getBatteryVoltage() {
  int raw = analogRead(VBATPIN);
  float voltage = raw * (3.6f / 1024.0f) * 2.0f;
  return voltage;
}

int getBatteryLevelPercent(float voltage) {
  float percent = (voltage - 3.2f) / (4.2f - 3.2f) * 100.0f;
  percent = constrain(percent, 0, 100);
  return (int)percent;
}

void setup() {
  Serial.begin(115200);
  Bluefruit.begin();
  while (!Serial);
  bno.begin();
  bno.setExtCrystalUse(true);
  Bluefruit.setName("Feather_Keyboard");

  blehid.begin();
  bleuart.begin();

  Bluefruit.setTxPower(4);
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.start(0);
  Bluefruit.Advertising.restartOnDisconnect(true);

  Serial.println("HID Keyboard Ready");
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float heading = euler.x();
  float pitch   = euler.y();
  float roll    = euler.z();

  float gyroX = gyro.x();
  float gyroY = gyro.y();
  float gyroZ = gyro.z();

  float gyro_mag = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  float pitch_diff = pitch - prev_pitch;
  float roll_diff = abs(roll - prev_roll);
  float heading_diff = abs(heading - prev_heading);

  // 判定ロジック
  if (gyro_mag > GYRO_THRESHOLD) {
    if (!down_detected &&
        pitch_diff < -PITCH_CHANGE_MIN &&
        roll_diff < ROLL_MAX &&
        heading_diff < HEADING_MAX) {
      down_detected = true;
      Serial.println("↓ Downward shake detected");
    }
    else if (down_detected &&
             pitch_diff > PITCH_CHANGE_MIN &&
             roll_diff < ROLL_MAX &&
             heading_diff < HEADING_MAX) {
      Serial.println("↑ Upward return detected → Sending ENTER");
      if (Bluefruit.connected()) {
        blehid.keyPress(0x20);// SPACE
        delay(100);
        blehid.keyRelease();
      }
      down_detected = false;
    }
  }

  // センサーログ出力
  Serial.print(millis() / 1000.0); Serial.print(",");
  Serial.print((int)pitch);   Serial.print(",");
  Serial.print((int)heading); Serial.print(",");
  Serial.print((int)roll);    Serial.print(",");
  Serial.print((int)accel.x());  Serial.print(",");
  Serial.print((int)accel.y());  Serial.print(",");
  Serial.print((int)accel.z());  Serial.print(",");
  Serial.print((int)gyroX);   Serial.print(",");
  Serial.print((int)gyroY);   Serial.println((int)gyroZ);

  float voltage = getBatteryVoltage();
  int batteryPercent = getBatteryLevelPercent(voltage);

  bleuart.print(millis() / 1000.0); bleuart.print(",");
  bleuart.print((int)pitch);   bleuart.print(",");
  bleuart.print((int)heading); bleuart.print(",");
  bleuart.print((int)roll);    bleuart.print(",");
  bleuart.print((int)accel.x());  bleuart.print(",");
  bleuart.print((int)accel.y());  bleuart.print(",");
  bleuart.print((int)accel.z());  bleuart.print(",");
  bleuart.print((int)gyroX);   bleuart.print(",");
  bleuart.println((int)gyroZ);

  Serial.print("Battery: ");
  Serial.print(voltage, 2); Serial.print("V, ");
  Serial.print(batteryPercent); Serial.println("%");

  bleuart.print("Battery: ");
  bleuart.print(voltage, 2); bleuart.print("V, ");
  bleuart.print(batteryPercent); bleuart.println("%");

  prev_pitch = pitch;
  prev_roll = roll;
  prev_heading = heading;

  delay(50);  // 約20Hz
}
