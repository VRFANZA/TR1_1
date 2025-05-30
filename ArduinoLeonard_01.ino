#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <math.h>  // floor()を使うため

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // 小数点以下を切り捨てて整数化
  int heading = (int)floor(euler.x());
  int pitch   = (int)floor(euler.y());
  int roll    = (int)floor(euler.z());

  int accelX = (int)floor(accel.x());
  int accelY = (int)floor(accel.y());
  int accelZ = (int)floor(accel.z());

  int gyroX = (int)floor(gyro.x());
  int gyroY = (int)floor(gyro.y());
  int gyroZ = (int)floor(gyro.z());

  // ✅ 時刻を先頭に追加（ミリ秒→秒単位にしてる）
  Serial.print(millis() / 1000.0); Serial.print(",");

  // CSV形式で出力：Heading,Pitch,Roll,AccelX,Y,Z,GyroX,Y,Z
  Serial.print(pitch);   Serial.print(",");
  Serial.print(heading); Serial.print(",");
  Serial.print(roll);    Serial.print(",");
  Serial.print(accelX);  Serial.print(",");
  Serial.print(accelY);  Serial.print(",");
  Serial.print(accelZ);  Serial.print(",");
  Serial.print(gyroX);   Serial.print(",");
  Serial.print(gyroY);   Serial.print(",");
  Serial.println(gyroZ);  // 最後は改行

  delay(50); // 約20Hzで送信
}
