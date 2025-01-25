#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
  float ax{},ay{},az{};
  float gx{},gy{},gz{};
  float gyroXOffset{} ,  gyroYOffset{}, gyroZOffset{};
  

void setup() {

  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
     
  }
  Serial.println("MPU6050 Found!");
  calibrateGyro();

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  // Print column headers for Serial Plotter
  // Serial.println(" Gyro_X Gyro_Y Gyro_Z");
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // ax = accel.acceleration.x;
  // ay = accel.acceleration.y;
  // az = accel.acceleration.z;

  gx = gyro.gyro.x-gyroXOffset;
  gy = gyro.gyro.y-gyroYOffset;
  gz = gyro.gyro.z-gyroZOffset;

// Serial.print("Accel_X Accel_Y Accel_Z")
 
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.print(gz); 
  Serial.print("\n");

  delay(20);
}
void calibrateGyro() {
  
 
  for (int i = 0; i < 1000; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    gyroXOffset += g.gyro.x;
    gyroYOffset += g.gyro.y;
    gyroZOffset += g.gyro.z;
  }
  
  gyroXOffset /= 1000;
  gyroYOffset /= 1000;
  gyroZOffset /= 1000;

  Serial.println("the gyro offset value are as follows:");
  Serial.println(gyroXOffset);
  Serial.println(gyroYOffset);
  Serial.println(gyroZOffset);
  Serial.println("the gryo offset value is taken here ");
  Serial.println("------------------------------------------------------------");
}