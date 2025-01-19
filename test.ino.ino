#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

// Variables to store GPS data
float lastLat = 0;
float lastLng = 0;

void MPU6050_DATA() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    Serial.println("---------------------");
    Serial.print("Accel X: ");
    Serial.print(a.acceleration.x);
    Serial.print(" m/s^2 Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(" m/s^2 Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");
    
    Serial.print("Gyro X: ");
    Serial.print(g.gyro.x);
    Serial.print(" rad/s Y: ");
    Serial.print(g.gyro.y);
    Serial.print(" rad/s Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");
    
    // Print last known GPS position with every MPU reading
    Serial.print("Last Known GPS - Lat: ");
    Serial.print(lastLat, 6);
    Serial.print(", Lng: ");
    Serial.println(lastLng, 6);
}

void GPS_DATA() {
    while (GPS_Serial.available() > 0) {
        if (gps.encode(GPS_Serial.read())) {
            if (gps.location.isUpdated()) {
                // Update the last known position
                lastLat = gps.location.lat();
                lastLng = gps.location.lng();
                
                Serial.println("**** New GPS Data ****");
                Serial.print("New Latitude: ");
                Serial.print(lastLat, 6);
                Serial.print(", Longitude: ");
                Serial.println(lastLng, 6);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
    Serial.println("GPS initialized!");
    
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");
}

void loop() {
    MPU6050_DATA();
    GPS_DATA();
    delay(50);  // 50ms delay gives us roughly 20Hz update rate
}
// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <TinyGPS++.h>
// 
// Adafruit_MPU6050 mpu; 
// TinyGPSPlus gps;
// 
// HardwareSerial GPS_Serial(2);
// 
// void MPU6050_DATA(){
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);
// 
  // Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2 ");
  // Serial.print("Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s^2 ");
  // Serial.print("Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s^2");
// 
  // Serial.print("Gyro X: "); Serial.print(g.gyro.x); Serial.print(" rad/s ");
  // Serial.print("Y: "); Serial.print(g.gyro.y); Serial.print(" rad/s ");
  // Serial.print("Z: "); Serial.print(g.gyro.z); Serial.println(" rad/s");
// }
// 
// void GPS_DATA(){
// while (GPS_Serial.available() > 0) {
    // if (gps.encode(GPS_Serial.read())) {
      // if (gps.location.isUpdated()) {
        // Serial.print("Latitude: "); Serial.print(gps.location.lat(), 6);
        // Serial.print(", Longitude: "); Serial.println(gps.location.lng(), 6);
      // }
    // }
  // }
// 
// }
// void setup() {
  // Serial.begin(115200);
// 
  //  GPS_Serial.begin(9600, SERIAL_8N1, 16,17);
  // Serial.println("GPS initialized!");
// 
    // if (!mpu.begin()) {
    // Serial.println("Failed to find MPU6050 chip");
    // while (1) {
      // delay(10);
    // }
  // }
  // Serial.println("MPU6050 Found!");
// }
// 
// void loop() {
//  MPU6050_DATA();
//  GPS_DATA();  
  // delay(50); 
// }
// 
// 
// 
// #include <HardwareSerial.h>

// // Define the SIM800L connection
// #define SIM800L_TX 17 // ESP32 TX pin connected to SIM800L RX
// #define SIM800L_RX 16 // ESP32 RX pin connected to SIM800L TX

// HardwareSerial SIM800L(2); // Use Hardware Serial2

// void setup() {
//   // Start the serial monitor
//   Serial.begin(115200); // Debugging via Serial Monitor
//   delay(1000);

//   // Start SIM800L communication
//   SIM800L.begin(9600, SERIAL_8N1, SIM800L_RX, SIM800L_TX); // Baud rate 9600
//   delay(1000);

//   // Test AT Command
//   Serial.println("Sending AT command...");
//   SIM800L.println("AT"); // Basic AT test command
// }

// void loop() {
//   // Read responses from SIM800L
//   if (SIM800L.available()) {
//     String response = SIM800L.readString();
//     Serial.println("SIM800L Response: " + response);
//   }

//   // Send commands from Serial Monitor to SIM800L
//   if (Serial.available()) {
//     String command = Serial.readString();
//     SIM800L.println(command);
//   }
// }

// void setup(){

// }
// void loop(){

// }

// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>

// Adafruit_MPU6050 mpu;

// void setup() {
//   Serial.begin(115200);
//   while (!Serial)
//     delay(10); 

//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("MPU6050 Found!");

//   mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
//   mpu.setGyroRange(MPU6050_RANGE_250_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//   delay(100);
// }

// void loop() {
//   sensors_event_t accel, gyro, temp;
//   mpu.getEvent(&accel, &gyro, &temp);

//   Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2 ");
//   Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s^2 ");
//   Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");

//   Serial.print("Gyro X: "); Serial.print(gyro.gyro.x); Serial.print(" rad/s ");
//   Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" rad/s ");
//   Serial.print("Z: "); Serial.print(gyro.gyro.z); Serial.println(" rad/s");

//   delay(50);
// }




// #include <TinyGPS++.h>
// #include <HardwareSerial.h>


// TinyGPSPlus gps;
// HardwareSerial GPS_Serial(2); 

// void setup() {
//   Serial.begin(115200);          
//   GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); 
//   Serial.println("GPS Module Test");
// }

// void loop() {
//   while (GPS_Serial.available() > 0) {
//     char c = GPS_Serial.read();

//     if (gps.encode(c)) {
//       if (gps.location.isUpdated()) {
//         Serial.print("Latitude: ");
//         Serial.println(gps.location.lat(), 6);
//         Serial.print("Longitude: ");
//         Serial.println(gps.location.lng(), 6);
//       }
//     }
//   }
// }


// void setup(){
//   pinMode(2,OUTPUT);
// }

// void loop(){
//   digitalWrite(2,HIGH);
//   delay(1000);
//     digitalWrite(2,LOW);
//   delay(1000);
// }

//  void setup() {
     // initialize digital pin LED_BUILTIN as an output.
    //  pinMode(LED_BUILTIN, OUTPUT);
//  }
 
//  void loop() {
    //  digitalWrite(LED_BUILTIN, HIGH);   
    //  delay(1000);                       
    //  digitalWrite(LED_BUILTIN, LOW);    
    // delay(1000);                       
// }
// void setup(){}
// void loop(){}