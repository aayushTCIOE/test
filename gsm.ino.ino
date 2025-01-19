
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

void setup(){
 pinMode(2,OUTPUT);
}

void loop(){
  digitalWrite(2,HIGH);
 delay(1000);
    digitalWrite(2,LOW);
 delay(1000);
}