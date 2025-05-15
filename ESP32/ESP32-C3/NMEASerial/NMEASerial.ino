// use UART1 for GPS
// GPS TX → ESP32 RX1 (GPIO 21)
// GPS RX → ESP32 TX1 (GPIO 20)  (only needed if you plan to send config to the GPS)
#define GPS_RX_PIN 20  // connects to GPS module TX pin
#define GPS_TX_PIN 21  // connects to GPS module RX pin (optional)

void setup() {
  // USB serial
  Serial.begin(115200);
  while(!Serial) { delay(10); }  // wait for Serial to be ready

  // UART1 for GPS @ 9600 baud, 8N1
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS reader started");
}

void loop() {
  // read any chars from GPS and write them to USB serial
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
  }
}
