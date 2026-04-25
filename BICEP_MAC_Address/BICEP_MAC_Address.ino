// Prints the ESP32's STA MAC address. Flash this to each slave, then copy
// the printed MAC into the master sketch's mac_elec / mac_press / mac_motor.
//
// Note: WiFi.macAddress() returns 00:00:00:00:00:00 if read before the
// radio mode is set — WIFI_STA must be selected first.

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(200);                  // let USB serial settle so the line isn't lost

  WiFi.mode(WIFI_STA);         // required: MAC reads as zeros otherwise
  delay(50);

  Serial.println();
  Serial.print("ESP32 STA MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
}
