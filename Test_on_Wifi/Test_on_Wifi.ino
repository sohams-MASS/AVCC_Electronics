#include <WiFi.h>

const char* AP_SSID = "AVCC_Master";
const char* AP_PASS = "12345678";

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, 1, false, 4);

  Serial.println("AP start result:");
  Serial.println(ok ? "SUCCESS" : "FAIL");

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {}
