#include <Arduino.h>
#include <WiFi.h>

#ifndef AP_SSID
#define AP_SSID "ESP32_Hotspot"
#endif
#ifndef AP_PASSWORD
#define AP_PASSWORD "esp32pass"
#endif

const uint16_t SERVER_PORT = 3333;
WiFiServer server(SERVER_PORT);
WiFiClient client;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Setting up WiFi Access Point...");
  
  // Create WiFi Access Point
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  Serial.println("Access Point created!");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  
  server.begin();
  Serial.printf("Socket server listening on port %u\n", SERVER_PORT);
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();
  }

  if (client && client.connected()) {
    if (client.available() > 0) {
      String line = client.readStringUntil('\n');
      line.trim();
      Serial.print("Received: ");
      Serial.println(line);

      if (line == "ping") {
        client.println("pong");
        client.flush();
        Serial.println("Sent: pong");
      } else {
        client.print("unknown:");
        client.println(line);
      }
    }
  }

  delay(10);
}
