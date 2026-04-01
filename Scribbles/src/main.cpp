#include <Arduino.h>
#include <WiFi.h>

#ifndef WIFI_SSID
#define WIFI_SSID "SSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "Password"
#endif

const uint16_t SERVER_PORT = 3333;
WiFiServer server(SERVER_PORT);
WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting WiFi connection...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }

  Serial.println();
  Serial.print("Connected to WiFi, IP: ");
  Serial.println(WiFi.localIP());

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
