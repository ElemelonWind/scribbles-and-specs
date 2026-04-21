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

// Latest state parsed from Specs.
struct Pose {
  bool valid;
  uint8_t x;          // 0-255 grid
  uint8_t y;          // 0-255 grid
  uint16_t heading;   // 0-359 degrees (decoded from 7-bit)
};

Pose bot_loc = {false, 0, 0, 0};
Pose current_wp = {false, 0, 0, 0};

// Decode 3-byte message:
//   byte[0]: bit7 = type (0=location, 1=waypoint), bits6..0 = heading_7bit
//   byte[1]: x (0-255)
//   byte[2]: y (0-255)
void handle_packet(const uint8_t buf[3]) {
  uint8_t msg_type = (buf[0] >> 7) & 0x01;
  uint8_t heading_7bit = buf[0] & 0x7F;
  uint16_t heading_deg = (uint16_t)(((uint32_t)heading_7bit * 360UL) / 128UL);
  uint8_t x = buf[1];
  uint8_t y = buf[2];

  if (msg_type == 0) {
    bot_loc = {true, x, y, heading_deg};
    Serial.printf("LOC  x=%3u y=%3u h=%3u\n", x, y, heading_deg);
  } else {
    current_wp = {true, x, y, heading_deg};
    Serial.printf("WP   x=%3u y=%3u h=%3u\n", x, y, heading_deg);
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Setting up WiFi Access Point...");

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
    if (client) {
      Serial.println("Client connected");
    }
  }

  if (client && client.connected()) {
    // Read 3-byte packets as they arrive.
    while (client.available() >= 3) {
      uint8_t buf[3];
      size_t n = client.readBytes(buf, 3);
      if (n == 3) {
        handle_packet(buf);
      }
    }
  }

  delay(1);
}
