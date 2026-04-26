#include <Arduino.h>
#include "HBridgeMotor.hpp"
#include <ESP32Encoder.h>
#include "PID_FF.hpp"
#include "WheelSpeed.hpp"
#include "ESP32_PWM.h"
#include "PathFollowing.hpp"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
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

HBridgeMotor motor2(23, 22, 21);
HBridgeMotor motor1(14, 13, 4);
HBridgeMotor motor0(26, 25, 27);

WheelSpeed wheelSpeed0;
WheelSpeed wheelSpeed1;
WheelSpeed wheelSpeed2;

PID_FF pid0(0.08929, 0.0188, 0.01, 0.1, 0.001); //  0.02, 0.1, 0.002
PID_FF pid1(0.08105, 0.02559, 0.01, 0.1, 0.001);
PID_FF pid2(0.08084, 0.02601, 0.01, 0.1, 0.001);

LookaheadController lookaheadController;

WheelCommand wheel_speed_cmd;

constexpr float INCH_TO_METER = 0.0254f;
constexpr float whiteboard_width = 45.0f * INCH_TO_METER;
constexpr float whiteboard_height = 29.0f * INCH_TO_METER;

// Latest state parsed from Specs.
struct Pose
{
  bool valid;
  float x;       // 45 inch grid
  float y;       // 29 inch grid
  float heading; // 0-359 degrees (decoded from 7-bit)
};

Pose bot_loc = {false, 0, 0, 0};
Pose current_wp = {false, 0, 0, 0};

// Decode 3-byte message:
//   byte[0]: bit7 = type (0=location, 1=waypoint), bits6..0 = heading_7bit
//   byte[1]: x (0-255)
//   byte[2]: y (0-255)

#define WAYPOINT_MSG_TYPE 1
#define LOCATION_MSG_TYPE 2
int handle_packet(const uint8_t buf[3])
{
  uint8_t msg_type = (buf[0] >> 7) & 0x01;
  uint8_t heading_7bit = buf[0] & 0x7F;
  float heading_deg = (float)(((uint32_t)heading_7bit * 2 * M_PI) / 128.0f);
  uint8_t x = buf[1];
  uint8_t y = buf[2];

  if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF)
  {
    // Special message to reset bot location.
    bot_loc = {false, 0, 0, 0};
    Serial.println("RESET");
    return LOCATION_MSG_TYPE;
  }

  // Convert grid coordinates to meters
  float x_m = (float)x * whiteboard_width / 255.0f;
  float y_m = (float)y * whiteboard_height / 255.0f;

  if (msg_type == 0)
  {
    bot_loc = {true, x_m, y_m, heading_deg};
    Serial.printf("LOC  x=%3f y=%3f h=%3f\n", x_m, y_m, heading_deg);
    return LOCATION_MSG_TYPE;
  }
  else
  {
    current_wp = {true, x_m, y_m, heading_deg};
    Serial.printf("WP   x=%3f y=%3f h=%3f\n", x_m, y_m, heading_deg);
    return WAYPOINT_MSG_TYPE;
  }
  return 0;
}

float targetSpeed = 0.5f;

#undef WHEEL_CHECK
#undef DRIVING_PATH_TEST
#define REMOTE_DRIVING

void setup()
{
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  wheelSpeed2 = WheelSpeed(18, 19);
  wheelSpeed1 = WheelSpeed(35, 34);
  wheelSpeed0 = WheelSpeed(36, 39);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("Setting up WiFi Access Point...");

#ifdef REMOTE_DRIVING
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
#endif
}

float clamp(float val, float min_val, float max_val)
{
  return (val < min_val) ? min_val : (val > max_val) ? max_val
                                                     : val;
}

void driveMotor(HBridgeMotor &motor, float speed_cmd)
{
  if (abs(speed_cmd) < 0.005f)
  {
    motor.stop();
  }
  else
  {
    motor.set(clamp(abs(speed_cmd), 0.0f, 0.15f), speed_cmd >= 0.0f);
  }
}

void loop()
{
#ifdef WHEEL_CHECK
  motor0.set(0.2f, true);
  motor1.set(0.2f, true);
  motor2.set(0.2f, true);

  Serial.printf("Wheel Speeds: %f, %f, %f\n", wheelSpeed0.calculateSpeed(), wheelSpeed1.calculateSpeed(), wheelSpeed2.calculateSpeed());
  delay(1000);
#endif

#ifdef DRIVING_PATH_TEST
  // Test path following by simulating a path and location updates.
  float x_start = 0.0f, y_start = 0.0f;
  float theta = M_PI / 6;
  float x_goal_1 = 0.5 * whiteboard_width, y_goal_1 = 0.0f;
  float x_goal_2 = 0.5 * whiteboard_width, y_goal_2 = 0.5 * whiteboard_height;

  lookaheadController.updatePath(x_start, y_start, x_goal_1, y_goal_1);

  for (float x = x_start; x <= x_goal_1; x += 0.05f)
  {
    wheel_speed_cmd = lookaheadController.command(x, y_start, theta);
    Serial.printf("CMD  u0=%f u1=%f u2=%f\n", wheel_speed_cmd.u0, wheel_speed_cmd.u1, wheel_speed_cmd.u2);
    float wheel0Speed = wheelSpeed0.calculateSpeed();
    float wheel1Speed = wheelSpeed1.calculateSpeed();
    float wheel2Speed = wheelSpeed2.calculateSpeed();
    // Serial.printf("SPEED wheel0=%f wheel1=%f wheel2=%f\n", wheel0Speed, wheel1Speed, wheel2Speed);

    float motor0Output = pid0.update(wheel_speed_cmd.u0, wheel0Speed);
    float motor1Output = pid1.update(wheel_speed_cmd.u1, wheel1Speed);
    float motor2Output = pid2.update(wheel_speed_cmd.u2, wheel2Speed);

    // Serial.printf("Driving motors with outputs: %f, %f, %f\n", motor0Output, motor1Output, motor2Output);

    driveMotor(motor0, motor0Output);
    driveMotor(motor1, motor1Output);
    driveMotor(motor2, motor2Output);
    delay(200);
  }

  lookaheadController.updatePath(x_goal_1, y_goal_1, x_goal_2, y_goal_2);

  for (float y = y_start; y <= y_goal_2; y += 0.05f)
  {
    wheel_speed_cmd = lookaheadController.command(x_goal_2, y, theta);
    Serial.printf("CMD  u0=%f u1=%f u2=%f\n", wheel_speed_cmd.u0, wheel_speed_cmd.u1, wheel_speed_cmd.u2);
    float wheel0Speed = wheelSpeed0.calculateSpeed();
    float wheel1Speed = wheelSpeed1.calculateSpeed();
    float wheel2Speed = wheelSpeed2.calculateSpeed();
    // Serial.printf("SPEED wheel0=%f wheel1=%f wheel2=%f\n", wheel0Speed, wheel1Speed, wheel2Speed);

    float motor0Output = pid0.update(wheel_speed_cmd.u0, wheel0Speed);
    float motor1Output = pid1.update(wheel_speed_cmd.u1, wheel1Speed);
    float motor2Output = pid2.update(wheel_speed_cmd.u2, wheel2Speed);

    // Serial.printf("Driving motors with outputs: %f, %f, %f\n", motor0Output, motor1Output, motor2Output);

    driveMotor(motor0, motor0Output);
    driveMotor(motor1, motor1Output);
    driveMotor(motor2, motor2Output);
    delay(200);
  }

#endif
#ifdef REMOTE_DRIVING
  if (!client || !client.connected())
  {
    client = server.available();
    if (client)
    {
      Serial.println("Client connected");
    }
  }

  if (client && client.connected())
  {
    // Read 3-byte packets as they arrive.
    while (client.available() >= 3)
    {
      uint8_t buf[3];
      size_t n = client.readBytes(buf, 3);
      if (n == 3)
      {
        int msg_type = handle_packet(buf);

        if (msg_type == WAYPOINT_MSG_TYPE && bot_loc.valid)
        {
          // Update lookahead controller with new waypoint and compute command.
          lookaheadController.updatePath(bot_loc.x, bot_loc.y, current_wp.x, current_wp.y);
          wheel_speed_cmd = lookaheadController.command(bot_loc.x, bot_loc.y, bot_loc.heading);
        }
        else if (msg_type == LOCATION_MSG_TYPE)
        {
          if (!bot_loc.valid)
          {
            // If location is not valid, stop the bot and reset.
            wheel_speed_cmd = {0, 0, 0};
            current_wp.valid = false;
          }
          // Update lookahead controller with new location and compute command.
          if (current_wp.valid)
          {
            wheel_speed_cmd = lookaheadController.command(bot_loc.x, bot_loc.y, bot_loc.heading);
            Serial.printf("CMD  u0=%f u1=%f u2=%f\n", wheel_speed_cmd.u0, wheel_speed_cmd.u1, wheel_speed_cmd.u2);
          }
        }
      }
    }
  }

  float wheel0Speed = wheelSpeed0.calculateSpeed();
  float wheel1Speed = wheelSpeed1.calculateSpeed();
  float wheel2Speed = wheelSpeed2.calculateSpeed();
  // Serial.printf("SPEED wheel0=%f wheel1=%f wheel2=%f\n", wheel0Speed, wheel1Speed, wheel2Speed);

  float motor0Output = pid0.update(wheel_speed_cmd.u0, wheel0Speed);
  float motor1Output = pid1.update(wheel_speed_cmd.u1, wheel1Speed);
  float motor2Output = pid2.update(wheel_speed_cmd.u2, wheel2Speed);

  // Serial.printf("Driving motors with outputs: %f, %f, %f\n", motor0Output, motor1Output, motor2Output);

  driveMotor(motor0, motor0Output);
  driveMotor(motor1, motor1Output);
  driveMotor(motor2, motor2Output);

  delay(200);
#endif
}
