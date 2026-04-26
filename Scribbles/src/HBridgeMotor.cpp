#include "HBridgeMotor.hpp"


HBridgeMotor::HBridgeMotor(int pin1, int pin2, int pinSpeed) {
    this->pin1 = pin1;
    this->pin2 = pin2;
    this->pinSpeed = pinSpeed;

    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pinSpeed, OUTPUT);


    // if (ITimer.attachInterruptInterval(20L, TimerHandler))
    // {
    //     int startMicros = micros();
    //     Serial.print(F("Starting ITimer OK, micros() = ")); Serial.println(startMicros);
    // }
    // else
    //     Serial.println(F("Can't set ITimer. Select another freq. or timer"));

    // ledcAttachChannel(pinSpeed, 30000, 8, 0);
}

void HBridgeMotor::set(float speed, bool direction) {
    if (direction) {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    } else {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
    }

    // ISR_PWM.setPWM(pinSpeed, 20000, speed * 100.0);
    analogWrite(pinSpeed, (int)(speed * 1023));
}

void HBridgeMotor::set(float speed) {
    if (speed >= 0.0f) {
        set(speed, FORWARD);
    } else {
        set(-speed, REVERSE);
    }
}

void HBridgeMotor::stop() {
    analogWrite(pinSpeed, 0);
    // ISR_PWM.setPWM(pinSpeed, 20000, 0);
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
}
