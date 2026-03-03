#include <Arduino.h>
#include <ESP32Servo.h>

// ===== Servo =====
Servo servoYaw;
Servo servoPitch;

#define SERVO_YAW_PIN    18
#define SERVO_PITCH_PIN  19

// ===== Optical Flow UART =====
HardwareSerial OFSerial(2);
#define RXD2 16
#define TXD2 17

// ===== PID Parameters =====
float Kp = 0.8;
float Ki = 0.02;
float Kd = 0.4;

float errorX = 0, lastErrorX = 0, integralX = 0;
float errorY = 0, lastErrorY = 0, integralY = 0;

int deadzone = 3;

// ===== Lock State =====
unsigned long lastDetectTime = 0;
bool lockMode = false;

// ===== Auto Scan =====
int scanDir = 1;
unsigned long lastScanMove = 0;

void autoScan();

void setup() {

  Serial.begin(115200);

  OFSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  servoYaw.attach(SERVO_YAW_PIN);
  servoPitch.attach(SERVO_PITCH_PIN);

  servoYaw.write(90);
  servoPitch.write(90);

  Serial.println("Target Lock System Ready");
}

void loop() {

  // ===== Read Optical Flow =====
  if (OFSerial.available()) {

    String data = OFSerial.readStringUntil('\n');

    int dx = 0, dy = 0;

    if (sscanf(data.c_str(), "DX:%d,DY:%d", &dx, &dy) == 2) {

      if (abs(dx) > deadzone || abs(dy) > deadzone) {

        lockMode = true;
        lastDetectTime = millis();

        // ===== PID X =====
        errorX = dx;
        integralX += errorX;
        float derivativeX = errorX - lastErrorX;
        float outputX = Kp * errorX + Ki * integralX + Kd * derivativeX;
        lastErrorX = errorX;

        int servoX = 90 + outputX;
        servoX = constrain(servoX, 75, 105);
        servoYaw.write(servoX);

        // ===== PID Y =====
        errorY = dy;
        integralY += errorY;
        float derivativeY = errorY - lastErrorY;
        float outputY = Kp * errorY + Ki * integralY + Kd * derivativeY;
        lastErrorY = errorY;

        int servoY = 90 + outputY;
        servoY = constrain(servoY, 75, 105);
        servoPitch.write(servoY);
      }
    }
  }

  // ===== Nếu mất mục tiêu > 2 giây =====
  if (millis() - lastDetectTime > 2000) {
    lockMode = false;
  }

  // ===== Auto Scan Mode =====
  if (!lockMode) {
    autoScan();
  }
}

// ===== Auto Scan Function =====
void autoScan() {

  if (millis() - lastScanMove > 600) {

    lastScanMove = millis();

    if (scanDir == 1) {
      servoYaw.write(105);   // quay phải
    } else {
      servoYaw.write(75);    // quay trái
    }

    delay(200);
    servoYaw.write(90);      // dừng

    scanDir *= -1;
  }
}
