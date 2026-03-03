#include <ESP32Servo.h>

Servo servoYaw;
Servo servoPitch;

#define SERVO_YAW_PIN 18
#define SERVO_PITCH_PIN 19

HardwareSerial OFSerial(2);

// ===== PID thông số =====
float Kp = 0.8;
float Ki = 0.01;
float Kd = 0.4;

float errorX, lastErrorX = 0, integralX = 0;
float errorY, lastErrorY = 0, integralY = 0;

int deadzone = 3;

unsigned long lastDetectTime = 0;
bool lockMode = false;

int scanDir = 1;

void setup() {
  Serial.begin(115200);
  OFSerial.begin(115200, SERIAL_8N1, 16, 17);

  servoYaw.attach(SERVO_YAW_PIN);
  servoPitch.attach(SERVO_PITCH_PIN);

  servoYaw.write(90);
  servoPitch.write(90);
}

void loop() {

  if (OFSerial.available()) {

    String data = OFSerial.readStringUntil('\n');

    int dx = 0, dy = 0;
    sscanf(data.c_str(), "DX:%d,DY:%d", &dx, &dy);

    if (abs(dx) > deadzone || abs(dy) > deadzone) {
      lockMode = true;
      lastDetectTime = millis();

      // ===== PID X =====
      errorX = dx;
      integralX += errorX;
      float derivativeX = errorX - lastErrorX;
      float outputX = Kp*errorX + Ki*integralX + Kd*derivativeX;
      lastErrorX = errorX;

      int servoX = 90 + outputX;
      servoX = constrain(servoX, 70, 110);
      servoYaw.write(servoX);

      // ===== PID Y =====
      errorY = dy;
      integralY += errorY;
      float derivativeY = errorY - lastErrorY;
      float outputY = Kp*errorY + Ki*integralY + Kd*derivativeY;
      lastErrorY = errorY;

      int servoY = 90 + outputY;
      servoY = constrain(servoY, 70, 110);
      servoPitch.write(servoY);
    }
  }

  // ===== Nếu mất mục tiêu 2 giây → Auto Scan =====
  if (millis() - lastDetectTime > 2000) {
    lockMode = false;
  }

  if (!lockMode) {
    autoScan();
  }
}

// ===== Auto Scan =====
void autoScan() {

  static unsigned long lastMove = 0;

  if (millis() - lastMove > 50) {
    lastMove = millis();

    servoYaw.write(90 + scanDir * 15);

    if (scanDir == 1) {
      delay(400);
      scanDir = -1;
    } else {
      delay(400);
      scanDir = 1;
    }

    servoYaw.write(90);
  }
}
