#include <Arduino.h>
#include <Servo.h>

// ===== Servo =====
Servo servoYaw;
Servo servoPitch;

#define SERVO_YAW_PIN   PA0
#define SERVO_PITCH_PIN PA1

// ===== Optical Flow UART =====
#define FLOW_SERIAL Serial1   // USART1 (PA9, PA10)

// ===== PID =====
float Kp = 0.8;
float Ki = 0.02;
float Kd = 0.4;

float errorX = 0, lastErrorX = 0, integralX = 0;
float errorY = 0, lastErrorY = 0, integralY = 0;

int deadzone = 3;

// ===== Lock state =====
unsigned long lastDetectTime = 0;
bool lockMode = false;

// ===== Scan =====
int scanDir = 1;
unsigned long lastScanMove = 0;

void autoScan();

void setup() {

  Serial.begin(115200);
  FLOW_SERIAL.begin(115200);

  servoYaw.attach(SERVO_YAW_PIN);
  servoPitch.attach(SERVO_PITCH_PIN);

  servoYaw.write(90);
  servoPitch.write(90);

  Serial.println("STM32 Target Lock Ready");
}

void loop() {

  if (FLOW_SERIAL.available()) {

    String data = FLOW_SERIAL.readStringUntil('\n');

    int dx = 0, dy = 0;

    if (sscanf(data.c_str(), "DX:%d,DY:%d", &dx, &dy) == 2) {

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
        servoX = constrain(servoX, 80, 100);
        servoYaw.write(servoX);

        // ===== PID Y =====
        errorY = dy;
        integralY += errorY;
        float derivativeY = errorY - lastErrorY;
        float outputY = Kp*errorY + Ki*integralY + Kd*derivativeY;
        lastErrorY = errorY;

        int servoY = 90 + outputY;
        servoY = constrain(servoY, 80, 100);
        servoPitch.write(servoY);
      }
    }
  }

  if (millis() - lastDetectTime > 2000) {
    lockMode = false;
  }

  if (!lockMode) {
    autoScan();
  }
}

void autoScan() {

  if (millis() - lastScanMove > 600) {

    lastScanMove = millis();

    if (scanDir == 1) {
      servoYaw.write(100);
    } else {
      servoYaw.write(80);
    }

    delay(200);
    servoYaw.write(90);

    scanDir *= -1;
  }
}
