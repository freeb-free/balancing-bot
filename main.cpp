// =================================================================
// 3RPS Ball Balancer + Bluetooth RC Car + Gyro (Smooth Drive - Max Speed 200 + Serial Data)
// =================================================================

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "InverseKinematics.h"

// ========== [1] 기구부 및 핀 설정 ==========
Machine machine(1.6535, 3.125, 1.9685, 3.5433);
const int XP = 14, XM = 16, YP = 15, YM = 17;

Servo servoA, servoB, servoC;
const int SERVO_PIN_A = 4;
const int SERVO_PIN_B = 3;
const int SERVO_PIN_C = 5;

// ========== [2] 영점 및 캘리브레이션 ==========
const float INIT_ANGLE_A = 98;
const int INIT_ANGLE_B = 90;
const int INIT_ANGLE_C = 87;

const double BASE_HEIGHT = 3.74;
double IK_OFFSET = 0;

const int TARGET_X = 498;
const int TARGET_Y = 515;

// ========== [3] PID 제어 변수 ==========
double Kp = 0.00015;
double Kd = 0.012;
double lastErrorX = 0;
double lastErrorY = 0;
float smoothA = INIT_ANGLE_A;
float smoothB = INIT_ANGLE_B;
float smoothC = INIT_ANGLE_C;
const float FILTER_VAL = 0.12;

// ========== [4] RC카 모터 핀 설정 ==========
const int ENA_PIN = 2;
const int IN1_PIN = 6;
const int IN2_PIN = 7;
const int IN3_PIN = 8;
const int IN4_PIN = 9;
const int ENB_PIN = 10;

// 🌟 스무스 드라이빙 제어 변수
float targetSpeedL = 0;  // 왼쪽 모터 목표 속도
float targetSpeedR = 0;  // 오른쪽 모터 목표 속도
float currentSpeedL = 0; // 왼쪽 모터 현재 속도
float currentSpeedR = 0; // 오른쪽 모터 현재 속도
const float ACCEL_STEP = 15.0; // 가감속 부드러움 정도 (작을수록 미끄러지듯, 클수록 빠릿하게)

// ========== [5] 자이로 센서 (MPU6050) ==========
const int MPU_ADDR = 0x68;
float gyroPitch = 0;
float gyroRoll = 0;
unsigned long gyroLastTime = 0;

// 멀티태스킹 타이머
unsigned long lastLoopTime = 0;

// 함수 선언
void setupMPU();
void updateGyro();
void pidBalance(int rawX, int rawY);
int readTouchX();
int readTouchY();
void updateSmoothMotors();

void setup()
{
    Serial.begin(115200);
    Serial1.begin(9600); 

    delay(1000); 

    Serial.println("=========================================");
    Serial.println("[System] Booting up...");
    
    setupMPU();

    IK_OFFSET = machine.theta(0, BASE_HEIGHT, 0, 0);

    servoA.attach(SERVO_PIN_A);
    servoB.attach(SERVO_PIN_B);
    servoC.attach(SERVO_PIN_C);

    servoA.write(INIT_ANGLE_A);
    servoB.write(INIT_ANGLE_B);
    servoC.write(INIT_ANGLE_C);

    pinMode(ENA_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(ENB_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    Serial.println("=========================================");
    Serial.println("[System] Smooth Drive (Max 200) + Serial Data Ready!");
    Serial.println("=========================================");

    lastLoopTime = millis();
    gyroLastTime = millis();
}

void loop()
{
    // 📱 [태스크 1] 블루투스 명령 수신
    if (Serial1.available()) 
    {
        char data = Serial1.read();
        
        if (data == 'F')      { targetSpeedL = 180; targetSpeedR = 180; }
        else if (data == 'B') { targetSpeedL = -180; targetSpeedR = -180; }
        else if (data == 'L') { targetSpeedL = -180; targetSpeedR = 180; }
        else if (data == 'R') { targetSpeedL = 180; targetSpeedR = -180; }
        else if (data == 'S') { targetSpeedL = 0;   targetSpeedR = 0; }
    }

    // 🤖 [태스크 2] 메인 루프 (20ms 간격)
    if (millis() - lastLoopTime >= 20)
    {
        lastLoopTime = millis();
        updateGyro();

        updateSmoothMotors();

        int tx = readTouchX();
        int ty = readTouchY();

        if (tx > 30 && tx < 1000 && ty > 30 && ty < 1000)
        {
            pidBalance(tx, ty);

            // 🌟 추가된 부분: 시리얼 모니터 종합 출력 (좌표, 자이로 각도, 서보모터 각도)
            Serial.print("🎯 좌표(X,Y): "); 
            Serial.print(tx); Serial.print(", "); Serial.print(ty);
            Serial.print("  |  📐 자이로(R,P): "); 
            Serial.print(gyroRoll, 2); Serial.print(", "); Serial.print(gyroPitch, 2);
            Serial.print("  |  ⚙️ 모터각도(A,B,C): ");
            Serial.print(smoothA, 1); Serial.print(", ");
            Serial.print(smoothB, 1); Serial.print(", ");
            Serial.println(smoothC, 1); // 여기서 줄바꿈
        }
        else
        {
            smoothA = (smoothA * 0.90) + (INIT_ANGLE_A * 0.10);
            smoothB = (smoothB * 0.90) + (INIT_ANGLE_B * 0.10);
            smoothC = (smoothC * 0.90) + (INIT_ANGLE_C * 0.10);

            servoA.write(smoothA);
            servoB.write(smoothB);
            servoC.write(smoothC);

            lastErrorX = 0;
            lastErrorY = 0;
        }
    }
}

// =================================================================
// 🌟 멈춤/출발 방지 스무스 모터 업데이트 함수
// =================================================================
void updateSmoothMotors() {
    // 1. 왼쪽 모터 부드러운 가감속
    if (currentSpeedL < targetSpeedL) {
        currentSpeedL += ACCEL_STEP;
        if (currentSpeedL > targetSpeedL) currentSpeedL = targetSpeedL;
    } else if (currentSpeedL > targetSpeedL) {
        currentSpeedL -= ACCEL_STEP;
        if (currentSpeedL < targetSpeedL) currentSpeedL = targetSpeedL;
    }

    // 2. 오른쪽 모터 부드러운 가감속
    if (currentSpeedR < targetSpeedR) {
        currentSpeedR += ACCEL_STEP;
        if (currentSpeedR > targetSpeedR) currentSpeedR = targetSpeedR;
    } else if (currentSpeedR > targetSpeedR) {
        currentSpeedR -= ACCEL_STEP;
        if (currentSpeedR < targetSpeedR) currentSpeedR = targetSpeedR;
    }

    // 3. 실제 모터 핀에 출력 적용 (왼쪽)
    if (currentSpeedL >= 0) {
        digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
        analogWrite(ENA_PIN, (int)currentSpeedL);
    } else {
        digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, HIGH);
        analogWrite(ENA_PIN, (int)(-currentSpeedL));
    }

    // 4. 실제 모터 핀에 출력 적용 (오른쪽)
    if (currentSpeedR >= 0) {
        digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
        analogWrite(ENB_PIN, (int)currentSpeedR);
    } else {
        digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, HIGH);
        analogWrite(ENB_PIN, (int)(-currentSpeedR));
    }
}

void pidBalance(int rawX, int rawY)
{
    double errorX = rawX - TARGET_X;
    double errorY = rawY - TARGET_Y;

    if (abs(errorX) < 30) errorX = 0;
    if (abs(errorY) < 30) errorY = 0;

    double derivX = errorX - lastErrorX;
    double derivY = errorY - lastErrorY;

    double outputX = (Kp * errorX) + (Kd * derivX);
    double outputY = (Kp * errorY) + (Kd * derivY);

    double compX = outputX - ((gyroRoll + 0.24) * 2.5);
    double compY = outputY - ((gyroPitch + 0.03) * 2.5);

    compX = constrain(compX, -0.15, 0.15);
    compY = constrain(compY, -0.26, 0.352);

    double da = machine.theta(0, BASE_HEIGHT, -compX, -compY);
    double db = machine.theta(1, BASE_HEIGHT, -compX, -compY);
    double dc = machine.theta(2, BASE_HEIGHT, -compX, -compY);

    if (!isnan(da) && !isnan(db) && !isnan(dc))
    {
        float targetA = INIT_ANGLE_A + (da - IK_OFFSET);
        float targetB = INIT_ANGLE_B + (db - IK_OFFSET);
        float targetC = INIT_ANGLE_C + (dc - IK_OFFSET);

        targetA = constrain(targetA, 10, 105);
        targetB = constrain(targetB, 10, 170);
        targetC = constrain(targetC, 10, 170);

        smoothA = (smoothA * (1.0 - FILTER_VAL)) + (targetA * FILTER_VAL);
        smoothB = (smoothB * (1.0 - FILTER_VAL)) + (targetB * FILTER_VAL);
        smoothC = (smoothC * (1.0 - FILTER_VAL)) + (targetC * FILTER_VAL);

        servoA.write(smoothA);
        servoB.write(smoothB);
        servoC.write(smoothC);
    }

    lastErrorX = errorX;
    lastErrorY = errorY;
}

// =================================================================
// 🌟 MPU6050 제어 및 터치 제어 함수
// =================================================================
void setupMPU() {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

void updateGyro() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    int16_t GyX = Wire.read() << 8 | Wire.read();
    int16_t GyY = Wire.read() << 8 | Wire.read();
    int16_t GyZ = Wire.read() << 8 | Wire.read();

    double dt = (millis() - gyroLastTime) / 1000.0;
    gyroLastTime = millis();

    float accelPitch = atan2(-AcX, sqrt((long)AcY * AcY + (long)AcZ * AcZ));
    float accelRoll = atan2(AcY, AcZ);

    float gyroRateX = (GyX / 131.0) * (PI / 180.0);
    float gyroRateY = (GyY / 131.0) * (PI / 180.0);

    gyroPitch = 0.98 * (gyroPitch + gyroRateY * dt) + 0.02 * accelPitch;
    gyroRoll = 0.98 * (gyroRoll + gyroRateX * dt) + 0.02 * accelRoll;
}

int readTouchX() {
    pinMode(XP, OUTPUT); digitalWrite(XP, HIGH);
    pinMode(XM, OUTPUT); digitalWrite(XM, LOW);
    pinMode(YP, INPUT_PULLDOWN); pinMode(YM, INPUT);
    delayMicroseconds(50); return analogRead(YP);
}

int readTouchY() {
    pinMode(YP, OUTPUT); digitalWrite(YP, HIGH);
    pinMode(YM, OUTPUT); digitalWrite(YM, LOW);
    pinMode(XP, INPUT_PULLDOWN); pinMode(XM, INPUT);
    delayMicroseconds(50); return analogRead(XP);
}
