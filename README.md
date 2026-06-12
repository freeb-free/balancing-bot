# 🎯 Mobile Active Attitude Control System (Ball Balancer on a Moving Platform)

**Teensy 4.1을 활용한 이동형 능동 자세 제어 시스템**
*Real-Time Ball Position Stabilization Using PID Control on a Driving RC Platform*

한경국립대학교 전자공학과 졸업작품 · 2026
우성훈 · 장경호 · 염정선

<p align="center">
  <img src="assets/hero_robot.png" width="48%" alt="완성된 이동형 Ball Balancer (측면)">
  <img src="assets/robot_top.png" width="48%" alt="완성된 이동형 Ball Balancer (상단)">
</p>

---

## 한 줄 소개

고정된 받침대가 아니라 **주행하는 RC카 위에서** 평판 위의 쇠구슬을 중심에 유지하는 Ball & Plate 시스템입니다. 터치패널로 공의 위치를, MPU6050으로 차체의 흔들림을 동시에 읽어, **이중 루프(공 위치 + 차체 자세) PD 제어**로 이동 중 외란까지 보정합니다.

---

##  Credits & References — 이 프로젝트가 배운 곳

이 프로젝트는 처음부터 끝까지 혼자 떠올린 것이 아니라, 먼저 길을 닦아 둔 분들의 작업을 **많이 참고하고 배우면서** 만들었습니다. 특히 아래 두 자료에 큰 빚을 지고 있습니다.

> ###  [B. Brown — "Ball Balancer: 8 Steps (with Pictures)" (Instructables)](https://www.instructables.com/Ball-Balancer/)
> **이 프로젝트의 출발점이자 가장 많이 인용하고 배운 자료입니다.** Ball & Plate 시스템의 전체 개념, 터치패널로 공 위치를 읽고 3개의 서보로 평판을 기울여 균형을 잡는 기본 아이디어, 그리고 프로젝트를 어떻게 단계적으로 풀어가는지를 이 글에서 익혔습니다. 본 작품은 여기서 배운 구조를 **"이동하는 플랫폼 위"** 라는 새로운 조건으로 확장한 것입니다.

> ###  [Aaed Musa — Ball-Balancer-V2 (GitHub)](https://github.com/aaedmusa/Ball-Balancer-V2)
> 3RPS 병렬 매니퓰레이터의 **역기구학(`Machine` 클래스 / `theta()` 함수)** 구현을 참고하였습니다. 목표 기울기(법선 벡터 + 높이)를 각 서보 각도로 변환하는 수식과 코드 구조를 이 저장소에서 학습해 본 시스템에 맞게 적용했습니다.

위 두 자료가 없었다면 이 작품은 시작조차 어려웠을 것입니다. 원작자분들께 깊이 감사드립니다. 🙇

---

## 왜 만들었나 (Motivation)

제약·화학·반도체 등 정밀 산업에서는 운반 중 적재물의 미세한 흔들림이 품질 저하나 유출 사고로 직결됩니다. 특히 액체처럼 자유 표면(free surface)이 출렁이는 적재물은 가속·감속·회전 시 **슬로싱(sloshing)** 이 발생해 제어가 까다롭습니다.

그래서 점성 감쇠가 거의 없어 **가장 제어하기 어려운 자유 표면 물체의 극단적 모델로 "평판 위의 공"** 을 선정했습니다. 미세한 기울기에도 즉시 굴러가는 한계 안정(marginally stable) 시스템인 공을, 이동 환경에서도 중심에 유지할 수 있다면 동일한 보정 원리를 다양한 정밀 운반 문제로 확장할 수 있다고 보았습니다.

<p align="center">
  <img src="assets/concept.png" width="45%" alt="핵심 개념: 일반 플랫폼 vs 능동 자세제어 플랫폼">
</p>

---

## 🎬 데모

`assets/demo.mp4` 에 동작 영상이 포함되어 있습니다. (방지턱 통과·외란 복원 등)

---

## 🛠️ 주요 부품 (Hardware)

| 부품 | 역할 |
|------|------|
| **Teensy 4.1** (ARM Cortex-M7, 600MHz) | 메인 제어기 — 위치 검출, PID 연산, 역기구학 계산 |
| **MG996R 서보모터 ×3** | 3RPS 플랫폼 기울기 제어 |
| **4-Wire Resistive Touch Panel (8.4")** | 쇠구슬 위치(X, Y) 실시간 검출 |
| **MPU6050 (6축 IMU)** | 주행 중 차체 기울기·진동(Roll/Pitch) 측정 |
| **L298N 모터 드라이버** | 이동 플랫폼 DC 모터 구동 |
| **HC-06 블루투스 모듈** | 스마트폰 주행 명령(F/B/L/R/S) 수신 |

---

## 동작 원리 (How It Works)

### 시스템 흐름

<p align="center">
  <img src="assets/system_flowchart.png" width="60%" alt="전체 시스템 동작 흐름도">
</p>

시스템은 **이중 루프 구조**로 동작합니다.

- **외측 루프 (공 위치 제어)** — 터치패널이 읽은 공 좌표와 목표 위치의 오차를 PD 제어해 플랫폼의 목표 기울기를 산출
- **내측 루프 (차체 자세 보정)** — MPU6050이 측정한 차체 기울기를 보상값으로 더해, 주행 중 발생하는 외란을 상쇄

이렇게 결정된 최종 기울기를 **3RPS 역기구학**으로 세 서보의 회전 각도로 변환합니다.

### PID(PD) 제어

빠른 복원과 진동 억제를 위해 **적분 제어(I)는 제외하고 P + D 제어**를 사용했습니다. (빠른 응답이 필요한 시스템에서 적분 누적은 오히려 진동을 유발) 반복 튜닝 결과 **`Kp = 0.0003`, `Kd = 0.023`** 을 최종 게인으로 선정했습니다.

<p align="center">
  <img src="assets/pid_concept.png" width="75%" alt="P / I / D 제어 원리">
</p>

```cpp
double errorX = rawX - TARGET_X;
double errorY = rawY - TARGET_Y;

double outputX = (Kp * errorX) + (Kd * (errorX - lastErrorX));
double outputY = (Kp * errorY) + (Kd * (errorY - lastErrorY));

// MPU6050 자세 보정 (내측 루프)
double compX = outputX - ((gyroRoll  + 0.24) * 0.4);
double compY = outputY - ((gyroPitch + 0.03) * 0.4);
```

### 3RPS 역기구학

베이스와 상단 플랫폼이 120° 간격의 세 다리로 연결된 병렬 구조입니다. 목표 기울기(법선 벡터 + 높이)를 입력받아, 각 다리의 좌표 → 벡터 길이 → 코사인 법칙으로 서보 각도를 계산합니다.

<p align="center">
  <img src="assets/platform_cad.png" width="48%" alt="3RPS 플랫폼 CAD">
</p>

```cpp
double da = machine.theta(0, BASE_HEIGHT, -compX, -compY);
double db = machine.theta(1, BASE_HEIGHT, -compX, -compY);
double dc = machine.theta(2, BASE_HEIGHT, -compX, -compY);
```

> 역기구학 모델은 [Aaed Musa의 Ball-Balancer-V2](https://github.com/aaedmusa/Ball-Balancer-V2)를 참고했습니다.

### MPU6050 자세 보정 (상보필터)

가속도계(절대 기준이지만 노이즈에 민감)와 자이로(빠르지만 누적 오차 발생)를 **상보필터**로 결합해 안정적인 Roll/Pitch를 얻습니다.

```cpp
gyroPitch = 0.98 * (gyroPitch + gyroRateY * dt) + 0.02 * accelPitch;
gyroRoll  = 0.98 * (gyroRoll  + gyroRateX * dt) + 0.02 * accelRoll;
```

---

## 전체 회로

<p align="center">
  <img src="assets/circuit_diagram.png" width="75%" alt="전체 회로도">
</p>

서보모터는 큰 전류가 필요하므로 마이크로컨트롤러와 **분리된 외부 전원**을 사용하고, 모든 장치는 공통 GND로 묶어 기준 전위를 일치시켰습니다.

---

## 🧪 실험 결과

- **터치패널 위치 검출** — 중심부 약 `(499, 513)`, 위치 변화에 따라 좌표가 안정적·연속적으로 출력
- **역기구학 + 서보 동작** — 목표 기울기에 따라 세 서보가 독립 회전하면서 평판은 하나의 평면 유지
- **PID 게인 튜닝** — `Kp=0.0003, Kd=0.023`에서 빠른 도달 + 최소 오버슈트
- **MPU6050 자세 보정** — 차체를 전·측방으로 기울여도 Pitch/Roll 변화를 정상 검출 및 보상
- **외란 복원** — 손으로 밀어도 공이 다시 중심으로 복귀
- **주행 + 방지턱(약 1cm) 통과** — 순간 충격에도 공이 플랫폼 밖으로 이탈하지 않고 중심 부근 유지

---

## 코드 구조

```
.
├── main.cpp                 # 메인 통합 코드 (위치검출·PID·IK·서보·RC카 주행)
├── InverseKinematics.h      # 3RPS 역기구학 Machine 클래스 선언
├── InverseKinematics.cpp    # 역기구학 theta() 구현
└── assets/                  # 사진·다이어그램·데모 영상
```

핵심 상수:

```cpp
Machine machine(1.6535, 3.125, 1.9685, 3.5433); // d, e, f, g (기구 치수)
const double BASE_HEIGHT = 3.74;
const int TARGET_X = 498, TARGET_Y = 515;       // 목표(중심) 좌표
double Kp = 0.0003, Kd = 0.023;                 // 최종 PD 게인
```

---

## 향후 개선

칼만 필터(Kalman Filter), 상태 공간 제어(State-Space Control), AI 기반 제어 알고리즘을 적용하면 더 향상된 성능을 기대할 수 있습니다. 응용 분야로는 제약·화학 시약 운반, 반도체/디스플레이 정밀 부품 이송, AMR·AGV·서비스 로봇의 적재물 안정화 등이 있습니다.

---

## 참고 문헌

1. **B. Brown**, *Ball Balancer: 8 Steps (with Pictures)*, Instructables, 2021. — https://www.instructables.com/Ball-Balancer/ ⭐
2. **Aaed Musa**, *Ball-Balancer-V2*, GitHub. — https://github.com/aaedmusa/Ball-Balancer-V2 ⭐
3. Paul Stoffregen, *Teensy 4.1 Development Board Technical Specifications*, PJRC, 2025.
4. InvenSense, *MPU-6000 and MPU-6050 Product Specification*, 2013.
5. Tower Pro, *MG996R Servo Motor Specification*, 2024.
6. A. M. Hussien, S. Fathollahi Dehkordi, A. Naeimifard, *"Sloshing Suppression in Liquid Transport Systems Using Fuzzy Logic Control Algorithm,"* Arabian Journal for Science and Engineering, 2025.
7. A. K. Shakya et al., *"Spill-free liquid container handling using deep reinforcement learning agents in feedback control,"* Applied Intelligence, vol. 56, 2026.

---

<p align="center">
  <sub>한경국립대학교 전자공학과 · 우성훈 · 장경호 · 염정선 · 2026</sub>
</p>
