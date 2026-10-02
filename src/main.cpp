#include <ESP32Servo.h>

// ================== 構成（左側専用） ==================
const int FINS_COUNT = 6;
const int finPins[FINS_COUNT] = { 33, 32, 18, 19, 21, 14 }; // 左側ピン

Servo fins[FINS_COUNT];

const float SERVO_MAX_DEG = 270.0;
const int   PULSE_MIN_US  = 500;
const int   PULSE_MAX_US  = 2400;

// ================== 角度の定義 ==================
const float SERVO_CENTER_DEG  = 135.0; // サーボ可動域の中央
const float FIN_AT_CENTER_DEG = 45.0;  // サーボ中央のときのヒレ角度
const float FIN_DOWN_DEG      = 90.0;  // 真下

const float FIN_ANGLE_MIN = -90.0;     // 水平より上 90°
const float FIN_ANGLE_MAX = 180.0;     // 真下をさらに 90° 越えた位置

// ================== 遊泳パラメータ ==================
float frequency   = 1.0;    // 振動数 [Hz]
float amplitude   = 30.0;   // 振幅 [deg]
float phaseStep   = 60.0;   // 隣のヒレとの位相差 [deg]

// --- 角度遷移用の変数 ---
float swimAngle       = 0.0;    // 現在のヒレ中心角度 (0:水平 〜 90:真下)
float targetSwimAngle = 0.0;    // 目標のヒレ中心角度
float transitionTime  = 6.0;    // 変化にかける時間 [秒]

float diveOffset  = 0.0;    // 潜行オフセット
float finDir      = 1.0;    // 回転方向

float sideAmpScale = 1.0;   // 旋回用倍率
float sideAmpTrim  = 1.3;   // 左側基本振幅補正（1.3倍）
float sideWaveDir  = 1.0;   // 波の進行方向 (+1:前進, -1:後退)

float finTrim[FINS_COUNT] = { 0, 0, 0, 0, 0, 5 };

enum SwimMode { SWIM_NORMAL, SWIM_STOPPED };
SwimMode mode = SWIM_STOPPED;

float wavePhase = 0.0;
unsigned long lastPhaseMs = 0;

// ================== 低レベル出力 ==================
void writeFin(int i, float angle) {
  angle = constrain(angle, 0.0, SERVO_MAX_DEG);
  int us = PULSE_MIN_US + (int)(angle / SERVO_MAX_DEG * (PULSE_MAX_US - PULSE_MIN_US));
  fins[i].writeMicroseconds(us);
}

float toServoAngle(int i, float finAngle) {
  finAngle = constrain(finAngle, FIN_ANGLE_MIN, FIN_ANGLE_MAX);
  return SERVO_CENTER_DEG + finDir * (finAngle - FIN_AT_CENTER_DEG) + finTrim[i];
}

void holdAll(float finAngle) {
  for (int i = 0; i < FINS_COUNT; i++) writeFin(i, toServoAngle(i, finAngle));
}

// ================== 操作・コマンド処理 ==================
void applyCommand(char c) {
  switch (c) {
    case 'w': // 前進
      sideWaveDir = 1.0; sideAmpScale = 1.0;
      lastPhaseMs = millis(); mode = SWIM_NORMAL;
      Serial.println("-> Command: Forward (w)");
      break;
    case 'x': // 停止
      mode = SWIM_STOPPED;
      holdAll(swimAngle + diveOffset);
      Serial.println("-> Command: Stop (x)");
      break;
    case 'a': // 左旋回
      sideWaveDir = 1.0; sideAmpScale = 0.4;
      Serial.println("-> Command: Turn Left (a)");
      break;
    case 'd': // 右旋回
      sideWaveDir = 1.0; sideAmpScale = 1.0;
      Serial.println("-> Command: Turn Right (d)");
      break;
    case 'c': // 直進に戻す
      sideWaveDir = 1.0; sideAmpScale = 1.0;
      Serial.println("-> Command: Straight (c)");
      break;
    case 'q': // その場左回り
      sideAmpScale = 1.0; sideWaveDir = -1.0;
      Serial.println("-> Command: Rotate Left (q)");
      break;
    case 'e': // その場右回り
      sideAmpScale = 1.0; sideWaveDir = 1.0;
      Serial.println("-> Command: Rotate Right (e)");
      break;
    case 'm': // 陸上モード（6秒かけて真下 90° へ）
      targetSwimAngle = 90.0;
      Serial.println("-> Command: Land Mode (90 deg / 6 sec) (m)");
      break;
    case 'n': // 水中モード（6秒かけて水平 0° へ復帰）
      targetSwimAngle = 0.0;
      Serial.println("-> Command: Water Mode (0 deg / 6 sec) (n)");
      break;
    case 'u': 
      diveOffset = constrain(diveOffset - 5, -30.0, 30.0);
      Serial.print("-> Dive Offset: "); Serial.println(diveOffset);
      break;
    case 'j': 
      diveOffset = constrain(diveOffset + 5, -30.0, 30.0);
      Serial.print("-> Dive Offset: "); Serial.println(diveOffset);
      break;
    case 'h': // 取付け用: 水平で停止
      mode = SWIM_STOPPED;
      holdAll(0);
      Serial.println("-> Command: Hold Horizontal (h)");
      break;
    case 'v': // 確認用: 真下で停止
      mode = SWIM_STOPPED;
      holdAll(FIN_DOWN_DEG);
      Serial.println("-> Command: Hold Down (v)");
      break;
  }
}

// ================== 波の計算 ==================
float calcFinDelta(int i) {
  float phase = wavePhase + sideWaveDir * radians(phaseStep * i);
  float amp   = amplitude * sideAmpScale * sideAmpTrim;
  return diveOffset + amp * sin(phase);
}

void updateSwim() {
  unsigned long now = millis();
  float dt = (now - lastPhaseMs) / 1000.0;
  lastPhaseMs = now;

  // 1. 角度の滑らかな遷移処理（6秒間かけて targetSwimAngle へ近づける）
  if (swimAngle != targetSwimAngle) {
    float changeSpeed = 90.0 / transitionTime; // 1秒あたりの変化度数
    if (swimAngle < targetSwimAngle) {
      swimAngle += changeSpeed * dt;
      if (swimAngle > targetSwimAngle) swimAngle = targetSwimAngle;
    } else {
      swimAngle -= changeSpeed * dt;
      if (swimAngle < targetSwimAngle) swimAngle = targetSwimAngle;
    }
  }

  // 2. 波動の位相更新
  wavePhase += 2.0 * PI * frequency * dt;
  if (wavePhase > 2.0 * PI) wavePhase -= 2.0 * PI;

  // 3. 各ヒレへの出力
  for (int i = 0; i < FINS_COUNT; i++) {
    writeFin(i, toServoAngle(i, swimAngle + calcFinDelta(i)));
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  for (int i = 0; i < FINS_COUNT; i++) {
    fins[i].setPeriodHertz(50);
    fins[i].attach(finPins[i], PULSE_MIN_US, PULSE_MAX_US);
    writeFin(i, toServoAngle(i, swimAngle));
    delay(50);
  }
  delay(1000);
  
  Serial.println("=== ESP32 Fin Control System Ready ===");
  applyCommand('w'); // 初期動作：前進
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial1.write(c); // 右側ESP32へ転送
    applyCommand(c);
  }

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate >= 15) {
    lastUpdate = now;
    if (mode == SWIM_NORMAL) updateSwim();
  }
}