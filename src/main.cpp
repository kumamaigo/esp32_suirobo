#include <ESP32Servo.h>

// ================== 構成 ==================
const int FINS_PER_SIDE = 6;
enum Side { LEFT = 0, RIGHT = 1 };
const int NUM_SIDES = 2;

// ピン配置（左 / 右）
const int finPins[NUM_SIDES][FINS_PER_SIDE] = {
  { 4,  5, 25, 26, 27, 14 },   // 左
  { 22, 21, 19, 18, 33, 32 }   // 右
};

Servo fins[NUM_SIDES][FINS_PER_SIDE];

// サーボの可動範囲（270°サーボ設定）
const float SERVO_MAX_DEG = 270.0;
const int   PULSE_MIN_US  = 500;
const int   PULSE_MAX_US  = 2400;

// ================== 遊泳パラメータ ==================
float frequency  = 1.0;    // 振動数 [Hz]
float amplitude  = 30.0;   // 振幅 [deg]
float phaseStep  = 60.0;   // 隣のヒレとの位相差 [deg]
float baseAngle  = 135.0;  // 基準角度
float diveOffset = 0.0;    // 潜行オフセット

// 左右の回転方向
float sideDir[NUM_SIDES] = { 1.0, 1.0 };

// 左右ごとの振幅倍率（旋回用）: 0.0〜1.0
float sideAmpScale[NUM_SIDES] = { 1.0, 1.0 };

// 左右ごとの基本振幅補正（左の動きを大きくするために1.3倍に設定）
float sideAmpTrim[NUM_SIDES] = { 1.3, 1.0 };

// 左右ごとの波の進行方向: +1 = 前進, -1 = 後退
float sideWaveDir[NUM_SIDES] = { 1.0, 1.0 };

// 左右の位相ずれ [deg]
float sidePhaseShift[NUM_SIDES] = { 0.0, 0.0 };

// ヒレごとの取り付け誤差補正 [deg]（水平フラット化用）
float finOffset[NUM_SIDES][FINS_PER_SIDE] = {
  { 90, 90, 90, 90, 90, 85 },   // 左
  { 90, 90, 90, 90, 90, 90 }    // 右
};

enum SwimMode { SWIM_NORMAL, SWIM_STOPPED };
SwimMode mode = SWIM_STOPPED;

// 経過時間の管理用
float wavePhase = 0.0;  // [rad]
unsigned long lastPhaseMs = 0;

// ================== 低レベル出力 ==================
void writeFin(int side, int i, float angle) {
  angle = constrain(angle, 0.0, SERVO_MAX_DEG);
  int us = PULSE_MIN_US + (int)(angle / SERVO_MAX_DEG * (PULSE_MAX_US - PULSE_MIN_US));
  fins[side][i].writeMicroseconds(us);
}

float toServoAngle(int side, int i, float delta) {
  return baseAngle + finOffset[side][i] + sideDir[side] * delta;
}

// ================== 操作用関数 ==================
void setDiveOffset(float deg) {
  diveOffset = constrain(deg, -30.0, 30.0);
}

void setTurn(float turn) {
  turn = constrain(turn, -1.0, 1.0);
  sideWaveDir[LEFT]  = 1.0;
  sideWaveDir[RIGHT] = 1.0;
  if (turn >= 0) {
    sideAmpScale[LEFT]  = 1.0;
    sideAmpScale[RIGHT] = 1.0 - turn;
  } else {
    sideAmpScale[LEFT]  = 1.0 + turn;
    sideAmpScale[RIGHT] = 1.0;
  }
}

void setSpin(int dir) {
  sideAmpScale[LEFT]  = 1.0;
  sideAmpScale[RIGHT] = 1.0;
  sideWaveDir[LEFT]   = (dir > 0) ?  1.0 : -1.0;
  sideWaveDir[RIGHT]  = (dir > 0) ? -1.0 :  1.0;
}

void startSwim() {
  lastPhaseMs = millis();
  mode = SWIM_NORMAL;
}

void stopSwim() {
  mode = SWIM_STOPPED;
  for (int s = 0; s < NUM_SIDES; s++) {
    for (int i = 0; i < FINS_PER_SIDE; i++) {
      writeFin(s, i, toServoAngle(s, i, diveOffset));
    }
  }
}

// ================== 波の計算 ==================
float calcFinDelta(int side, int i) {
  float phase = wavePhase
              + sideWaveDir[side] * radians(phaseStep * i)
              + radians(sidePhaseShift[side]);
  
  // sideAmpTrim を掛け合わせて左右個別の振幅調整を実施
  float amp = amplitude * sideAmpScale[side] * sideAmpTrim[side];
  
  return diveOffset + amp * sin(phase);
}

void updateSwim() {
  unsigned long now = millis();
  float dt = (now - lastPhaseMs) / 1000.0;
  lastPhaseMs = now;

  wavePhase += 2.0 * PI * frequency * dt;
  if (wavePhase > 2.0 * PI) wavePhase -= 2.0 * PI;

  for (int s = 0; s < NUM_SIDES; s++) {
    for (int i = 0; i < FINS_PER_SIDE; i++) {
      writeFin(s, i, toServoAngle(s, i, calcFinDelta(s, i)));
    }
  }
}

// ================== シリアルで動作確認 ==================
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'w': setTurn(0); startSwim();              break;
    case 'x': stopSwim();                            break;
    case 'a': setTurn(-0.6);                         break;
    case 'd': setTurn( 0.6);                         break;
    case 'c': setTurn(0);                            break;
    case 'q': setSpin(-1);                           break;
    case 'e': setSpin( 1);                           break;
    case 'u': setDiveOffset(diveOffset - 5);         break;
    case 'j': setDiveOffset(diveOffset + 5);         break;
    default: return;
  }
  Serial.printf("cmd=%c  ampL=%.2f ampR=%.2f waveL=%+.0f waveR=%+.0f dive=%.1f\n",
                c, sideAmpScale[LEFT], sideAmpScale[RIGHT],
                sideWaveDir[LEFT], sideWaveDir[RIGHT], diveOffset);
}

// ================== setup / loop ==================
void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int s = 0; s < NUM_SIDES; s++) {
    for (int i = 0; i < FINS_PER_SIDE; i++) {
      fins[s][i].setPeriodHertz(50);
      fins[s][i].attach(finPins[s][i], PULSE_MIN_US, PULSE_MAX_US);
      writeFin(s, i, toServoAngle(s, i, 0));
      delay(50);
    }
  }

  delay(1000);
  startSwim();
}

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 15; // ms

void loop() {
  handleSerial();

  unsigned long now = millis();
  if (now - lastUpdate >= updateInterval) {
    lastUpdate = now;
    if (mode == SWIM_NORMAL) {
      updateSwim();
    }
  }
}