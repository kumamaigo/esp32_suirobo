#include <ESP32Servo.h>

// ================== 構成（左側専用） ==================
const int FINS_COUNT = 6;
const int finPins[FINS_COUNT] = { 33, 32, 18, 19, 21, 14 }; // 左側ピン

Servo fins[FINS_COUNT];

const float SERVO_MAX_DEG = 270.0;
const int   PULSE_MIN_US  = 500;
const int   PULSE_MAX_US  = 2400;

// ================== 角度の定義 ==================
// ヒレ角度: 0 = 水平, +90 = 真下, マイナス = 水平より上
const float SERVO_CENTER_DEG  = 135.0; // サーボ可動域の中央
const float FIN_AT_CENTER_DEG = 45.0;  // サーボ中央のときのヒレ角度
                                       // → 上90°〜下180° を使える
const float FIN_DOWN_DEG      = 90.0;  // 真下

// ヒレの機構的な可動限界（胴体などに当たる場合はここを狭める）
const float FIN_ANGLE_MIN = -90.0;     // 水平より上 90°
const float FIN_ANGLE_MAX = 180.0;     // 真下をさらに 90° 越えた位置

// ================== 遊泳パラメータ ==================
float frequency   = 1.0;    // 振動数 [Hz]
float amplitude   = 30.0;   // 振幅 [deg]
float phaseStep   = 60.0;   // 隣のヒレとの位相差 [deg]
float swimAngle   = 90.0;   // 遊泳中心のヒレ角度 (0:水平 〜 90:真下)
float diveOffset  = 0.0;    // 潜行オフセット
float finDir      = 1.0;    // 回転方向

float sideAmpScale = 1.0;   // 旋回用倍率
float sideAmpTrim  = 1.3;   // 左側基本振幅補正（1.3倍）
float sideWaveDir  = 1.0;   // 波の進行方向 (+1:前進, -1:後退)

// ホーン取付けの歯ずれ補正 [deg]（数度程度の微調整用）
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

// ヒレ角度(0=水平, 90=真下) → サーボ角
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
      break;
    case 'x': // 停止
      mode = SWIM_STOPPED;
      holdAll(swimAngle + diveOffset);
      break;
    case 'a': // 左旋回 (左の振幅を減衰)
      sideWaveDir = 1.0; sideAmpScale = 0.4;
      break;
    case 'd': // 右旋回 (左は全開)
      sideWaveDir = 1.0; sideAmpScale = 1.0;
      break;
    case 'c': // 直進に戻す
      sideWaveDir = 1.0; sideAmpScale = 1.0;
      break;
    case 'q': // その場左回り
      sideAmpScale = 1.0; sideWaveDir = -1.0;
      break;
    case 'e': // その場右回り
      sideAmpScale = 1.0; sideWaveDir = 1.0;
      break;
    case 'u': diveOffset = constrain(diveOffset - 5, -30.0, 30.0); break;
    case 'j': diveOffset = constrain(diveOffset + 5, -30.0, 30.0); break;
    case 'r': swimAngle = constrain(swimAngle - 10, 0.0, 90.0); break; // 水平寄りへ
    case 'f': swimAngle = constrain(swimAngle + 10, 0.0, 90.0); break; // 真下寄りへ
    case 'h': // 取付け用: 水平で停止
      mode = SWIM_STOPPED;
      holdAll(0);
      break;
    case 'v': // 確認用: 真下で停止
      mode = SWIM_STOPPED;
      holdAll(FIN_DOWN_DEG);
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

  wavePhase += 2.0 * PI * frequency * dt;
  if (wavePhase > 2.0 * PI) wavePhase -= 2.0 * PI;

  for (int i = 0; i < FINS_COUNT; i++) {
    writeFin(i, toServoAngle(i, swimAngle + calcFinDelta(i)));
  }
}

void setup() {
  Serial.begin(115200);                       // PC用シリアル
  Serial1.begin(115200, SERIAL_8N1, 16, 17);  // 右側ESP32通信用 (RX:16, TX:17)

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  for (int i = 0; i < FINS_COUNT; i++) {
    fins[i].setPeriodHertz(50);
    fins[i].attach(finPins[i], PULSE_MIN_US, PULSE_MAX_US);
    writeFin(i, toServoAngle(i, swimAngle));
    delay(50);
  }
  delay(1000);
  applyCommand('w'); // 初期動作：前進
}

void loop() {
  // PCからのコマンド受信 ＆ 右側ESP32へUART転送
  if (Serial.available()) {
    char c = Serial.read();
    Serial1.write(c); // 右側ESP32へそのまま送信
    applyCommand(c);  // 自身の制御に反映
  }

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate >= 15) {
    lastUpdate = now;
    if (mode == SWIM_NORMAL) updateSwim();
  }
}