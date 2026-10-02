#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

// ================== Wi-Fi設定 (Access Point) ==================
const char* ssid     = "FishRobot-AP"; // スマホから接続するWi-Fi名
const char* password = "password123";  // Wi-Fiパスワード (8文字以上)

WebServer server(80);

// ================== 構成（左側専用） ==================
const int FINS_COUNT = 6;
const int finPins[FINS_COUNT] = { 33, 32, 18, 19, 21, 14 }; // 左側ピン

Servo fins[FINS_COUNT];

const float SERVO_MAX_DEG = 270.0;
const int   PULSE_MIN_US  = 500;
const int   PULSE_MAX_US  = 2400;

// ================== 角度の定義 ==================
const float SERVO_CENTER_DEG  = 135.0;
const float FIN_AT_CENTER_DEG = 45.0;
const float FIN_DOWN_DEG      = 90.0;

const float FIN_ANGLE_MIN = -90.0;
const float FIN_ANGLE_MAX = 180.0;

// ================== 遊泳パラメータ ==================
float frequency   = 1.0;
float amplitude   = 30.0;
float phaseStep   = 60.0;
float swimAngle   = 90.0;
float diveOffset  = 0.0;
float finDir      = 1.0;

float sideAmpScale = 1.0;
float sideAmpTrim  = 1.3;
float sideWaveDir  = 1.0;

float finTrim[FINS_COUNT] = { 0, 0, 0, 0, 0, 5 };

enum SwimMode { SWIM_NORMAL, SWIM_STOPPED };
SwimMode mode = SWIM_STOPPED;

String currentStatus = "停止中";

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
      currentStatus = "前進";
      break;
    case 's': // 後進
      sideWaveDir = -1.0; sideAmpScale = 1.0;
      lastPhaseMs = millis(); mode = SWIM_NORMAL;
      currentStatus = "後進";
      break;
    case 'x': // 停止
      mode = SWIM_STOPPED;
      holdAll(swimAngle + diveOffset);
      currentStatus = "停止中";
      break;
    case 'a': // 左前旋回
      sideWaveDir = 1.0; sideAmpScale = 0.4;
      lastPhaseMs = millis(); mode = SWIM_NORMAL;
      currentStatus = "左前旋回";
      break;
    case 'd': // 右前旋回
      sideWaveDir = 1.0; sideAmpScale = 1.0;
      lastPhaseMs = millis(); mode = SWIM_NORMAL;
      currentStatus = "右前旋回";
      break;
    case 'z': // 左後旋回
      sideWaveDir = -1.0; sideAmpScale = 0.4;
      lastPhaseMs = millis(); mode = SWIM_NORMAL;
      currentStatus = "左後旋回";
      break;
    case 'c': // 右後旋回
      sideWaveDir = -1.0; sideAmpScale = 1.0;
      lastPhaseMs = millis(); mode = SWIM_NORMAL;
      currentStatus = "右後旋回";
      break;
    case 'q': // その場左旋回
      sideAmpScale = 1.0; sideWaveDir = -1.0;
      lastPhaseMs = millis(); mode = SWIM_NORMAL;
      currentStatus = "その場左旋回";
      break;
    case 'e': // その場右旋回
      sideAmpScale = 1.0; sideWaveDir = 1.0;
      lastPhaseMs = millis(); mode = SWIM_NORMAL;
      currentStatus = "その場右旋回";
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

// ================== HTML 画面 (Web UI) ==================
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Fish Robot Controller</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      display: flex;
      justify-content: center;
      align-items: center;
      min-height: 100vh;
      margin: 0;
      background-color: #f0f0f0;
      user-select: none;
    }
    .grid-container {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
      width: 90vw;
      max-width: 600px;
      padding: 10px;
    }
    .btn {
      background-color: #dbeafe;
      border: 2px solid #1e3a8a;
      border-radius: 8px;
      padding: 20px 10px;
      font-size: 18px;
      font-weight: bold;
      color: #000;
      display: flex;
      justify-content: center;
      align-items: center;
      cursor: pointer;
      text-align: center;
      box-shadow: 2px 2px 5px rgba(0,0,0,0.2);
    }
    .btn:active {
      background-color: #bfdbfe;
      transform: scale(0.98);
    }
    .btn-stop {
      background-color: #fee2e2;
      border: 2px solid #991b1b;
      color: #991b1b;
    }
    .btn-stop:active {
      background-color: #fca5a5;
    }
    .status-card {
      background-color: #ffedd5;
      border: 2px solid #c2410c;
      border-radius: 8px;
      display: flex;
      justify-content: center;
      align-items: center;
      font-size: 18px;
      font-weight: bold;
      color: #000;
      padding: 15px 5px;
      text-align: center;
    }
    .arrow { margin-left: 6px; font-size: 20px; }
  </style>
</head>
<body>
  <div class="grid-container">
    <div class="btn" onclick="sendCmd('a')">左前旋回 <span class="arrow">↖</span></div>
    <div class="btn" onclick="sendCmd('w')">前進 <span class="arrow">↑</span></div>
    <div class="btn" onclick="sendCmd('d')">右前旋回 <span class="arrow">↗</span></div>

    <div class="btn" onclick="sendCmd('q')">その場左旋回 ↺</div>
    <div class="status-card" id="status">STATUS: 停止中</div>
    <div class="btn" onclick="sendCmd('e')">↻ その場右旋回</div>

    <div class="btn" onclick="sendCmd('z')">左後旋回 <span class="arrow">↙</span></div>
    <div class="btn" onclick="sendCmd('s')">後進 <span class="arrow">↓</span></div>
    <div class="btn" onclick="sendCmd('c')">右後旋回 <span class="arrow">↘</span></div>

    <!-- 後進の下（中央列の4段目）に配置 -->
    <div class="btn btn-stop" style="grid-column: 2;" onclick="sendCmd('x')">停止 ⏹</div>
  </div>

  <script>
    function sendCmd(cmd) {
      fetch('/cmd?val=' + cmd)
        .then(response => response.text())
        .then(txt => {
          document.getElementById('status').innerText = 'STATUS: ' + txt;
        });
    }
  </script>
</body>
</html>
)rawliteral";

// ================== HTTPハンドラ ==================
void handleRoot() {
  server.send(200, "text/html", HTML_PAGE);
}

void handleCmd() {
  if (server.hasArg("val")) {
    char c = server.arg("val").charAt(0);
    Serial1.write(c); // 右側ESP32へ転送
    applyCommand(c);  // 自身の制御に反映
    server.send(200, "text/plain", currentStatus);
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17); // 右側ESP32通信用 (RX:16, TX:17)

  // SoftAP（親機）モードの起動
  WiFi.softAP(ssid, password);
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Webサーバー設定
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  for (int i = 0; i < FINS_COUNT; i++) {
    fins[i].setPeriodHertz(50);
    fins[i].attach(finPins[i], PULSE_MIN_US, PULSE_MAX_US);
    writeFin(i, toServoAngle(i, swimAngle));
    delay(50);
  }
  delay(1000);
}

void loop() {
  server.handleClient(); // Webリクエスト処理

  // PCシリアルからのデバッグ入力対応
  if (Serial.available()) {
    char c = Serial.read();
    Serial1.write(c);
    applyCommand(c);
  }

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate >= 15) {
    lastUpdate = now;
    if (mode == SWIM_NORMAL) updateSwim();
  }
}