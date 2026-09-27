#include <ESP32Servo.h>


const int NUM_FINS = 6;
const int finPins[NUM_FINS] = {4,5,25,26,27,14};

Servo fins[NUM_FINS];


float frequency   = 1.0;   //1秒間に何回往復運動するか（振動数）を表す変数
float amplitude   = 75.0;  //振幅
float phaseStep   = 60.0;  //隣のヒレとの位相差
float baseAngle   = 135.0; //基準角度
float diveOffset  = 0.0;   //潜行オフセット


float finOffset[NUM_FINS] = {0, 0, 0, 0, 0, 5};


enum SwimMode { SWIM_NORMAL, SWIM_STOPPED };
SwimMode mode = SWIM_STOPPED;


void setDiveOffset(float deg) {
  if (deg > 30.0) deg = 30.0;
  if (deg < -30.0) deg = -30.0;
  diveOffset = deg;
}

void startSwim() {
  mode = SWIM_NORMAL;
}

void stopSwim() {
  mode = SWIM_STOPPED;
  for (int i = 0; i < NUM_FINS; i++) {
    fins[i].write(baseAngle + diveOffset);
  }
}


float calcFinAngle(int finIndex, float t_sec, float amp) {
  float phaseRad = radians(phaseStep * finIndex);
  float omega = 2.0 * PI * frequency;
  float s = sin(omega * t_sec + phaseRad);
  return baseAngle + diveOffset + amp * s;
}

void updateSwim() {
  float t_sec = millis() / 1000.0;
  for (int i = 0; i < NUM_FINS; i++) {
    float angle = calcFinAngle(i, t_sec, amplitude);
    angle = constrain(angle, 0, 270);
    fins[i].write(angle);
  }
}

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NUM_FINS; i++) {
    fins[i].setPeriodHertz(50);
    fins[i].attach(finPins[i], 500, 2400);
    fins[i].write(baseAngle);
  }

  delay(1000);
  startSwim();
}

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 15; // ms

void loop() {
  unsigned long now = millis();
  if (now - lastUpdate >= updateInterval) {
    lastUpdate = now;

    switch (mode) {
      case SWIM_NORMAL:
        updateSwim();
        break;
      case SWIM_STOPPED:
        break;
    }
  }
}