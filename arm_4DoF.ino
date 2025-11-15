#include <Servo.h>

// ---------------------- CONFIG ----------------------
const int PIN_SERVO_BASE   = 3;   // myservo1
const int PIN_SERVO_LOWER  = 5;   // myservo2
const int PIN_SERVO_UPPER  = 6;   // myservo3
const int PIN_SERVO_CLAW   = 9;   // myservo4

const int PIN_RIGHT_X = A2;
const int PIN_RIGHT_Y = A5;
const int PIN_RIGHT_Z = 7;  // INPUT_PULLUP

const int PIN_LEFT_X  = A3;
const int PIN_LEFT_Y  = A4;
const int PIN_LEFT_Z  = 8;  // INPUT_PULLUP

// Limiti di sicurezza (gradi)
const int LIM_BASE_MIN  = 1,   LIM_BASE_MAX  = 180;
const int LIM_LOWER_MIN = 25,  LIM_LOWER_MAX = 180;
const int LIM_UPPER_MIN = 1,   LIM_UPPER_MAX = 135;
const int LIM_CLAW_MIN  = 45,  LIM_CLAW_MAX  = 120;

// Postura di reset
const int HOME_BASE  = 90;
const int HOME_LOWER = 90;
const int HOME_UPPER = 90;
const int HOME_CLAW  = 90;

// Dead-zone e filtro
int DEADZONE = 140;        // allargha (160–200) se trema
const float ALPHA = 0.25;  // filtro EMA: 0.1 = più liscio, 0.4 = più reattivo

// Velocità massime (gradi/secondo)
const float MAXSPD_BASE  = 180.0;
const float MAXSPD_LOWER = 120.0;
const float MAXSPD_UPPER = 120.0;
const float MAXSPD_CLAW  = 160.0;

// Inversioni (se un asse “va al contrario”, metti true)
bool INV_RIGHT_X = true;  // base
bool INV_RIGHT_Y = true;   // lower arm (di solito Y destro è "invertito")
bool INV_LEFT_X  = true;  // claw
bool INV_LEFT_Y  = false;  // upper arm

// ---------------------- STATE ----------------------
Servo sBase, sLower, sUpper, sClaw;

// posizioni “float” per integrazione morbida
float pBase  = HOME_BASE;
float pLower = HOME_LOWER;
float pUpper = HOME_UPPER;
float pClaw  = HOME_CLAW;
// posizioni intere inviate ai servo
int   iBase  = HOME_BASE;
int   iLower = HOME_LOWER;
int   iUpper = HOME_UPPER;
int   iClaw  = HOME_CLAW;

// centri calibrati
int C_RIGHT_X = 512, C_RIGHT_Y = 512, C_LEFT_X = 512, C_LEFT_Y = 512;

// valori filtrati
float fRX = 512, fRY = 512, fLX = 512, fLY = 512;

// pulsanti (debounce con edge detect)
int lastZR = HIGH, lastZL = HIGH;
unsigned long tLastZR = 0, tLastZL = 0;
const unsigned long DEBOUNCE_MS = 50;

// stato pinza toggle
bool clawOpen = false;

// tempo
unsigned long tPrev = 0;

// ---------------------- UTILS ----------------------
int readAnalogSmooth(int pin, float &fval) {
  int raw = analogRead(pin);
  fval = ALPHA * raw + (1.0 - ALPHA) * fval;
  return (int)(fval + 0.5f);
}

float axisToSpeed(int value, int center, int deadzone, float maxSpd, bool invert=false) {
  int diff = value - center;
  if (invert) diff = -diff;
  int ad = abs(diff);
  if (ad <= deadzone) return 0.0f;

  // scala (0..1) oltre la deadzone rispetto a ~metà corsa
  float effective = (float)(ad - deadzone);
  float range = 512.0f - deadzone;    // metà A/D (1024/2) meno deadzone
  if (range < 1) range = 1;
  float norm = effective / range;     // 0..~1
  if (norm > 1.0f) norm = 1.0f;

  float spd = norm * maxSpd;          // deg/sec
  return (diff < 0 ? -spd : spd);
}

int writeIfChanged(Servo &sv, float &pf, int &pi, int minA, int maxA) {
  if (pf < minA) pf = minA;
  if (pf > maxA) pf = maxA;
  int newI = (int)lround(pf);
  if (newI != pi) {
    pi = newI;
    sv.write(pi);
  }
  return pi;
}

void resetPosture() {
  pBase  = HOME_BASE;
  pLower = HOME_LOWER;
  pUpper = HOME_UPPER;
  pClaw  = HOME_CLAW;
  writeIfChanged(sBase,  pBase,  iBase,  LIM_BASE_MIN,  LIM_BASE_MAX);
  writeIfChanged(sLower, pLower, iLower, LIM_LOWER_MIN, LIM_LOWER_MAX);
  writeIfChanged(sUpper, pUpper, iUpper, LIM_UPPER_MIN, LIM_UPPER_MAX);
  writeIfChanged(sClaw,  pClaw,  iClaw,  LIM_CLAW_MIN,  LIM_CLAW_MAX);
}

void toggleClaw() {
  if (clawOpen) pClaw = LIM_CLAW_MIN; else pClaw = LIM_CLAW_MAX;
  clawOpen = !clawOpen;
  writeIfChanged(sClaw, pClaw, iClaw, LIM_CLAW_MIN, LIM_CLAW_MAX);
}

void calibrateCenters() {
  // legge per ~200 campioni e fa la media
  const int N = 200;
  long sx=0, sy=0, ux=0, uy=0;
  for (int i=0;i<N;i++) {
    sx += analogRead(PIN_RIGHT_X);
    sy += analogRead(PIN_RIGHT_Y);
    ux += analogRead(PIN_LEFT_X);
    uy += analogRead(PIN_LEFT_Y);
    delay(2);
  }
  C_RIGHT_X = sx / N;
  C_RIGHT_Y = sy / N;
  C_LEFT_X  = ux / N;
  C_LEFT_Y  = uy / N;

  // inizializza i filtri con i centri reali
  fRX = C_RIGHT_X; fRY = C_RIGHT_Y; fLX = C_LEFT_X; fLY = C_LEFT_Y;
}

// ---------------------- SETUP ----------------------
void setup() {
  sBase.attach (PIN_SERVO_BASE);
  sLower.attach(PIN_SERVO_LOWER);
  sUpper.attach(PIN_SERVO_UPPER);
  sClaw.attach (PIN_SERVO_CLAW);

  pinMode(PIN_RIGHT_Z, INPUT_PULLUP);
  pinMode(PIN_LEFT_Z,  INPUT_PULLUP);

  Serial.begin(9600);
  delay(200);

  calibrateCenters();    // ← importantissimo se i joystick non centrano a 512

  resetPosture();

  tPrev = millis();

  // Piccolo messaggio diagnostico
  Serial.print("Centers: RX="); Serial.print(C_RIGHT_X);
  Serial.print(" RY="); Serial.print(C_RIGHT_Y);
  Serial.print(" LX="); Serial.print(C_LEFT_X);
  Serial.print(" LY="); Serial.println(C_LEFT_Y);
}

// ---------------------- LOOP ----------------------
void loop() {
  // tempo (per velocità a gradi/secondo)
  unsigned long now = millis();
  float dt = (now - tPrev) / 1000.0f;
  if (dt < 0) dt = 0;
  if (dt > 0.05f) dt = 0.05f; // limita step (anti salti se ci sono lag)
  tPrev = now;

  // letture filtrate
  int RX = readAnalogSmooth(PIN_RIGHT_X, fRX);
  int RY = readAnalogSmooth(PIN_RIGHT_Y, fRY);
  int LX = readAnalogSmooth(PIN_LEFT_X,  fLX);
  int LY = readAnalogSmooth(PIN_LEFT_Y,  fLY);

  // velocità (deg/s), con inversioni dove serve
  float vBase  = axisToSpeed(RX, C_RIGHT_X, DEADZONE, MAXSPD_BASE,  INV_RIGHT_X);
  float vLower = axisToSpeed(RY, C_RIGHT_Y, DEADZONE, MAXSPD_LOWER, INV_RIGHT_Y);
  float vUpper = axisToSpeed(LY, C_LEFT_Y,  DEADZONE, MAXSPD_UPPER, INV_LEFT_Y);
  float vClaw  = axisToSpeed(LX, C_LEFT_X,  DEADZONE, MAXSPD_CLAW,  INV_LEFT_X);

  // integrazione posizioni
  pBase  += vBase  * dt;
  pLower += vLower * dt;
  pUpper += vUpper * dt;
  pClaw  += vClaw  * dt;

  // scrivi solo se cambiate
  writeIfChanged(sBase,  pBase,  iBase,  LIM_BASE_MIN,  LIM_BASE_MAX);
  writeIfChanged(sLower, pLower, iLower, LIM_LOWER_MIN, LIM_LOWER_MAX);
  writeIfChanged(sUpper, pUpper, iUpper, LIM_UPPER_MIN, LIM_UPPER_MAX);
  writeIfChanged(sClaw,  pClaw,  iClaw,  LIM_CLAW_MIN,  LIM_CLAW_MAX);

  // ----- Pulsanti con debounce ed edge detect -----
  int zR = digitalRead(PIN_RIGHT_Z);
  int zL = digitalRead(PIN_LEFT_Z);

  if (zL != lastZL) {           // variazione
    tLastZL = now;
    lastZL = zL;
  } else if (zL == LOW && (now - tLastZL) > DEBOUNCE_MS) {
    // fronte stabile di pressione (LOW con INPUT_PULLUP)
    toggleClaw();
    tLastZL = now + 1000;       // evita retrigger finché resta premuto
  }

  if (zR != lastZR) {
    tLastZR = now;
    lastZR = zR;
  } else if (zR == LOW && (now - tLastZR) > DEBOUNCE_MS) {
    resetPosture();
    tLastZR = now + 1000;
  }

  // piccola pausa
  delay(5);
}
