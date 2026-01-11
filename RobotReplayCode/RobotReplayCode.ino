#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Arduino.h>

// ========== Motor & Servo Setup ==========
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *frontLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *backLeft   = AFMS.getMotor(2);
Adafruit_DCMotor *backRight  = AFMS.getMotor(3);
Adafruit_DCMotor *frontRight = AFMS.getMotor(4);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ     50
#define SERVO_MIN_US   800
#define SERVO_MAX_US   2200
const float z_home = 4.05f;

static const uint8_t servoChannel[6] = {0, 1, 2, 3, 4, 5};
static int zeroPos[6] = {1500, 1500, 1500, 1500, 1500, 1500};
#define INV1 1
#define INV2 3
#define INV3 5

// Servo geometry
static const float servo_min_angle = -1.3963f;
static const float servo_max_angle =  1.3963f;
static const float servo_mult = 400.0f / (PI / 4.0f);
static const float L1 = 0.79f, L2 = 4.66f, PD = 2.99f, RD = 2.42f;
static const float deg30 = (PI / 6.0f);
static const float theta_p = (37.5f * PI / 180.0f);
static const float theta_r = (8.0f * PI / 180.0f);
static const float theta_ang = (PI / 3.0f - theta_p) * 0.5f;

static const float beta[6] = {PI/2, -PI/2, -PI/6, 5*PI/6, -5*PI/6, PI/6};

static const float p[2][6] = {
  {-PD * cosf(deg30 - theta_ang), -PD * cosf(deg30 - theta_ang), PD * sinf(theta_ang),
   PD * cosf(deg30 + theta_ang), PD * cosf(deg30 + theta_ang), PD * sinf(theta_ang)},
  {-PD * sinf(deg30 - theta_ang), PD * sinf(deg30 - theta_ang), PD * cosf(theta_ang),
   PD * sinf(deg30 + theta_ang), -PD * sinf(deg30 + theta_ang), -PD * cosf(theta_ang)}
};

static float re[3][6] = {
  {-RD * sinf(deg30 + theta_r*0.5f), -RD * sinf(deg30 + theta_r*0.5f), -RD * sinf(deg30 - theta_r*0.5f),
    RD * cosf(theta_r*0.5f), RD * cosf(theta_r*0.5f), -RD * sinf(deg30 - theta_r*0.5f)},
  {-RD * cosf(deg30 + theta_r*0.5f), RD * cosf(deg30 + theta_r*0.5f), RD * cosf(deg30 - theta_r*0.5f),
    RD * sinf(theta_r*0.5f), -RD * sinf(theta_r*0.5f), -RD * cosf(deg30 - theta_r*0.5f)},
  {0, 0, 0, 0, 0, 0}
};

static float M[3][3], rxp[3][6], T[3], H[3] = {0, 0, z_home}, theta_a[6] = {0};
float lastZ = 15;

// ========== Ultrasonic Sensor ==========
#define TRIG_PIN 6
#define ECHO_PIN 7
unsigned long lastPingTime = 0;

float readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout
  if (duration == 0) return -1;
  return duration * 0.034 / 2.0;
}

// ========== Platform Code ==========
uint16_t usToPwmTicks(int us) {
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  float ticks = (float)us * 4096.0f / 20000.0f;
  return (uint16_t)(ticks + 0.5f);
}

void writeServoUS(uint8_t channel, int us) {
  pwm.setPWM(channel, 0, usToPwmTicks(us));
}

void getMatrix(float rx, float ry, float rz) {
  float phi = rx, theta = ry, psi = rz;
  M[0][0] = cosf(psi)*cosf(theta);
  M[1][0] = -sinf(psi)*cosf(phi) + cosf(psi)*sinf(theta)*sinf(phi);
  M[2][0] = sinf(psi)*sinf(phi)  + cosf(psi)*cosf(phi)*sinf(theta);
  M[0][1] = sinf(psi)*cosf(theta);
  M[1][1] = cosf(psi)*cosf(phi)  + sinf(psi)*sinf(theta)*sinf(phi);
  M[2][1] = cosf(theta)*sinf(phi);
  M[0][2] = -sinf(theta);
  M[1][2] = -cosf(psi)*sinf(phi) + sinf(psi)*sinf(theta)*cosf(phi);
  M[2][2] = cosf(theta)*cosf(phi);
}

void getTranslation(float x_in, float y_in, float z_in) {
  T[0] = x_in + H[0];
  T[1] = y_in + H[1];
  T[2] = z_in + H[2];
}

void getRotatedPlatformPoints() {
  for (int i = 0; i < 6; i++) {
    rxp[0][i] = T[0] + M[0][0]*re[0][i] + M[0][1]*re[1][i] + M[0][2]*re[2][i];
    rxp[1][i] = T[1] + M[1][0]*re[0][i] + M[1][1]*re[1][i] + M[1][2]*re[2][i];
    rxp[2][i] = T[2] + M[2][0]*re[0][i] + M[2][1]*re[1][i] + M[2][2]*re[2][i];
  }
}

float getAlpha(int i) {
  float th = theta_a[i], minA = servo_min_angle, maxA = servo_max_angle;
  for (int steps = 0; steps < 20; steps++) {
    float qx = L1*cosf(th)*cosf(beta[i]) + p[0][i];
    float qy = L1*cosf(th)*sinf(beta[i]) + p[1][i];
    float qz = L1*sinf(th);
    float dx = rxp[0][i] - qx, dy = rxp[1][i] - qy, dz = rxp[2][i] - qz;
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    float diff = L2 - dist;
    if (fabs(diff) < 0.01f) return th;
    if (dist < L2) maxA = th; else minA = th;
    th = minA + (maxA - minA) * 0.5f;
  }
  return th;
}

int setPlatformPose_mm(float x_mm, float y_mm, float z_mm, float pitchDeg, float rollDeg, float yawDeg) {
  float x_in = x_mm / 25.4f, y_in = y_mm / 25.4f, z_in = z_mm / 25.4f;
  float pitchRad = pitchDeg * (PI / 180.0f), rollRad = rollDeg * (PI / 180.0f), yawRad = yawDeg * (PI / 180.0f);
  getMatrix(pitchRad, rollRad, yawRad);
  getTranslation(x_in, y_in, z_in);
  getRotatedPlatformPoints();
  int errorCount = 0;
  for (int i = 0; i < 6; i++) {
    theta_a[i] = getAlpha(i);
    float offset = theta_a[i] * servo_mult;
    int us = zeroPos[i] + (i == INV1 || i == INV2 || i == INV3 ? -offset : offset);
    if (us < SERVO_MIN_US || us > SERVO_MAX_US) errorCount++;
    writeServoUS(servoChannel[i], us);
  }
  return errorCount;
}

// ========== Motor Code ==========
void setMotor(Adafruit_DCMotor *motor, float dir) {
  float threshold = 0.05;
  if (fabs(dir) < threshold) dir = 0;
  float speedLimit = 0.4;
  int speed = abs(dir * 255 * speedLimit);
  motor->setSpeed(speed);
  if (dir > 0) motor->run(FORWARD);
  else if (dir < 0) motor->run(BACKWARD);
  else motor->run(RELEASE);
}

void drive(float x_drive, float y_drive, float turnL, float turnR) {
  float x = constrain(x_drive, -1.0f, 1.0f);
  float y = constrain(y_drive, -1.0f, 1.0f);
  float r = ((turnR + 1.0f) - (turnL + 1.0f)) / 2.0f;

  float fl = y + x + r;
  float fr = y - x - r;
  float bl = y - x + r;
  float br = y + x - r;

  float maxVal = max(max(abs(fl), abs(fr)), max(abs(bl), abs(br)));
  if (maxVal > 1.0f) {
    fl /= maxVal; fr /= maxVal; bl /= maxVal; br /= maxVal;
  }

  setMotor(frontLeft,  fl);
  setMotor(frontRight, fr);
  setMotor(backLeft,   bl);
  setMotor(backRight,  br);
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  AFMS.begin();
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); 
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  setPlatformPose_mm(0, 0, 15, 0, 0, 0);  // default: shell up
}

// ========== Main Loop ==========
void loop() {
  static String inputString = "";
  static bool stringComplete = false;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      stringComplete = true;
      break;
    } else {
      inputString += c;
    }
  }

  if (stringComplete) {
    inputString.trim();
    float v[10];
    int idx = 0, from = 0;
    while (idx < 10) {
      int next = inputString.indexOf(',', from);
      String num = (next == -1) ? inputString.substring(from) : inputString.substring(from, next);
      v[idx++] = num.toFloat();
      from = next + 1;
      if (next == -1) break;
    }

    if (idx == 10) {
      lastZ = v[2];
      setPlatformPose_mm(v[0], v[1], v[2], v[3], v[4], v[5]);
      drive(v[6], v[7], v[9], v[8]);
    }

    inputString = "";
    stringComplete = false;
  }

  if (lastZ >= 14.5 && millis() - lastPingTime > 150) {
    float dist = readUltrasonicCM();
    if (dist > 0) {
      Serial.print("DIST:");
      Serial.println(dist, 2);
    }
    lastPingTime = millis();
  }
}
