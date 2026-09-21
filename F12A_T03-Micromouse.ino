#include "Motor.hpp"
#include "Encoder.hpp"
#include "PID.hpp"
#include "Display.hpp"
#include "Model.hpp"
#include "StateEstimator.hpp"
#include "PurePursuit.hpp"
#include "LIDAR.hpp"

// Motor
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10

// Encoder
#define EN_1_A 2
#define EN_1_B 7
#define EN_2_A 3
#define EN_2_B 8

// Wheel (m)
#define WHEEL_DIAMETER 0.03f
#define WHEEL_RADIUS   (WHEEL_DIAMETER / 2.0f)
#define WHEEL_BASE     0.08f

// PID tuning
#define PID_KP 100.0f
#define PID_KI 1.5f
#define PID_KD 3.0f

// Pure pursuit
#define PP_LOOKAHEAD   0.06f
#define PP_CRUISE_SPEED 0.2f
#define PWM_PER_MPS    800.0f

#define DISPLAY_ENABLED false
#define SERIAL_ENABLED true

const Waypoint2D WAYPOINTS[] = {
  {0.00f, 0.00f},
  {0.20f, 0.00f},
  {0.20f, 0.20f},
  {0.00f, 0.20f},
  {0.00f, 0.00f},
};
const size_t WAYPOINT_LEN = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);

Motor motorL(MOT1PWM, MOT1DIR);
Motor motorR(MOT2PWM, MOT2DIR);
Encoder encL(EN_1_A, EN_1_B);
Encoder encR(EN_2_A, EN_2_B);
PID pidL(PID_KP, PID_KI, PID_KD);
PID pidR(PID_KP, PID_KI, PID_KD);
Model model(WHEEL_BASE);
StateEstimator stateEstimator(model, encL, encR, WHEEL_RADIUS);
PurePursuit purePursuit(WHEEL_BASE, PP_LOOKAHEAD, PP_CRUISE_SPEED);
Display display;
Lidar lidarL (A0);
Lidar lidarR (A1);
Lidar lidarF (A2);

bool displayOk = false;
uint32_t lastDisplayMs = 0;
float prevLeftDist = 0;
float prevRightDist = 0;
uint32_t lastControlUs = 0;

static constexpr uint32_t kDisplayIntervalMs = 500;

void updateMotion() {
  stateEstimator.update();

  uint32_t nowUs = micros();
  float dt = 0;
  if (lastControlUs != 0) {
    dt = (nowUs - lastControlUs) * 1e-6f;
  }
  lastControlUs = nowUs;
  if (dt <= 0) {
    return;
  }

  float vLeftMeas = (stateEstimator.getLeftDist() - prevLeftDist) / dt;
  float vRightMeas = (stateEstimator.getRightDist() - prevRightDist) / dt;
  prevLeftDist = stateEstimator.getLeftDist();
  prevRightDist = stateEstimator.getRightDist();

  Pose2D pose = {
    stateEstimator.getX(),
    stateEstimator.getY(),
    stateEstimator.getTheta(),
  };

  float vLeftCmd = 0;
  float vRightCmd = 0;
  if (purePursuit.compute(pose, vLeftCmd, vRightCmd)) {
    pidL.setTarget(vLeftCmd);
    pidR.setTarget(vRightCmd);
    int16_t pwmL = constrain((int16_t)(pidL.compute(vLeftMeas) + vLeftCmd * PWM_PER_MPS), -255, 255);
    int16_t pwmR = constrain((int16_t)(pidR.compute(vRightMeas) + vRightCmd * PWM_PER_MPS), -255, 255);
    motorL.setPWM(pwmL);
    motorR.setPWM(pwmR);
  } else {
    motorL.stop();
    motorR.stop();
    pidL.reset();
    pidR.reset();
  }
}

void renderDisplay() {
  display.clear();
  display.printPose(0, stateEstimator.getX(), stateEstimator.getY(), stateEstimator.getTheta());
  display.printWheelDist(4, stateEstimator.getLeftDist(), stateEstimator.getRightDist());
  display.show();
}

void updateDisplayIfDue() {
  if (!displayOk) return;
  if (millis() - lastDisplayMs < kDisplayIntervalMs) return;

  lastDisplayMs = millis();
  renderDisplay();
}

void updateSerialOut() {
  if (!SERIAL_ENABLED) return;

  Serial.print(stateEstimator.getX(), 5);
  Serial.print(',');
  Serial.print(stateEstimator.getY(), 5);
  Serial.print(',');
  Serial.print(stateEstimator.getTheta(), 5);
  Serial.print(',');
  Serial.print(stateEstimator.getLeftDist(), 5);
  Serial.print(',');
  Serial.print(stateEstimator.getRightDist(), 5);
  Serial.print(',');
  Serial.print(stateEstimator.getVx(), 5);
  Serial.print(',');
  Serial.print(stateEstimator.getVy(), 5);
  Serial.print(',');
  Serial.println(stateEstimator.getOmega(), 5);

  //Print LIDAR
  lidarF.read();
  lidarL.read();
  lidarR.read();

  Serial.print("Front: ");
  lidarF.printRange(Serial);
  Serial.print("  Left: ");
  lidarL.printRange(Serial);
  Serial.print("  Right: ");
  lidarR.printRange(Serial);
  Serial.println();
}

void setup() {
  if (SERIAL_ENABLED) {
    Serial.begin(115200);
    Wire.begin();
    Lidar::initAll();
  }
  delay(1000);

  displayOk = DISPLAY_ENABLED && display.begin();
  lastDisplayMs = millis();
  lastControlUs = micros();
  prevLeftDist = stateEstimator.getLeftDist();
  prevRightDist = stateEstimator.getRightDist();
  pidL.reset();
  pidR.reset();
  purePursuit.setPath(WAYPOINTS, WAYPOINT_LEN);
}

void loop() {
  updateMotion();
  updateDisplayIfDue();
  updateSerialOut();
  
}
