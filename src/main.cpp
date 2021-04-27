#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Ticker.h>
#include <thermistor.h>

enum t_ControllerState {
  CONTROLLER_STATE_IDLE,
  CONTROLLER_STATE_IGNITION,
  CONTROLLER_STATE_RUNNING,
  CONTROLLER_STATE_SHUTTING_DOWN,
};

#define SERVO_HEATER_PIN 10
#define THERMISTOR_PIN A3
#define ON_OFF_PIN 4
#define FAN_MODE_PIN 5
#define RELAY_ON_OFF_PIN 3
#define RELAY_HIGH_LO_PIN 2

t_ControllerState controllerState = CONTROLLER_STATE_IDLE;

// Define Variables we'll be connecting to
double Setpoint, camTemperature, Output;

// Specify the links and initial tuning parameters
double Kp = 0.5, Ki = 0.3, Kd = 0;
PID heaterPID(&camTemperature, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

Servo flowServo;
Servo fakeServo;
thermistor body(THERMISTOR_PIN, 0);

void syncOnServoPulse() {
  while (digitalRead(SERVO_HEATER_PIN) == LOW);
  while (digitalRead(SERVO_HEATER_PIN) == HIGH);
 }

void turnOnServo() {
  flowServo.attach(SERVO_HEATER_PIN, MIN_PULSE_WIDTH + 250, MAX_PULSE_WIDTH  + 250);
  digitalWrite(LED_BUILTIN, HIGH);
}

void turnOffServo() {
  if (flowServo.attached()) {
    syncOnServoPulse();
    flowServo.detach();
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void initFlowServo() {
  turnOnServo();
  flowServo.write(180);
  delay(1000);
  turnOffServo();
}

void fanControl();

void controlLoop() {
  if (heaterPID.Compute()) {
    if (abs(flowServo.read() - Output) >= 1.0) {
      turnOnServo();
      flowServo.write(Output);
      syncOnServoPulse();
      fanControl();
    } else {
      turnOffServo();
    }
  }
}

void sensorLoop() {
  const float x = 0.86;
  camTemperature = camTemperature * x + body.analog2temp() * (1 - x);
}

void reportLoop() {
  Serial.print("On/Off pin:");
  Serial.println(digitalRead(ON_OFF_PIN));
  Serial.print("Fan mode pin:");
  Serial.println(digitalRead(FAN_MODE_PIN));
  Serial.print("Temperature: ");
  Serial.println(camTemperature);
  Serial.print("Setpoint: ");
  Serial.println(Setpoint);
  Serial.print("Output: ");
  Serial.println(Output);
  Serial.println("----------------------");
}

void fanControl() {
  digitalWrite(RELAY_ON_OFF_PIN,  (camTemperature > 50.0));
  digitalWrite(RELAY_HIGH_LO_PIN, (camTemperature < 160.0));
}

void fanControlLoop() {
  if (controllerState != CONTROLLER_STATE_RUNNING) {
    fanControl();
  }
}

void startControlLoop();
void stopControlLoop();

void stateControlLoop() {
  switch (controllerState) {
  case CONTROLLER_STATE_IDLE:
    if (digitalRead(ON_OFF_PIN) == LOW) {
      controllerState = CONTROLLER_STATE_IGNITION;
    }
    break;
  case CONTROLLER_STATE_IGNITION:
    Serial.println("Igniting burner...");
    turnOnServo();
    Output = 30;
    flowServo.write(Output);
    delay(10000);
    Serial.println("Starting control loop...");
    camTemperature = body.analog2temp();
    startControlLoop();
    controllerState = CONTROLLER_STATE_RUNNING;
    Serial.println("Done...");
    break;
  case CONTROLLER_STATE_RUNNING:
    if (digitalRead(FAN_MODE_PIN) == LOW) {
      Setpoint = 200;
    } else {
      Setpoint = 150;
    }
    if (digitalRead(ON_OFF_PIN) == HIGH) {
      controllerState = CONTROLLER_STATE_SHUTTING_DOWN;
    }
    break;
  case CONTROLLER_STATE_SHUTTING_DOWN:
    Serial.println("Stopping control loop..");
    stopControlLoop();
    Serial.println("Shutting down the burner...");
    flowServo.write(30);
    turnOnServo();
    delay(5000);
    flowServo.write(180);
    delay(5000);
    turnOffServo();
    Serial.println("Done...");
    controllerState = CONTROLLER_STATE_IDLE;
    break;
  }
}

Ticker controlLoopTicker(controlLoop, 1000, 0, MILLIS);
Ticker sensorLoopTicker(sensorLoop, 10, 0, MILLIS);
Ticker reportLoopTicker(reportLoop, 1000, 0, MILLIS);
Ticker fanControlLoopTicker(fanControlLoop, 10000, 0, MILLIS);
Ticker stateControlLoopTicker(stateControlLoop, 250, 0, MILLIS);

void startControlLoop() {
  Output = 30;
  heaterPID.SetOutputLimits(0, 53);
  heaterPID.SetMode(AUTOMATIC);
  Setpoint = 100;
  controlLoopTicker.start();
}

void stopControlLoop() {
  Output = 0;
  heaterPID.SetMode(MANUAL);
  Setpoint = 0;
  controlLoopTicker.stop();
}

void setup() {
  Serial.begin(115200);
  fakeServo.attach(9);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ON_OFF_PIN, INPUT);
  pinMode(FAN_MODE_PIN, INPUT);
  digitalWrite(ON_OFF_PIN, HIGH);
  digitalWrite(FAN_MODE_PIN, HIGH);
  digitalWrite(SERVO_HEATER_PIN, HIGH);
  pinMode(RELAY_ON_OFF_PIN, OUTPUT);
  pinMode(RELAY_HIGH_LO_PIN, OUTPUT);
  initFlowServo();
  sensorLoop();
  sensorLoopTicker.start();
  reportLoopTicker.start();
  fanControlLoopTicker.start();
  stateControlLoopTicker.start();
}

void loop() {
  controlLoopTicker.update();
  sensorLoopTicker.update();
  reportLoopTicker.update();
  fanControlLoopTicker.update();
  stateControlLoopTicker.update();
}