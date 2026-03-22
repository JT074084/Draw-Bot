#include "AccelStepper.h"
#include "WiFi.h"
#include "ArduinoOTA.h"

// WiFi for OTA Updates
const char *ssid = "Your SSID Here";
const char *password = "Your Password Here";

// IK variables
double x = 50.0;
double y = 120.0;
double linkOne = 73.55;
double linkTwo = 94.7;
double alphaOne;
double alphaTwo;
double alphaFinal;
double betaOne;
double betaTwo;
double betaFinal;
double c;
double d = 102.5;
double e;

// Controls
const int up = 16;
const int down = 17;
const int left = 25;
const int right = 26;
const int actionButton = 4;

// Stepper Motors
const int leftPulPin = 23;
const int leftDirPin = 27;
const int rightPulPin = 32;
const int rightDirPin = 33;
AccelStepper leftMotor(AccelStepper::DRIVER, leftPulPin, leftDirPin);
AccelStepper rightMotor(AccelStepper::DRIVER, rightPulPin, rightDirPin);
const int leftEnablePin = 21;
const int rightEnablePin = 22;

// Limit Switches
const int leftLimitSwitch = 18;
const int rightLimitSwitch = 19;

// Inverse Kinematics Stuff
double leftCurrentAngle = 0.0;
double rightCurrentAngle = 0.0;
double leftAngleDifference = 0.0;
double rightAngleDifference = 0.0;
double leftNewCalculatedAngle = 0.0;
double rightNewCalculatedAngle = 0.0;
double leftStepCount = 0.0;
double rightStepCount = 0.0;
double leftFinalStepCount = 0.0;
double rightFinalStepCount = 0.0;
const double anglePerStep = 1.8;
const double microSteppingLevel = 0.125;
const double microSteppingAngle = anglePerStep * microSteppingLevel;

bool triggerMovement = false;

void inverseKinematics() {
  c = sqrt((x * x) + (y * y));
  e = sqrt(((d - x) * (d - x)) + (y * y));

  // First angles
  alphaOne = atan(y / x) * (180 / PI);
  alphaTwo = acos(((linkTwo * linkTwo) - (c * c) - (linkOne * linkOne)) / (-2 * linkOne * c)) * (180 / PI);

  // Second Angles
  betaOne = atan(y / (d - x)) * (180 / PI);
  betaTwo = acos(((linkTwo * linkTwo) - (e * e) - (linkOne * linkOne)) / (-2 * linkOne * e)) * (180 / PI);

  // Final Angles
  if (x < 0 ) {
    alphaFinal = 180 + ((alphaOne) + (alphaTwo));
    betaFinal = 180 - ((betaOne) + (betaTwo));
  } else if (x > d) {
    alphaFinal = ((alphaOne) + (alphaTwo));
    betaFinal = -1 * ((betaOne) + (betaTwo)); 
  } else {
    alphaFinal = ((alphaOne) + (alphaTwo));
    betaFinal = 180 - ((betaOne) + (betaTwo));
  }
}

void moveEndEffector() {
  inverseKinematics();
  Serial.printf("alphaFinal: %f, betaFinal: %f\n", alphaFinal, betaFinal);
  // Angle differences
  leftAngleDifference = leftCurrentAngle - alphaFinal;
  rightAngleDifference = rightCurrentAngle - betaFinal;

  if (leftAngleDifference < 0) {
    leftNewCalculatedAngle = abs(leftAngleDifference);
  } else {
    leftNewCalculatedAngle = leftAngleDifference;
  }

  if (rightAngleDifference < 0) {
    rightNewCalculatedAngle = abs(rightAngleDifference);
  } else {
    rightNewCalculatedAngle = rightAngleDifference;
  }

  // Calculate Steps
  leftStepCount = leftNewCalculatedAngle / microSteppingAngle;
  leftStepCount = (long)leftStepCount;
  rightStepCount = rightNewCalculatedAngle / microSteppingAngle;
  rightStepCount = (long)rightStepCount;

  if (leftAngleDifference < 0) {
    leftFinalStepCount = -abs(leftStepCount);
  } else {
    leftFinalStepCount = leftStepCount;
  }

  if (rightAngleDifference < 0) {
    rightFinalStepCount = -abs(rightStepCount);
  } else {
    rightFinalStepCount = rightStepCount;
  }

  // Move the motors to those angles
  // NEW
  leftMotor.setSpeed(500.0);
  leftMotor.setAcceleration(400.0);
  leftMotor.move(leftFinalStepCount);
  rightMotor.setSpeed(500.0);
  rightMotor.setAcceleration(400.0);
  rightMotor.move(rightFinalStepCount);
  bool runMotors = true;
  bool leftMotorArrived = false;
  bool rightMotorArrived = false;
  while (runMotors) {
    if (leftMotor.distanceToGo() != 0) {
      leftMotor.run();
    } else {
      leftMotorArrived = true;
      leftMotor.setSpeed(0.0);
      leftMotor.stop();
      leftMotor.setCurrentPosition(0.0);
      leftCurrentAngle = alphaFinal;
    }

    if (rightMotor.distanceToGo() != 0) {
      rightMotor.run();
    } else {
      rightMotorArrived = true;
      rightMotor.setSpeed(0.0);
      rightMotor.stop();
      rightMotor.setCurrentPosition(0.0);
      rightCurrentAngle = betaFinal;
    }

    if (rightMotorArrived && leftMotorArrived) {
      runMotors = false;
    }
  }


  // OLD
  // leftMotor.setSpeed(500.0);
  // leftMotor.setAcceleration(400.0);
  // leftMotor.move(leftFinalStepCount);
  // while (leftMotor.distanceToGo() != 0) {
  //   leftMotor.run();
  // }
  // leftMotor.setSpeed(0.0);
  // leftMotor.stop();
  // leftMotor.setCurrentPosition(0.0);
  // leftCurrentAngle = alphaFinal;

  // rightMotor.setSpeed(500.0);
  // rightMotor.setAcceleration(400.0);
  // rightMotor.move(rightFinalStepCount);
  // while (rightMotor.distanceToGo() != 0) {
  //   rightMotor.run();
  // }
  // rightMotor.setSpeed(0.0);
  // rightMotor.stop();
  // rightMotor.setCurrentPosition(0.0);
  // rightCurrentAngle = betaFinal;

  triggerMovement = false;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // == WIFI SETUP ==
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // == OTA UPDATES SETUP ==
  // Setup OTA
  ArduinoOTA.onStart([]() {
    Serial.println("OTA: Start updating");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA: End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  delay(500);

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Stepper Motor Setup
  pinMode(leftEnablePin, OUTPUT);
  pinMode(rightEnablePin, OUTPUT);
  leftMotor.setMaxSpeed(2000.0);
  rightMotor.setMaxSpeed(2000.0);
  // Turn Motors off
  digitalWrite(leftEnablePin, HIGH);
  digitalWrite(rightEnablePin, HIGH);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  

  // Button Pin Modes
  pinMode(up, INPUT_PULLUP);
  pinMode(down, INPUT_PULLUP);
  pinMode(left, INPUT_PULLUP);
  pinMode(right, INPUT_PULLUP);
  pinMode(actionButton, INPUT_PULLUP);

  // Limit Switch Pin Modes
  pinMode(leftLimitSwitch, INPUT_PULLUP);
  pinMode(rightLimitSwitch, INPUT_PULLUP);

  // Stepper Motor Homing
  digitalWrite(leftEnablePin, LOW);
  digitalWrite(rightEnablePin, LOW);

  leftMotor.setSpeed(-100.0);
  while (digitalRead(leftLimitSwitch) == HIGH) {
    leftMotor.runSpeed();
  }
  leftCurrentAngle = 180.0;
  leftMotor.setSpeed(0.0);
  leftMotor.stop();
  leftMotor.setCurrentPosition(0.0);

  rightMotor.setSpeed(100.0);
  while (digitalRead(rightLimitSwitch) == HIGH) {
    rightMotor.runSpeed();
  }
  rightCurrentAngle = 0.0;
  rightMotor.setSpeed(0.0);
  rightMotor.stop();
  rightMotor.setCurrentPosition(0.0);

  // Do IK and move pen to a starting point on the grid.
  moveEndEffector();

  digitalWrite(2, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(up) == LOW) {
    y += 5;
    triggerMovement = true;
    delay(10);
  } else if (digitalRead(down) == LOW) {
    y -= 5;
    triggerMovement = true;
    delay(10);
  }

  if (digitalRead(left) == LOW) {
    x -= 5;
    triggerMovement = true;
    delay(10);
  } else if (digitalRead(right) == LOW) {
    x += 5;
    triggerMovement = true;
    delay(10);
  }

  if (digitalRead(actionButton) == LOW) {
    square();
    // triangle();
    // hexagon();
  }

  if (triggerMovement) {
    moveEndEffector();
  }

  delay(100);

  ArduinoOTA.handle();
}

void square() {
  y -= 40;
  moveEndEffector();
  delay(10);
  x += 40;
  moveEndEffector();
  delay(10);
  y += 40;
  moveEndEffector();
  delay(10);
  x -= 40;
  moveEndEffector();
  delay(10);
}

void triangle() {
  x = 50.0;
  y = 130.0;
  moveEndEffector();
  //delay(10);

  x = 20.0;
  y = 90.0;
  moveEndEffector();
  //delay(10);

  x = 80.0;
  y = 90.0;
  moveEndEffector();
  //delay(10);

  x = 50.0;
  y = 130.0;
  moveEndEffector();
  //delay(10);
}

void hexagon() {
  x = 50.0;
  y = 130.0;
  moveEndEffector();
  delay(10);

  x = 30.0;
  y = 120.0;
  moveEndEffector();
  delay(10);

  x = 20.0;
  y = 105.0;
  moveEndEffector();
  delay(10);

  x = 30.0;
  y = 90.0;
  moveEndEffector();
  delay(10);
  
  x = 50.0;
  y = 90.0;
  moveEndEffector();
  delay(10);

  x = 60.0;
  y = 105.0;
  moveEndEffector();
  delay(10);

  x = 50.0;
  y = 130.0;
  moveEndEffector();
  delay(10);
}
