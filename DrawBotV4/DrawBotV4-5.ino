#include "WiFi.h"
#include "ArduinoOTA.h"
#include <Wire.h>
#include "PCF8575.h"
#include "AccelStepper.h"
#include <AS5600.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <vector>

// WiFi for OTA Updates
const char *ssid = "YOUR_SSID";
const char *password = "YOUR_PASSWORD";

WebServer server(80);

TwoWire I2C_B = TwoWire(1);

PCF8575 pcf(0x20, &Wire);

// Encoders
AS5600 leftEncoder(&Wire);
AS5600 rightEncoder(&I2C_B);
const int leftEncoderDir = 4;
const int rightEncoderDir = 18;

// Limit Switches
const int leftLimitSwitch = 5;
const int rightLimitSwitch = 6;

// LEDs
const int indicatorLED = 7;

// Stepper Motors
const int leftPulPin = 23;
const int leftDirPin = 27;
const int rightPulPin = 25;
const int rightDirPin = 26;
AccelStepper leftMotor(AccelStepper::DRIVER, leftPulPin, leftDirPin);
AccelStepper rightMotor(AccelStepper::DRIVER, rightPulPin, rightDirPin);
const int leftEnablePin = 21;
const int rightEnablePin = 22;

// Inverse Kinematics Variables
double x = 50.0;
double y = 120.0;
double l1 = 80.0;
double l2 = 150.0;
double c;
double d = 112.25;
double e;
double a1;
double a2;
double aFinal;
double b1;
double b2;
double bFinal;

// Step Angles and Micro Stepping
const double anglePerStep = 1.8;
const double microSteppingLevel = 0.125;
const double microSteppingAngle = anglePerStep * microSteppingLevel;

// Motor Speed
const double speed = 1000.0;
const double acceleration = 900.0;

// Motor Step Error Rate
const double errorThreshold = 0.5;
double leftMotorError = 0.0;
double rightMotorError = 0.0;

// Coordinate Storage
struct Point {
  double x;
  double y;
};

std::vector<Point> coords;

// Angle Offsets
const double leftAngleOffset = 0.0;
const double rightAngleOffset = 0.0;

// Function for calculating steps to take
double calculateSteps(double newAngle, double currentAngle) {
  double angleDifference = currentAngle - newAngle;

  double adjustedAngleDifference = 0.0;
  if (angleDifference < 0) {
    adjustedAngleDifference = abs(angleDifference);
  } else {
    adjustedAngleDifference = angleDifference;
  }

  // Calculate Steps
  double stepCount = adjustedAngleDifference / microSteppingAngle;
  stepCount = (long)stepCount;

  double finalStepCount = 0.0;
  if (angleDifference < 0) {
    finalStepCount = -abs(stepCount);
  } else {
    finalStepCount = stepCount;
  }

  return finalStepCount;
}

double calculateStepsShort(double newAngle, double currentAngle) {
  double angleDifference = currentAngle - newAngle;

  if (angleDifference > 180.0)  angleDifference -= 360.0;
  if (angleDifference < -180.0) angleDifference += 360.0;

  double stepCount = (long)(abs(angleDifference) / microSteppingAngle);
  return (angleDifference < 0) ? -abs(stepCount) : stepCount;
}

// Function for doing Inverse Kinematics
void inverseKinematics(double xCoord, double yCoord) {
  c = sqrt((xCoord * xCoord) + (yCoord * yCoord));
  e = sqrt(((d - xCoord) * (d - xCoord) + (yCoord * yCoord)));

  // First Angle
  a1 = atan2(yCoord, xCoord);
  a2 = acos(((l1 * l1) + (c * c) - (l2 * l2)) / (2 * l1 * c));
  aFinal = (a1 + a2) * 57.296 + leftAngleOffset;

  // Second Angle
  b1 = atan2(yCoord, (d - xCoord));
  b2 = acos(((e * e) + (l1 * l1) - (l2 * l2)) / (2 * e * l1));
  bFinal = 180 - ((b1 + b2) * 57.296) + rightAngleOffset;

  if (bFinal < 0) bFinal += 360.0;
}

void homeMotors() {
  // Home Left Motor
  // Turn On Left Motor
  digitalWrite(leftEnablePin, LOW);
  // Move motor until limit switch is pressed
  leftMotor.setSpeed(-100.0);
  while (pcf.read(leftLimitSwitch) != LOW) {
    leftMotor.runSpeed();
  }
  // Set new angle of the left encoder
  double leftCurrentAngle = leftEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  leftEncoder.setOffset(-leftCurrentAngle + 270);
  leftCurrentAngle = leftEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  Serial.print("Left Motor Current Angle: ");
  Serial.println(leftCurrentAngle);
  // Prep left motor for homing right motor
  leftMotor.setCurrentPosition(0.0);
  leftMotor.setSpeed(500.0);
  leftMotor.setAcceleration(400.0);
  double leftStepCount = calculateSteps(45.0, leftCurrentAngle);
  leftMotor.move(leftStepCount);
  while (leftMotor.distanceToGo() != 0) {
    leftMotor.run();
  }
  leftMotor.setSpeed(0.0);
  leftMotor.stop();
  leftMotor.setCurrentPosition(0.0);

  // Home Right Motor
  // Turn on right motor
  digitalWrite(rightEnablePin, LOW);
  // Move Motor until limit switch is pressed
  rightMotor.setSpeed(100.0);
  while (pcf.read(rightLimitSwitch) != LOW) {
    rightMotor.runSpeed();
  }
  rightMotor.setSpeed(0.0);
  rightMotor.stop();
  rightMotor.setCurrentPosition(0.0);
  delay(50);
  // Set first new angle of the right encoder
  double rightCurrentAngle = rightEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  rightEncoder.setOffset(-rightCurrentAngle); // Sets Current Angle To 0 Degrees
  rightCurrentAngle = rightEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  Serial.print("Right Motor Current Angle 1: ");
  Serial.println(rightCurrentAngle);
  // Move to 90 degrees and reset back to 0 degrees
  rightMotor.setSpeed(500.0);
  rightMotor.setAcceleration(400.0);
  double rightStepCount = calculateSteps(90.0, rightCurrentAngle);
  rightMotor.move(rightStepCount);
  while (rightMotor.distanceToGo() != 0) {
    rightMotor.run();
  }
  rightMotor.setSpeed(0.0);
  rightMotor.stop();
  rightMotor.setCurrentPosition(0.0);
  rightCurrentAngle = rightEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  float oldRightOffset = rightEncoder.getOffset();
  rightEncoder.setOffset(oldRightOffset - rightCurrentAngle); // Sets Current Angle To 0 Degrees
  rightCurrentAngle = rightEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  Serial.print("Right Motor Current Angle 2: ");
  Serial.println(rightCurrentAngle);

  // Move Left Motor back to 180
  leftCurrentAngle = leftEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  leftMotor.setSpeed(500.0);
  leftMotor.setAcceleration(400.0);
  leftStepCount = calculateSteps(180.0, leftCurrentAngle);
  leftMotor.move(leftStepCount);
  while (leftMotor.distanceToGo() != 0) {
    leftMotor.run();
  }
  leftMotor.setSpeed(0.0);
  leftMotor.stop();
  leftMotor.setCurrentPosition(0.0);

  Serial.print("Applied final offsets -> Left: ");
  Serial.print(leftAngleOffset);
  Serial.print("  Right: ");
  Serial.println(rightAngleOffset);
}

bool pointReachable(double xCoord, double yCoord) {
  // Precheck
  if (yCoord < 75) {
    return false;
  }

  double leftPointDistance = sqrt((xCoord * xCoord) + (yCoord * yCoord));
  double rightPointDistance = sqrt(((d - xCoord) * (d - xCoord)) + (yCoord * yCoord));

  bool leftPointReachable = (abs(l1 - l2) <= leftPointDistance) && (leftPointDistance <= (l1 + l2));
  bool rightPointReachable = (abs(l1 - l2) <= rightPointDistance) && (rightPointDistance <= (l1 + l2));

  if (leftPointReachable && rightPointReachable) {
    return true;
  } else {
    return false;
  }
}

void moveMotors(double xCoord, double yCoord) {
  // Reachability Check
  pcf.write(indicatorLED, LOW);
  if (!pointReachable(xCoord, yCoord)) {
    pcf.write(indicatorLED, HIGH);
    return;
  }

  inverseKinematics(xCoord, yCoord);

  double leftCurrentAngle = leftEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  leftMotor.setSpeed(speed);
  leftMotor.setAcceleration(acceleration);
  double leftStepCount = calculateStepsShort(aFinal, leftCurrentAngle);
  leftMotor.move(leftStepCount);

  double rightCurrentAngle = rightEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  rightMotor.setSpeed(speed);
  rightMotor.setAcceleration(acceleration);
  double rightStepCount = calculateStepsShort(bFinal, rightCurrentAngle);
  rightMotor.move(rightStepCount);

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
    }

    if (rightMotor.distanceToGo() != 0) {
      rightMotor.run();
    } else {
      rightMotorArrived = true;
      rightMotor.setSpeed(0.0);
      rightMotor.stop();
      rightMotor.setCurrentPosition(0.0);
    }

    if (rightMotorArrived && leftMotorArrived) {
      runMotors = false;
    }
  }
  
  // Double Check Left Motor
  leftCurrentAngle = leftEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  leftMotorError = abs(aFinal - leftCurrentAngle);
  if (leftMotorError > errorThreshold) {
    bool stopMainLoop = false;
    while (!stopMainLoop) {
      leftCurrentAngle = leftEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
      // Read the current angle and see if the error difference is too high
      if (errorThreshold > abs(aFinal - leftCurrentAngle)) {
        stopMainLoop = true;
        break;
      }
      leftMotor.setSpeed(speed);
      leftMotor.setAcceleration(acceleration);
      leftStepCount = calculateStepsShort(aFinal, leftCurrentAngle);
      leftMotor.move(leftStepCount);
      while (leftMotor.distanceToGo() != 0) {
        leftMotor.run();
      }
      leftMotor.setSpeed(0.0);
      leftMotor.stop();
      leftMotor.setCurrentPosition(0.0);
    }
  }

  // Double Check Right Motor
  rightCurrentAngle = rightEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
  rightMotorError = abs(bFinal - rightCurrentAngle);
  if (rightMotorError > errorThreshold) {
    bool mainStopLoop = false;
    while(!mainStopLoop) {
      rightCurrentAngle = rightEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
      // Read curretn angle and see if the error difference is too high
      if (errorThreshold > abs(bFinal - rightCurrentAngle)) {
        mainStopLoop = true;
        break;
      }
      rightMotor.setSpeed(speed);
      rightMotor.setAcceleration(acceleration);
      rightStepCount = calculateStepsShort(bFinal, rightCurrentAngle);
      rightMotor.move(rightStepCount);
      while (rightMotor.distanceToGo() != 0) {
        rightMotor.run();
      }
      rightMotor.setSpeed(0.0);
      rightMotor.stop();
      rightMotor.setCurrentPosition(0.0);
    }
  }
  
}

void handlePoints() {
  JsonDocument doc;
  deserializeJson(doc, server.arg("plain"));

  // Loop through the sent points and store them in the array
  for (JsonArray point : doc.as<JsonArray>()) {
    double xPoint = point[0];
    double yPoint = point[1];
    
    coords.push_back({xPoint, yPoint});
  }

  String angleStorage = "[";

  // Move the pen to those points
  for (int i = 0; i < coords.size(); i++) {
    moveMotors(coords[i].x, coords[i].y);

    // === Angle Correction Read Test ===
    angleStorage += "(";
    angleStorage += leftEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
    angleStorage += ", ";
    angleStorage += rightEncoder.readAngle() * AS5600_RAW_TO_DEGREES;
    angleStorage += "), ";
  }
  angleStorage += "]";

  // Clear the array so it's ready for the next set of points
  coords.clear();
  server.send(200, "text/plain", angleStorage);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // == WIFI SETUP ==
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  delay(1000);
  Serial.println(WiFi.localIP());

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

  // == I2C Wire and IO Expander Setup == 
  Wire.begin(32, 33);
  // I2C Scanner
  Serial.println("Scanning I2C bus...");
  for (int addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Device found at 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("Scan complete.");

  if (!pcf.begin()) {
    Serial.println("Could Not initalize IO exapander...");
  }
  if (!pcf.isConnected()) {
    Serial.println("IO Expander Not Connected :(");
  } else {
    Serial.println("IO Expander Connected! :)");
  }
  pcf.begin();

  // Initalize Limit Switches
  pcf.write(leftLimitSwitch, HIGH);
  pcf.write(rightLimitSwitch, HIGH);

  // Initalize Indicator LED
  pcf.write(indicatorLED, LOW);

  // == Encoders ==
  leftEncoder.begin(leftEncoderDir);
  leftEncoder.setDirection(AS5600_COUNTERCLOCK_WISE);
  Serial.printf("Left  encoder connected: %s\n", leftEncoder.isConnected() ? "Yes" : "No");

  I2C_B.begin(16, 17);

  rightEncoder.begin(rightEncoderDir);
  rightEncoder.setDirection(AS5600_COUNTERCLOCK_WISE);
  Serial.printf("Right encoder connected: %s\n", rightEncoder.isConnected() ? "Yes" : "No");

  // Home Motors and Encoders
  homeMotors();
  Serial.print("Right Home Angle: ");
  Serial.println(rightEncoder.readAngle() * AS5600_RAW_TO_DEGREES);
  Serial.print("Left Home Angle: ");
  Serial.println(leftEncoder.readAngle() * AS5600_RAW_TO_DEGREES);
  
  // Move pen to starting point
  moveMotors(x, y);
  Serial.print("Right Starting Point Angle: ");
  Serial.println(rightEncoder.readAngle() * AS5600_RAW_TO_DEGREES);
  Serial.print("Left Starting Point Angle: ");
  Serial.println(leftEncoder.readAngle() * AS5600_RAW_TO_DEGREES);

  server.on("/points", HTTP_POST, handlePoints);
  server.begin();

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.println("Draw Bot Started!");
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
}
