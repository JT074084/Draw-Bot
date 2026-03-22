#include "ESP32Servo.h"

Servo servoOne;
Servo servoTwo;

// IK variables
double x = 30.0;
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

const int up = 16;
const int down = 17;
const int left = 25;
const int right = 26;

void setup() {
  Serial.begin(115200);
  delay(100);

  servoOne.attach(14);
  servoTwo.attach(12);

  pinMode(up, INPUT_PULLUP);
  pinMode(down, INPUT_PULLUP);
  pinMode(left, INPUT_PULLUP);
  pinMode(right, INPUT_PULLUP);
}

void loop() {
  // IK Calc
  c = sqrt((x * x) + (y * y));
  e = sqrt(((d - x) * (d - x)) + (y * y));
  alphaOne = atan(y / x) * (180 / PI);
  alphaTwo = acos(((linkTwo * linkTwo) - (c * c) - (linkOne * linkOne)) / (-2 * linkOne * c)) * (180 / PI);

  betaOne = atan(y / (d - x)) * (180 / PI);
  betaTwo = acos(((linkTwo * linkTwo) - (e * e) - (linkOne * linkOne)) / (-2 * linkOne * e)) * (180 / PI);

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

  //Printing the results! 
  Serial.print("alpha One: ");
  Serial.print(alphaOne);
  Serial.print(" |  beta One:");
  Serial.print(betaOne);
  Serial.print(" alpha Two: ");
  Serial.print(alphaTwo);
  Serial.print(" |  beta Two:");
  Serial.print(betaTwo);
  Serial.print(" Alpha: ");
  Serial.print(alphaFinal);
  Serial.print(" |  Beta: ");
  Serial.print(betaFinal);
  Serial.print(" | X: ");
  Serial.print(x);
  Serial.print(" | Y: ");
  Serial.println(y);
  servoOne.write(alphaFinal);
  servoTwo.write(betaFinal);

  if (digitalRead(up) == LOW) {
    y += 5.0;
    delay(50);
  } else if (digitalRead(down) == LOW) {
    y -= 5.0;
    delay(50);
  }

  if (digitalRead(left) == LOW) {
    x -= 5.0;
    delay(50);
  } else if (digitalRead(right) == LOW) {
    x += 5.0;
    delay(50);
  }

  delay(10);
}
