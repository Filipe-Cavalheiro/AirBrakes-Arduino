#include <Servo.h>
#include "AirBrakes.h"

#define APOGEE 3000
#define MAX_AREA 0.004484
#define KP 8
#define KI 1
#define KD 1
#define ROCKET_CD 0.5
#define AB_CD 1.17
#define MASS 23088.0
#define ENDCHAR "#"

Servo myservo;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  myservo.attach(3);
  myservo.write(0);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  Parameters param(APOGEE, KP, KI, KD, 0.01767, MAX_AREA, ROCKET_CD, AB_CD);
  Rocket rocket(0, 0, 0, MASS);
  Miscellaneous misc(0, 0, 0, 0, 0, 1.0);
  AirBrakes ab;

  String buffer;
  double values[4];
  char* token;
  double AB_drag;

  while (true) {
    if (Serial.available() == 0) {
      Serial.print("A");
      Serial.print(ENDCHAR);
      delay(100);
      continue;
    }

    buffer = Serial.readString();
    token = strtok((char*)buffer.c_str(), ",");
    for (int i = 0; i < 4; ++i) {
      values[i] = atof(token);
      token = strtok(NULL, ",");
      if (token == NULL) {
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      }
    }
    rocket._height = values[1];
    rocket._vertVelocity = values[2];
    rocket._velocity = values[3];

    if (rocket._height >= APOGEE - 1) {
      myservo.write(180);
      while (1); // end the code
    }

    AB_drag = ab.airbrakeForce(rocket, param, misc);

    double area = ab.airBrakeArea(AB_drag, rocket._velocity, ab.airDensity(rocket._height), param._AB_CD);
    int angle = ab.calculateServoAngle(param, area);
    myservo.write(angle);
    Serial.print(angle);
    Serial.print(ENDCHAR);
  }
}