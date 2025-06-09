#include <AFMotor.h>

AF_DCMotor m1(1);
AF_DCMotor m2(2);
AF_DCMotor m3(3);
AF_DCMotor m4(4);

void setup() {
  Serial.begin(9600);
  m1.setSpeed(255);
  m2.setSpeed(255);
  m3.setSpeed(255);
  m4.setSpeed(255);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'F') { // 전진
      m1.run(FORWARD); m2.run(FORWARD);
      m3.run(FORWARD); m4.run(FORWARD);
    } 
    else if (cmd == 'B') { // 후진
      m1.run(BACKWARD); m2.run(BACKWARD);
      m3.run(BACKWARD); m4.run(BACKWARD);
    } 
    else if (cmd == 'L') { // 좌회전
      m1.run(BACKWARD); m2.run(FORWARD);
      m3.run(BACKWARD); m4.run(FORWARD);
    } 
    else if (cmd == 'R') { // 우회전
      m1.run(FORWARD); m2.run(BACKWARD);
      m3.run(FORWARD); m4.run(BACKWARD);
    } 
    else if (cmd == 'S') { // 정지
      m1.run(RELEASE); m2.run(RELEASE);
      m3.run(RELEASE); m4.run(RELEASE);
    }
  }
}


