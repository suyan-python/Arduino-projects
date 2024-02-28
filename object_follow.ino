#include<Servo.h>
//motors
#define motora1 6
#define motorb1 3
#define motora2 4
#define motorb2 5

#define enA 12
#define enB 13

//ir sensors
#define RirSensor A0
#define LirSensor A1

//ultrasonic sensor
#define TRIG A5
#define ECHO A4

int duration = 0;
unsigned int distance = 0;

Servo servo;
int angle1 = 0;

int Max_speed = 50;
int righttyre = 30;
int lefttyre  = 30;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(motora1, OUTPUT);
  pinMode(motorb1, OUTPUT);
  pinMode(motora2, OUTPUT);
  pinMode(motorb2, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(LirSensor, INPUT);
  pinMode(RirSensor, INPUT);

  pinMode(TRIG , OUTPUT);
  pinMode(ECHO , INPUT);

  servo.attach(2);

  for (angle1 = 90; angle1 <= 180; angle1 += 1) {
    servo.write(angle1);
    delay(15);
  } for (angle1 = 180; angle1 >= 0; angle1 -= 1) {
    servo.write(angle1);
    delay(15);
  } for (angle1 = 0; angle1 <= 90; angle1 += 1) {
    servo.write(angle1);
    delay(15);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);
  distance = getDistance();
  Serial.print("dist: ");
  Serial.println(distance);

  int LEFT_SENSOR = digitalRead(LirSensor);
  int RIGHT_SENSOR = digitalRead(RirSensor);

  Serial.print("leftsenor: ");
  Serial.print(LEFT_SENSOR);
  Serial.print("   ");
  Serial.print("RIghtsenor: ");
  Serial.print(RIGHT_SENSOR);

  if ((RIGHT_SENSOR == 1) && (LEFT_SENSOR == 1) && (distance >= 10 && distance <= 30)) {
    forward();
        delay(2000);
    Serial.println("Forward");
  // } else if ((RIGHT_SENSOR == 0) && (LEFT_SENSOR == 0) && distance <= 10) {
  //   backward();
  //       delay(2000);
  //   Serial.println("backward");
  }else if ((RIGHT_SENSOR == 0) && (LEFT_SENSOR == 1)) {
    left();
    delay(2000);
    Serial.println("left");
  } else if ((RIGHT_SENSOR == 1) && (LEFT_SENSOR == 0)) {
    right();
    delay(2000);
    Serial.println("right");
  } else if ((RIGHT_SENSOR == 1) && (LEFT_SENSOR == 1)) {
    Stop();
        delay(2000);
    Serial.println("stop");
  }
}
int getDistance() {
  digitalWrite(TRIG , LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG , HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG , LOW);

  duration = pulseIn(ECHO , HIGH);
  distance = (duration / 2) / 29.1 ;  //29.1 is a speed of sound.
  return distance;
}
// void backward()
// {
//   digitalWrite(motora1, HIGH);
//   digitalWrite(motorb1, LOW);
//   digitalWrite(motora2, HIGH);
//   digitalWrite(motorb2, LOW);

//   analogWrite(enA, Max_speed);
//   analogWrite(enB, Max_speed);
// }

void forward()
{
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, HIGH);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, HIGH);

  analogWrite(enA, 30);
  analogWrite(enB, 30);
}

void right()
{
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, HIGH);
  digitalWrite(motora2, HIGH);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, 30);
  analogWrite(enB, 30);
}

void left()
{
  digitalWrite(motora1, HIGH);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, HIGH);

  analogWrite(enA, 30);
  analogWrite(enB, 30);
}
void Stop()
{
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, LOW);
}