#include <Arduino.h>

//motor driver pins
#define PWMA PA_9
#define AIN1 PA_8
#define AIN2 PB_15

#define PWMB PA_10
#define BIN1 PB_13
#define BIN2 PB_14


enum Side {
  RIGHT,
  LEFT,
  STRAIGHT,
  BACK
};

enum Direction {
  CLOCKWISE,
  COUNTER_CLOCKWISE
};

enum MotionStates {
  FIND,
  ATTACK
} motion_state;

void runMotor(Side motor_side, Direction dir, uint8_t speed);

void setup() {
    //motor driver setup
  pinMode(PWMA,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);

  pinMode(PC_13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(PC_13, HIGH);
  //digitalWrite(AIN2, LOW);

  //analogWrite(PWMA, 250);

  //runMotor(RIGHT, CLOCKWISE, 200);
  //runMotor(LEFT, CLOCKWISE, 200);
}


void runMotor(Side motor_side, Direction dir, uint8_t speed) {
  int controll_pin1;
  int controll_pin2;

  int pwm_pin;

  if (motor_side == RIGHT) {
    controll_pin1 = AIN1;
    controll_pin2 = AIN2;

    pwm_pin = PWMA;
  } else if (motor_side == LEFT) {
    controll_pin1 = BIN1;
    controll_pin2 = BIN2;

    pwm_pin = PWMB;
  }

  int value;

  if (dir == CLOCKWISE) {
    value = HIGH;      
  } else if (dir == COUNTER_CLOCKWISE) {
    value = LOW;
  }

  digitalWrite(controll_pin1, value);
  digitalWrite(controll_pin2, !value);

  analogWrite(pwm_pin, speed);
}
