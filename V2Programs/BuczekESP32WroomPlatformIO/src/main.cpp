#include <Arduino.h>
#include "Adafruit_VL53L0X.h"

#if 1

void setup() {
  Serial.begin(9600);
  

}

void loop() {
  int val = analogRead(12);

  Serial.println(val);
  delay(20);
}

#endif

#if 0
#define XSHUT_A 18
#define XSHUT_B 19

Adafruit_VL53L0X lox_a = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_b = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  pinMode(XSHUT_A, OUTPUT);
  pinMode(XSHUT_B, OUTPUT);

  digitalWrite(XSHUT_A, LOW);
  digitalWrite(XSHUT_B, LOW);
  delay(10);

  Serial.println("Adafruit VL53L0X test.");
  
  digitalWrite(XSHUT_A, HIGH);
  if (!lox_a.begin(0x30)) {
    Serial.println(F("Failed to boot VL53L0X A"));
    while(1);
  }

  digitalWrite(XSHUT_B, HIGH);
  if (!lox_b.begin(0x36)) {
    Serial.println(F("Failed to boot VL53L0X B"));
    while(1);
  }


  // power
  Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));

  // start continuous ranging
  lox_a.startRangeContinuous();
  lox_b.startRangeContinuous();
}

uint16_t a_val;
uint16_t b_val;

void loop() {
  if (lox_a.isRangeComplete()) {
    a_val = lox_a.readRange();

    Serial.print("Distance in mm: ");
    Serial.print(a_val);
    Serial.print("  ");
    Serial.println(b_val);
  }

  if (lox_b.isRangeComplete()) {
    b_val = lox_b.readRange();
  }


  
}

#endif