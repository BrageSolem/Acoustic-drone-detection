#include <Arduino.h>


/* connectors on the h -bridge
 * left side
 * pin 1 + pin 9 - 5V
 * pin 2 - d2
 * pin 3 - Green motor
 * pin 4 - gnd
 * pin 5 empty
 * pin 6 - black motor
 * pin 7 - d3
 * pin 8 - 3V8
 *
 * right side
 * pin 16 - 5V
 * pin 15 - d6
 * pin 14 - Red motor
 * pin 13 - gnd
 * pin 12 empty
 * pin 11 - Blue motor
 * pin 10 - d10
 * pin 9 - pin 1 - 5V
 *
 **/

const int IN1 = 2;   // pin 2, 1A, coil 1 direction
const int IN2 = 3;   // pin 7, 2A, coil 1 direction
const int IN3 = 10;  // pin 10, 3A, coil 2 direction
const int IN4 = 6;   // pin 15, 4A , coil 2 direction

void setStep(int a, int b, int c, int d) {
    digitalWrite(IN1, a);
    digitalWrite(IN2, b);
    digitalWrite(IN3, c);
    digitalWrite(IN4, d);
}

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    setStep(1,0,0,0); delay(50);
    setStep(0,1,1,0); delay(50);
    setStep(0,1,0,1); delay(50);
    setStep(1,0,0,1); delay(50);

}