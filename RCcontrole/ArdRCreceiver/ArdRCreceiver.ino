/*
* Arduino Wireless Communication Tutorial
*     Example 2 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define ledblue 4
#define ledred 5
#define ledgreen 6
RF24 radio(9, 10); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;
void setup() {
  pinMode(53,OUTPUT);
  pinMode(ledblue, OUTPUT);
  pinMode(ledgreen, OUTPUT);
  pinMode(ledred, OUTPUT);
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00001
  radio.openReadingPipe(1, addresses[0]); // 00002
  radio.setPALevel(RF24_PA_MIN);
  digitalWrite(ledblue, HIGH);
  delay(1000);
  digitalWrite(ledblue, LOW);
  digitalWrite(ledred, HIGH);
  delay(1000);
  digitalWrite(ledred, LOW);
  digitalWrite(ledgreen, HIGH);
  delay(1000);
  digitalWrite(ledgreen, LOW);
}
void loop() {
  delay(5);
  radio.startListening();
  radio.read(&buttonState, sizeof(buttonState));
  if (buttonState == HIGH) {
    digitalWrite(ledblue, HIGH);
  }
  else {
    digitalWrite(ledblue, LOW);
  }
}
