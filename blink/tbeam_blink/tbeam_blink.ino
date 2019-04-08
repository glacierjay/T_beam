/*
  Blink
  Modified by Glacierjay to flash the blue LED on ttgo T-beam
  Turns on an LED on for one second, then off for one second, repeatedly.
  This example code is in the public domain (from Arduino examples/blink).
*/

// Blink the lights. I didn't find  a platform with LED_BUILTIN defined for T-beam, so
// I initially used board type "ttgo LoRa32-OLED V1" in arduino IDE. Now I see a board
// T-beam also, but for some reason LED_BUILTIN is not working for me. 
// Maybe I am having a version problem. In
// https://github.com/espressif/arduino-esp32/blob/master/variants/t-beam/pins_arduino.h
// there are the following lines:
//     static const uint8_t LED_BUILTIN = 14;
//     #define BUILTIN_LED  LED_BUILTIN 
// 14 is the blue LED near the GPS.
// it compiles with board "T-beam" or "ttgo LoRa32-OLED V1"
const int blueLED = 14;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin as an output.
  pinMode(blueLED, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(blueLED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(blueLED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
