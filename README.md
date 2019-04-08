# T_beam
Learning ttgo T-Beam LoRa + GPS + Wifi ESP32 board using Arduino IDE

These programs work on the T-beam device from ttgo. The T-beam is a low-cost development
board with GPS, LoRa radio, WiFi and BLE, controlled by an ESP32 chip. You can program
it using the Arduino IDE or a number of other development environments. These examples are
all tested on Arduino.

To start programming ESP32 devices on Arduino, first install the board collection in
your Arduino IDE, adding https://dl.espressif.com/dl/package_esp32_index.json into the
Arduiuno board manager.
Directions for adding esp32 boards are at the following link:
https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
I used board type "T-beam" to compile and load my programs.

For more information about the T-beam, start with the following links:
https://github.com/LilyGO/TTGO-T-Beam
http://tinymicros.com/wiki/TTGO_T-Beam

Contents of this repository:
blink: the usual "hello world" program for Arduino boards. This blinks the
    onboard blue LED near the GPS chip. I have not been able to control
    any of the other LEDs onboard.
mqtt: Uses the built-in WiFi to connect to a local MQTT server. Also controls the LED
    and reads the built-in button. 
gps: Reads from the onboard GPS. Sends the location and time over LoRa packets, flashes the
    blue LED when it sends a packet, reads the onboard button, and prints GPS information with 
    Serial.print() in JSON format.

I also have a ttgo-LoRa-OLED device. I can recieve the LoRa packets from the GPS program
above from the Lora reciever program at: https://github.com/glacierjay/ttgo_lora_OLED.git

All these are similar to what is available online.