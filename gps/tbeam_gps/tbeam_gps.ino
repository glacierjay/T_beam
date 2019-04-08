/*****************************************
* ESP32 GPS VKEL 9600 Bds
* Modified by Glacierjay
* works on ttgo T-beam, except centiseconds is always zero.
* This program runs on the ttgo T-beam. It sends time and lat/long in a LoRa
* packet on 903.9 MHz every 5 seconds. It uses simple LoRa, not LoRaWAN.
* Works with receiver program ttgo_oled_rcv_lora_V0_3.ino, which receives the
* LoRa packets this program sends.
* To make it compile, install the esp32 boards in Arduino board manager from:
*    https://github.com/espressif/arduino-esp32
* There are directions in the readme for how to install on Windows and other sys.
* The Lora libraries used in these examples are Arduino-lora, from:
*   https://github.com/sandeepmistry/arduino-LoRa
* The sample code is leveraged from several sources, including arduino-esp32 and the
* ttgo exampes:
*      https://github.com/LilyGO/TTGO-T-Beam
* Changes for ver 1:
* 1. Reduced size of LoRa packet for tiny display on LoRa-OLED recieve program.
* 2. Added BUTTON_PIN (39) as input & print state. This is the middle button
* next to the LoRa chip.
* Red/green LEDs next to power switch: The red LED indicates that power is being
* supplied by the USB. When no battery is present it is lit all the time. If there is
* a battery and USB power, the red LED indicates charging, it goes off and the green
* LED comes on when the battery is charged. If there is no USB power and the device is
* running on battery power, neither LEd is lit. If there is no battery and the switch
* is in the "off" position, the green LED is illuminated. 
* TODO: Go to deep sleep mode between gps readings to save battery. Try it
* with LoRaWAN. Figure out how to turn off bright power/charge LED near power switch.
******************************************/

#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa
#include <TinyGPS++.h>                       

TinyGPSPlus gps;                            
// HardwareSerial Serial1(1); // Glacierjay had to comment this out to remove linker warning

                // SPI LoRa Radio
#define LORA_SCK 5        // GPIO5 - SX1276 SCK
#define LORA_MISO 19     // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 -  SX1276 MOSI
#define LORA_CS 18     // GPIO18 -   SX1276 CS
#define LORA_RST 14   // GPIO14 -    SX1276 RST
#define LORA_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)

const int blueLED = 14; // blue LED next to GPS -- blink when packet sent
int pktCount = 0; // count how many packets sent

#define BUTTON_PIN 39

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX for GPS

    // Very important for SPI pin & LoRa configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         

  pinMode(blueLED, OUTPUT); // For LED feedback
  pinMode(BUTTON_PIN, INPUT); // Middle button next to LoRa chip. The one on the right is RESET, careful...

  if (!LoRa.begin(903900000)) {
    Serial.println("Starting LoRa failed!");
    while (1); // if LoRa won't start just die with infinite loop
  }
  LoRa.setSpreadingFactor(7); // ranges from 6-12, default 7 see API docs. Changed for ver 0.1 Glacierjay
  LoRa.setTxPower(14, PA_OUTPUT_PA_BOOST_PIN);
  remote_clear_display(); // tell the receiver to clear the display
}

void remote_clear_display(){ // see ttgo_oled_rcv_lora_V0_3.ino
  LoRa.beginPacket();
  pktCount++;
  LoRa.print("clearDisp");
  LoRa.endPacket();
}

void print_info_json(){
  Serial.print("{\'Valid\': \'");
  Serial.print(gps.location.isValid());
  Serial.print("\', \'Lat\': \'");
  Serial.print(gps.location.lat(), 5);
  Serial.print("\', \'Long\': \'");
  Serial.print(gps.location.lng(), 4);
  Serial.print("\', \'Satellites\': \'");
  Serial.print(gps.satellites.value());
  Serial.print("\', \'Altitude\': \'");
  Serial.print(gps.altitude.feet());
  Serial.print("\', \'Time\': \'");
  Serial.printf("%.2d:%.2d:%.2d",gps.time.hour(),gps.time.minute(),gps.time.second());
  Serial.print("\', \'Button state\': \'");
  Serial.print(digitalRead(BUTTON_PIN));
  Serial.println("\'}");
}

void loop()
{
  print_info_json();
 // send LoRa packet
if (gps.location.isValid()){  // first couple of times through the loop, gps not yet available
  digitalWrite(blueLED, HIGH);  // Turn blue LED on
  LoRa.beginPacket();
  LoRa.printf("Time: %.2d:%.2d:%.2d\n",gps.time.hour(),gps.time.minute(),gps.time.second());
  LoRa.print("Lat: ");
  LoRa.print(gps.location.lat(),4);
  LoRa.print(" Long: ");
  LoRa.print(gps.location.lng(),4);
  LoRa.endPacket();
  digitalWrite(blueLED, LOW); // Turn blue LED off
}
else {
  LoRa.beginPacket();
  LoRa.print("Error: Invalid gps data");
  LoRa.endPacket();
}

  smartDelay(10000);                                      

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}
