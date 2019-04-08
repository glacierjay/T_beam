/* tbeam_mqtt_client
 *  Glacierjay adapted this from several sources using PubSub client
 *  This program connects to local AP using WiFi, then connects to MQTT server
 *  using PubSub. The MQTT server I am using is mosquitto running on a Raspberry
 *  Pi. I use MQTT.fx running on my PC to send/recieve MQTT messages. 
 *  Publish 1 to topic "Glacierjay" will turn on the blue LED. Publish "2" will
 *  turn it off again. Publish "3" will call send_state() function below which
 *  reports whether the built-in button is "pressed" or "not pressed".
 *  Arduino board type is "T-beam" from "ESP32 Arduino".
 */
#include <WiFi.h>
WiFiClient espClient;
#include <PubSubClient.h>
PubSubClient client(espClient);
#define in_topic "Glacierjay" // use this to accept commands
#define button_topic "Glacierjay/button" // use to send state of button
const char* ssid     = "karenjay"; // home ssid
const char* password = "mounthood"; // home password
const char* mqtt_server = "192.168.1.20"; // RPi 3 model b+ mexpi that runs mosquitto
long lastMsg = 0;
char msg[50];
const int blueLED = 14; // blue LED next to GPS -- on or off when mqtt recieved
const int BUTTON_PIN = 39; // Middle button next to LoRa chip. The one on the right is RESET, careful...

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  randomSeed(micros());
  Serial.print("\nWiFi connected,IP address: ");Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("mqtt received:( ");Serial.print(topic);Serial.print(")");
  payload[length] = '\0'; // terminate it just in case
  Serial.println((char*)payload);
  // process mqtt commands. 1=blue led on, 2=blue led off, 3=status?
  if ((char)payload[0] == '1') digitalWrite(blueLED, HIGH);   // LED on
  else if ((char)payload[0] == '2')digitalWrite(blueLED, LOW);  // LED off
  else if ((char)payload[0] == '3') send_state();
}

void reconnect() {
  while (!client.connected()) {  // Loop until we're reconnected
    String clientId = "TbeamClient-";  // Create a random client ID
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {  // Attempt to connect
      Serial.println("connected to MQTT");
      client.publish(in_topic, "Glacierjay mqtt connected");  // Once connected, publish an announcement...
      client.subscribe(in_topic); // Commands come in to this topic 1=ledOn 2=ledOff 3=sendState
    }
    else {
      Serial.print("MQTT client connect failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);  // Wait 5 seconds before retrying
    }
  }
}

// Send MQTT message whether button is pressed or not
void send_state(){
  if (digitalRead(BUTTON_PIN) == 1){
    client.publish(button_topic,"not pressed");
  }
  else{
    client.publish(button_topic,"pressed");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(blueLED, OUTPUT); // Blue LED near GPS chip
  pinMode(BUTTON_PIN, INPUT);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  send_state();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected())reconnect();
  client.loop();
  long now = millis();
  if (now - lastMsg > 60000) { // publish something every minute
    lastMsg = now;
    client.publish("Glacierjay/alive", "Glacierjay is live");
  }
}
