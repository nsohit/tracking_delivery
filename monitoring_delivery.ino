// This example uses an Arduino MKR GSM 1400 board
// to connect to shiftr.io.
//
// IMPORTANT: This example uses the new MKRGSM library.
//
// You can check on your device after a successful
// connection here: https://shiftr.io/try.
//
// by Sandeep Mistry
// https://github.com/256dpi/arduino-mqtt

#include <MKRGSM.h>
#include <MQTT.h>
#include <TinyGPS.h> 
#include <math.h>
#include <SimpleDHT.h>

const int y_out = A6;
int y_adc_value;

int pinDHT22 = 7;
SimpleDHT22 dht22(pinDHT22);
 
#define SerialGPS Serial1
TinyGPS gps;

const char pin[]      = "";
const char apn[]      = "internet";
const char login[]    = "";
const char password[] = "";

GSMClient net;
GPRS gprs;
GSM gsmAccess;
MQTTClient client;

unsigned long lastMillis = 0;

void connect() {
  // connection state
  bool connected = false;

  Serial.print("connecting to cellular network ...");

  // After starting the modem with gsmAccess.begin()
  // attach to the GPRS network with the APN, login and password
  while (!connected) {
    if ((gsmAccess.begin(pin) == GSM_READY) &&
        (gprs.attachGPRS(apn, login, password) == GPRS_READY)) {
      connected = true;
    } else {
      Serial.print(".");
      delay(1000);
    }
  }

  Serial.print("\nconnecting...");
  while (!client.connect("arduino", "NUR_SOHIT17", "98b36ea5b70a42818fdcdde6453f9c3c")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("NUR_SOHIT17/feeds/tes");
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload,String &payload2) {
  Serial.println("incoming: " + topic + " - " + payload+ " - " + payload);
}

void setup() {
  Serial.begin(9600);
  SerialGPS.begin(9600); //TEST GPS.

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported by Arduino.
  // You need to set the IP address directly.
  client.begin("io.adafruit.com", net);
//  client.onMessage(messageReceived);

  connect();
}

void loop() {
    float temperature = 0;
  float humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); Serial.println(err);delay(2000);
    return;
  }
  float suhu =(float)temperature;
  y_adc_value = analogRead(y_out); 
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  client.loop();
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (SerialGPS.available())
    {
      char c = SerialGPS.read();
      // Serial.write(c); // hilangkan koment jika mau melihat data yang dikirim dari modul GPS
      if (gps.encode(c)) // Parsing semua data
        newData = true;
    }
  }
   float flat, flon,sat;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
   // Serial.print("LAT=");
    (flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    //Serial.print(" LON=");
    (flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    sat = (gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    String mesage1 = String (suhu);
    String mesage2 = String (y_adc_value);
    String mesage3 = String (flat,6)+","+String (flon,6);
  client.loop();

  if (!client.connected()) {
    connect();
  }

  // publish a message roughly every second.
  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
    client.publish("NUR_SOHIT17/feeds/suhu",mesage1);
    delay(3000);
    client.publish("NUR_SOHIT17/feeds/getaran",mesage2);
    delay(3000);
    client.publish("NUR_SOHIT17/feeds/gps",mesage3);
    delay(3000);
  }

  
}
