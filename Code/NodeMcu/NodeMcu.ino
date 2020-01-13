#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include "FirebaseHttpClient.h"
#include "FirebaseError.h"
#include "FirebaseObject.h"
#include <SoftwareSerial.h>
SoftwareSerial arduino(4,0); // RX, TX
#define firebaseURl "cband-1661e.firebaseio.com"
#define authCode "xHBQFxfehvmbAk7JZnUtkn6DImi4nbr6QcAAomtF"
#define wifiName "waleed"
#define wifiPass "waleed129"
String phrase;

void setupFirebase() {
  Firebase.begin(firebaseURl, authCode);
}

void setupWifi() {
  WiFi.begin(wifiName, wifiPass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
    delay(500);
  }
}

void setup() {
  Serial.begin(9600);
  arduino.begin(9600);
  setupWifi();
  setupFirebase();
}


void loop() {
   while (arduino.available())
      {   
          Serial.println("Check");
          phrase=arduino.readStringUntil('\n');
          Firebase.pushString("c-inno-band/Sensor",phrase);
          delay(100);
          Serial.print(phrase);
    }
  
  // handle error
  if (Firebase.failed()) {
      Serial.print("setting /number failed:");
      Serial.println(Firebase.error());  
      return;
  }
  delay(500);
}
