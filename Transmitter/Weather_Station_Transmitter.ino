/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>

//DHT 11 Initilization
#include "DHT.h"
#define DHT_PIN 4
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

Adafruit_BMP085 bmp;

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0x24, 0x6F, 0x28, 0x7C, 0xD9, 0x4c};
//uint8_t broadcastAddress2[] = {0xFF, , , , , };
//uint8_t broadcastAddress3[] = {0xFF, , , , , };

typedef struct readings_struct {
  float temperature, humidity, pressure, rainfall;
} readings_struct;

readings_struct station;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // register peer
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  //Initialize DHT 11 Humidity Sensor and BMP180
  dht.begin();
  bool status = bmp.begin(0x76);

}

void loop() {

  //Collection of weather station data
  station.temperature = bmp.readTemperature();
  station.humidity = dht.readHumidity();
  station.pressure = (bmp.readPressure() / 100.0F);

  //Sending weather station data to reciever
  esp_err_t result = esp_now_send(0, (uint8_t *) &station, sizeof(readings_struct));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
}
