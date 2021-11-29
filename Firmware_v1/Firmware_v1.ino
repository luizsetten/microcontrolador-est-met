#include <cstdio>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <user_interface.h>
#include "DHT.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_ADS1X15.h>


ESP8266WiFiMulti WiFiMulti;

Adafruit_BMP280 bmp; // I2C
Adafruit_ADS1115 ads1115;

os_timer_t tmr0; //Cria o Timer.
os_timer_t tmr1;

// Direção do vento
volatile int windDirCount = 0;
volatile float windDir[10];

volatile int rainCount = 0;
volatile int windSpeedCount = 0;

volatile bool sendToServer = false;

// COnfigurações wifi
#define SERVER_IP "192.168.0.106:80"
#ifndef STASSID
#define STASSID "Wimax Luiz Gustavo Setten"
#define STAPSK "34346037"

// Configurações DHT
#define DHTPIN 2      // pino que estamos conectado
#define DHTTYPE DHT11 // DHT 11

//#define I2C_ADDRESS 0x48

#define rain 13
#define wind 15
#endif

DHT dht(DHTPIN, DHTTYPE);

// Cria rotinas de interrupção (para os dois pinos de interrupção, chuva e velocidade do vento)
void ICACHE_RAM_ATTR funcaoInterrupcaoRain();
void ICACHE_RAM_ATTR funcaoInterrupcaoWind();

void ICACHE_RAM_ATTR funcaoInterrupcaoRain(){
  rainCount = rainCount + 1;
  Serial.println("Choveu");
}

void ICACHE_RAM_ATTR funcaoInterrupcaoWind(){
  windSpeedCount = windSpeedCount + 1;
  Serial.println("Ventou");
}

void sendDataViaWifi(void *x) {
  sendToServer = true;
}

void sendDataViaWifi1() {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    Serial.printf("Starting sending data");
  
    int16_t voltage = 0;
    Serial.print("Pino A0: ");
    voltage =  ads1115.readADC_SingleEnded(0);
    Serial.print(voltage);
    Serial.println("");
  
    Serial.print("Pino A1: ");
    voltage =  ads1115.readADC_SingleEnded(1);
    Serial.print(voltage);
    Serial.println("");

    Serial.print("Umidade: ");
    Serial.print(h);
    Serial.print(" %t");
    Serial.print("Temperatura: ");
    Serial.print(t);
    Serial.println(" *C");
//        }
  
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
  
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
  
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1018)); /* Adjusted to local forecast! */
    Serial.println(" m");
    if ((WiFiMulti.run() == WL_CONNECTED)){
    WiFiClient client;
    HTTPClient http;

    if (http.begin(client, "http://192.168.0.106:80/records")) {  // HTTP
      http.addHeader("Content-Type", "application/json");
//
//      Serial.print("[HTTP] POST...\n");
//      // start connection and send HTTP header
//      int httpCode = http.POST("{ \"station_id\": \"adqas8dqasdd8asd8asdjasd\", \"temperature\": 14.5 }");
//
//      // httpCode will be negative on error
//      if (httpCode > 0) {
//        // HTTP header has been send and Server response header has been handled
//        Serial.printf("[HTTP] POST... code: %d\n", httpCode);
//
//        // file found at server
//        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
//          String payload = http.getString();
//          Serial.println(payload);
//        }
//      } else {
//        Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
//      }
//
//      http.end();
//    } else {
//      Serial.printf("[HTTP} Unable to connect\n");

    Serial.print("[HTTP] POST...\n");
    // start connection and send HTTP header and body
    //    int httpCode = http.POST("{\t\n\t\"temperature\": 28.5,\n\t\"pressure\": 880,\n\t\"humidity\": 80,\n\t\"rainfall\": 0,\n\t\"wind_gust\": 2,\n\t\"wind_speed\": 24,\n\t\"wind_direction\": 180,\n\t\"solar_incidence\": 12.5,\n\t\"station_id\": \"5663b746-744a-40a4-a590-a7ac9abc48d8\"\n}");
    char buf[1000];
    snprintf(buf, sizeof buf, "{"
                              "\"temperature\": \"%f\","
                              "\"pressure\": \"%f\","
                              "\"humidity\": \"%f\","
                              "\"rainfall\": \"%f\","
                              "\"wind_gust\": \"%f\","
                              "\"wind_speed\": \"%f\","
                              "\"wind_direction\": \"%f\","
                              "\"solar_incidence\": \"%f\","
                              "\"station_id\": \"5663b746-744a-40a4-a590-a7ac9abc48d8\""
                              "}",
             bmp.readTemperature(), bmp.readPressure(), 1, rainCount * 0.25, 1, bmp.readAltitude(1018), bmp.readAltitude(1018), bmp.readAltitude(1018));


    int httpCode = 0;

    while(httpCode != HTTP_CODE_OK) {  
     Serial.printf("Started while");
     httpCode = http.POST(buf);
     
    // httpCode will be negative on error
    if (httpCode > 0){
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);
      if (httpCode == HTTP_CODE_OK){
        const String &payload = http.getString();
        Serial.println("received payload:\n<<");
        Serial.println(payload);
        Serial.println(">>");
      }
    }
    else{
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
      delay(300);
    }
        http.end();
   }

    rainCount = 0;
    windSpeedCount = 0;
  }
    }

}

void readWindDir(void *z){
  if (windDirCount == 10){
    windDirCount = 0;
  }

  int16_t adc0;
  windDir[windDirCount] = ads1115.readADC_SingleEnded(0);
  windDirCount = windDirCount + 1;
}
//
//float readChannel(ADS1115_MUX channel){
//  float voltage = 0.0;
//  adc.setCompareChannels(channel);
//  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
//  return voltage;
//}

void setup(){
  os_timer_setfn(&tmr0, readWindDir, NULL); //Indica ao Timer qual sera sua Sub rotina.
  os_timer_arm(&tmr0, 6000, true);          //Inidica ao Timer seu Tempo em mS e se sera repetido ou apenas uma vez (loop = true)
  os_timer_setfn(&tmr1, sendDataViaWifi, NULL); //Indica ao Timer qual sera sua Sub rotina.
  os_timer_arm(&tmr1, 6000, true);          //Inidica ao Timer seu Tempo em mS e se sera repetido ou apenas uma vez (loop = true)
  attachInterrupt(digitalPinToInterrupt(rain), funcaoInterrupcaoRain, FALLING);
  attachInterrupt(digitalPinToInterrupt(wind), funcaoInterrupcaoWind, FALLING);
  pinMode(rain, INPUT);
  pinMode(wind, INPUT);
  Serial.begin(9600);
  dht.begin();
  ads1115.begin(0x48);

  Wire.begin();
  Serial.begin(9600);

  if (!bmp.begin(0x76))
  { //0x76 is the address of BMP280
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    while (1)
      delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // Inicia a conexão do wifi
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP(STASSID, STAPSK);
//  WiFi.begin(STASSID, STAPSK);

//  while (WiFi.status() != WL_CONNECTED){
//    delay(500);
//    Serial.print(".");
//  }
//  Serial.println("");
//  Serial.print("Connected! IP address: ");
//  Serial.println(WiFi.localIP());
}

void loop(){
  delay(50);
  if(sendToServer) {
    sendDataViaWifi1();
    sendToServer = false;
  }
}
