#include <cstdio>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <user_interface.h>
#include "DHT.h"

//#define SERVER_IP "10.0.1.7:9080" // PC address with emulation on host
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <ADS1115_WE.h>
#include <Wire.h>

Adafruit_BMP280 bmp; // I2C

os_timer_t tmr0; //Cria o Timer.
volatile int windDirCount = 0;
volatile float windDir[10];

#define SERVER_IP "192.168.0.106:80"

#ifndef STASSID
#define STASSID "Wimax Luiz Gustavo Setten"
#define STAPSK "34346037"

#define DHTPIN 2      // pino que estamos conectado
#define DHTTYPE DHT11 // DHT 11

#define I2C_ADDRESS 0x48

#define rain 13
#define wind 15
#endif
DHT dht(DHTPIN, DHTTYPE);

ADS1115_WE adc(I2C_ADDRESS);

void ICACHE_RAM_ATTR funcaoInterrupcaoRain();

void ICACHE_RAM_ATTR funcaoInterrupcaoWind();

void ICACHE_RAM_ATTR funcaoInterrupcaoRain()
{
  Serial.println("Choveu");
}

void ICACHE_RAM_ATTR funcaoInterrupcaoWind()
{
  Serial.println("Ventou");
}

void setup()
{
  os_timer_setfn(&tmr0, readWindDir, NULL); //Indica ao Timer qual sera sua Sub rotina.
  os_timer_arm(&tmr0, 6000, true);          //Inidica ao Timer seu Tempo em mS e se sera repetido ou apenas uma vez (loop = true)
  attachInterrupt(digitalPinToInterrupt(rain), funcaoInterrupcaoRain, FALLING);
  attachInterrupt(digitalPinToInterrupt(wind), funcaoInterrupcaoWind, FALLING);
  pinMode(rain, INPUT);
  pinMode(wind, INPUT);
  Serial.begin(9600);
  dht.begin();

  Wire.begin();
  Serial.begin(9600);
  if (!adc.init())
  {
    Serial.println("ADS1115 nao conectado!");
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_6144);
  adc.setCompareChannels(ADS1115_COMP_3_GND);
  adc.setMeasureMode(ADS1115_CONTINUOUS);

  if (!bmp.begin(0x76))
  { //0x76 is the address of BMP280
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    while (1)
      delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  WiFi.begin(STASSID, STAPSK);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
}

void loop()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  float voltage = 0.0;
  Serial.print("Pino A0: ");
  voltage = readChannel(ADS1115_COMP_0_GND);
  Serial.print(voltage);
  Serial.println("");

  Serial.print("Pino A1: ");
  voltage = readChannel(ADS1115_COMP_1_GND);
  Serial.print(voltage);
  Serial.println("");

  //    if (isnan(t) || isnan(h))
  //    {
  //      Serial.println("Failed to read from DHT");
  //    }
  //    else
  //    {
  Serial.print("Umidade: ");
  Serial.print(h);
  Serial.print(" %t");
  Serial.print("Temperatura: ");
  Serial.print(t);
  Serial.println(" *C");
  //    }

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1018)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println();
  //    delay(2000);
  // wait for WiFi connection
  delay(60000); // Trocar para interrupção por tempo
  if ((WiFi.status() == WL_CONNECTED))
  {

    WiFiClient client;
    HTTPClient http;

    Serial.print("[HTTP] begin...\n");
    // configure traged server and url
    http.begin(client, "http://" SERVER_IP "/api/records"); //HTTP
    http.addHeader("Content-Type", "application/json");

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
             bmp.readTemperature(), bmp.readPressure(), h, bmp.readAltitude(1018), bmp.readAltitude(1018), bmp.readAltitude(1018), bmp.readAltitude(1018), bmp.readAltitude(1018));

    //      Serial.printf(buf);

    int httpCode = http.POST(buf);

    // httpCode will be negative on error
    if (httpCode > 0)
    {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK)
      {
        const String &payload = http.getString();
        Serial.println("received payload:\n<<");
        Serial.println(payload);
        Serial.println(">>");
      }
    }
    else
    {
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);

      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  }
}

void readWindDir(void *z)
{
  if (windDirCount == 10)
  {
    windDirCount = 0;
  }

  windDir[windDirCount] = readChannel(ADS1115_COMP_1_GND);
  windDirCount = windDirCount + 1;
}

float readChannel(ADS1115_MUX channel)
{
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}
