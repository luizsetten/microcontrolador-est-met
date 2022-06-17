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

os_timer_t tmr0; // Cria o Timer.
os_timer_t tmr1;

// Direção do vento
volatile int windDirCount = 0;
volatile float windDir[10];

volatile float rainCount = 0;
volatile float windSpeedCount = 0;

volatile bool sendToServer = false;
const char *station_id = "5663b746-744a-40a4-a590-a7ac9abc48d8";
const float Vcc = 3.03;

// Configurações wifi

#define STASSID "Wimax Luiz Gustavo Setten"
#define STAPSK "34346037"

// Configurações DHT
#define DHTPIN 2      // pino que estamos conectado
#define DHTTYPE DHT11 // DHT 11

//#define I2C_ADDRESS 0x48

#define rain 12 // D6
#define wind 13 // D7

DHT dht(DHTPIN, DHTTYPE);

// Cria rotinas de interrupção (para os dois pinos de interrupção, chuva e velocidade do vento)
void ICACHE_RAM_ATTR funcaoInterrupcaoRain();
void ICACHE_RAM_ATTR funcaoInterrupcaoWind();

void ICACHE_RAM_ATTR funcaoInterrupcaoRain()
{
  rainCount = rainCount + 0.33;
  Serial.println("Choveu");
}

void ICACHE_RAM_ATTR funcaoInterrupcaoWind()
{
  windSpeedCount = windSpeedCount + 0.34;
  Serial.println("Ventou");
}

void sendDataViaWifi(void *x)
{
  sendToServer = true;
}

void sendDataViaWifi1()
{
  float humidity = dht.readHumidity();
  float dhtTemperature = dht.readTemperature();
  float voltage = ads1115.computeVolts(ads1115.readADC_SingleEnded(0));
  float solar_voltage = ads1115.computeVolts(ads1115.readADC_SingleEnded(1));
  float bmpTemperature = bmp.readTemperature();
  float bmpPressure = bmp.readPressure();

  Serial.printf("\n\nDados obtidos: \n");

  Serial.printf("Pino A0: %f\n", voltage);
  Serial.printf("Pino A1: %f\n", solar_voltage);
  Serial.printf("DHT Umidade: %f\n", humidity);
  Serial.printf("DHT Temperatura: %f °C\n", dhtTemperature);
  Serial.printf("BMP Pessão: %f Pa\n", bmpPressure);
  Serial.printf("BMP Temperatura: %f °C\n", bmpTemperature);
  Serial.printf("Altitude Aproximada: %f m\n", bmp.readAltitude(1018));

  char windDirection[200];

  snprintf(windDirection, sizeof windDirection, "\"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\"", windDir[0], windDir[1], windDir[2], windDir[3], windDir[4], windDir[5], windDir[6], windDir[7], windDir[8], windDir[9]);

  Serial.printf("Direção do vento %s \n", windDirection);

  char buf[1000];

  snprintf(buf, sizeof buf,
           "{"
           "\"temperature\": %f,"
           "\"pressure\": %f,"
           "\"humidity\": %f,"
           "\"precipitation\": %f,"
           "\"wind_gust\": %f,"
           "\"wind_speed\": %f,"
           "\"wind_direction\": %s,"
           "\"solar_incidence\": %f,"
           "\"station_id\": \"%s\""
           "}",

           bmpTemperature,
           bmpPressure,
           humidity,
           rainCount,
           windSpeedCount, // em m/s
           windSpeedCount,
           windDirection,
           solar_voltage,
           station_id);

  int httpCode = 0;

  Serial.printf("\n\nIniciando envio de dados\n");
  
  if ((WiFiMulti.run() == WL_CONNECTED))
  {
    Serial.print("Conectado\n");
    WiFiClient client;
    HTTPClient http;
    int tentativas = 0;

    while (httpCode != HTTP_CODE_OK && tentativas < 1)
    {
      tentativas++;

      http.begin(client, "http://wheater-if.ddns.net/api/records");
      http.addHeader("Content-Type", "application/json");

      Serial.printf("Tentativa %d ...\n", tentativas);
      Serial.print("Requisição [HTTP] POST...\n");

      httpCode = http.POST(buf);

      Serial.printf("Resposta [HTTP] POST... code: %d\n", httpCode);

      if (httpCode == HTTP_CODE_OK)
      {
        const String &payload = http.getString();
        Serial.println("Payload recebido:\n <<");
        Serial.println(payload);
        Serial.println(">>");
      }
      else
      {
        Serial.printf("Resposta [HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
        delay(3000);
      }
      http.end();
    }
    rainCount = 0;
    windSpeedCount = 0;
  }
}

void readWindDir(void *z)
{
  if (windDirCount == 10)
  {
    windDirCount = 0;
  }

  windDir[windDirCount] = ads1115.computeVolts(ads1115.readADC_SingleEnded(0) / Vcc) * 360;
  windDirCount = windDirCount + 1;
}

void setup()
{
  os_timer_setfn(&tmr0, readWindDir, NULL);     // Indica ao Timer qual sera sua Sub rotina.
  os_timer_arm(&tmr0, 6000, true);              // Inidica ao Timer seu Tempo em mS e se sera repetido ou apenas uma vez (loop = true)
  os_timer_setfn(&tmr1, sendDataViaWifi, NULL); // Indica ao Timer qual sera sua Sub rotina.
  os_timer_arm(&tmr1, 60000, true);             // Inidica ao Timer seu Tempo em mS e se sera repetido ou apenas uma vez (loop = true)
  attachInterrupt(digitalPinToInterrupt(rain), funcaoInterrupcaoRain, FALLING);
  attachInterrupt(digitalPinToInterrupt(wind), funcaoInterrupcaoWind, FALLING);
  pinMode(rain, INPUT);
  pinMode(wind, INPUT);
  dht.begin();
  ads1115.begin(0x48);

  Wire.begin();
  Serial.begin(115200);

  if (!bmp.begin(0x76))
  { // 0x76 is the address of BMP280
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    while (1)
      delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering.
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time.

  // Inicia a conexão do wifi

  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP(STASSID, STAPSK);

  Serial.print("\n\nConfigurado\n");
}

void loop()
{
  delay(50);

  if (sendToServer)
  {
    sendDataViaWifi1();
    sendToServer = false;
  }
}

// http://cta.if.ufrgs.br/projects/estacao-meteorologica-modular/wiki/Anem%C3%B4metro
// https://pt.aliexpress.com/item/4000907599589.html?spm=a2g0o.ppclist.product.2.6c85BVmwBVmwdO&pdp_npi=2%40dis%21COP%21COP%20108%2C758.07%21COP%20108%2C758.07%21%21%21%21%21%402101d1b016554366019494198e242d%2110000010488518431%21btf&_t=pvid%3Ac1f8165a-539b-4653-9eac-5ad1f917bab6&afTraceInfo=4000907599589__pc__pcBridgePPC__xxxxxx__1655436602&gatewayAdapt=glo2bra