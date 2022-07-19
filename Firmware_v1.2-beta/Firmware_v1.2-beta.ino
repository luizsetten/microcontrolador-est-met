#include <cstdio>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <user_interface.h>
#include <DHT.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_ADS1X15.h>
#include <Time.h>
// Fazer engenharia reversa para adaptar https://github.com/switchdoclabs/SDL_Weather_80422/blob/master/SDL_Weather_80422.cpp

//#include "SDL_Weather_80422.h"
#include <SDL_Weather_80422.h>

//#define pinLED LED_BUILTIN  // LED connected to digital pin 16
//#define pinAnem 13 // Anenometer connected to pin 13 - Int 5 - Mega   / Uno pin 2
//#define pinRain 12 // Anenometer connected to pin 13 - Int 0 - Mega   / Uno Pin 3
//#define intAnem 5  // int 0 (check for Uno)
//#define intRain 1  // int 1

ESP8266WiFiMulti WiFiMulti;

Adafruit_BMP280 bmp; // I2C
Adafruit_ADS1115 ads1115;

os_timer_t tmr0; // Cria o Timer.
os_timer_t tmr1;

// Direção do vento
volatile int windDirCount = 0;
volatile float windDir[10];

volatile float rainCount = 0;
// volatile float windSpeedCount = 0;

volatile bool sendToServer = false;
const char *station_id = "5663b746-744a-40a4-a590-a7ac9abc48d8";
const float Vcc = 3.03;

// Configurações wifi

#define STASSID "wifi1"
#define STAPSK "senha1"

// Configurações DHT
#define DHTPIN 2      // pino que estamos conectado
#define DHTTYPE DHT11 // DHT 11

//#define I2C_ADDRESS 0x48

#define rain 12 // D6
#define wind 13 // D7
#define WIND_FACTOR 2.400

unsigned long lastWindTime;

long currentWindCount = 0;
unsigned long shortestWindTime = 0;
float currentWindSpeed = 0.0;
float sampleTime = 5.0;
unsigned long startSampleTime = 0;

DHT dht(DHTPIN, DHTTYPE);

// Cria rotinas de interrupção (para os dois pinos de interrupção, chuva e velocidade do vento)
void ICACHE_RAM_ATTR funcaoInterrupcaoRain();
void ICACHE_RAM_ATTR funcaoInterrupcaoWind();

void ICACHE_RAM_ATTR funcaoInterrupcaoRain()
{
  rainCount = rainCount + (0.2794 / 2);
  Serial.println("Choveu");
}

void ICACHE_RAM_ATTR funcaoInterrupcaoWind()
{
  unsigned long currentTime = (unsigned long)(micros() - lastWindTime);

  lastWindTime = micros();
  if (currentTime > 1000) // debounce
  {
    currentWindCount++;
    if (currentTime < shortestWindTime)
    {
      shortestWindTime = currentTime;
    }
  }
}

float get_wind_gust()
{
  unsigned long latestTime;
  latestTime = shortestWindTime;
  shortestWindTime = 0xffffffff;
  double time = latestTime / 1000000.0; // in microseconds
  shortestWindTime = 0xffffffff;
  return (1 / (time)) * WIND_FACTOR / 2;
}

float get_current_wind_speed()
{
  startSampleTime = micros();
  unsigned long compareValue;
  compareValue = sampleTime * 1000000;

  if (micros() - startSampleTime >= compareValue)
  {
    float timeSpan;
    timeSpan = (micros() - startSampleTime);

    currentWindSpeed = ((float)currentWindCount / (timeSpan)) * WIND_FACTOR * 1000000;

    currentWindCount = 0;

    startSampleTime = micros();
  }

  return currentWindSpeed;
}

void sendDataViaWifi(void *x)
{
  sendToServer = true;
}

void sendDataViaWifi1()
{
  float humidity = dht.readHumidity();
  float dhtTemperature = dht.readTemperature();
  float direction_voltage = ads1115.computeVolts(ads1115.readADC_SingleEnded(0));
  float solar_voltage = ads1115.computeVolts(ads1115.readADC_SingleEnded(1));
  float solar_incidence = (solar_voltage * 1000) / (39.1 * 76.09);
  float bmpTemperature = bmp.readTemperature();
  float bmpPressure = bmp.readPressure();
  float windSpeed = get_current_wind_speed() / 3.6;
  float windGust = get_wind_gust() / 3.6;

  Serial.printf("\n\nDados obtidos: \n");

  Serial.printf("Pino A0: %f\n", voltage);
  Serial.printf("Pino A1: %f\n", (solar_voltage * 1000) / (39.1 * 76.09));
  Serial.printf("DHT Umidade: %f\n", humidity);
  Serial.printf("DHT Temperatura: %f °C\n", dhtTemperature);
  Serial.printf("BMP Pessão: %f Pa\n", bmpPressure);
  Serial.printf("BMP Temperatura: %f °C\n", bmpTemperature);
  Serial.printf("Altitude Aproximada: %f m\n", bmp.readAltitude(1018));
  Serial.printf("Velocidade do Vento: %f km/h\n", windSpeed);
  Serial.printf("Rajada do Vento: %f km/h\n", windGust);
  Serial.printf("Chuva: %f mm\n", rainCount);

  char windDirection[200];

  snprintf(windDirection, sizeof windDirection, "\"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\"", windDir[0], windDir[1], windDir[2], windDir[3], windDir[4], windDir[5], windDir[6], windDir[7], windDir[8], windDir[9]);

  char buf[1000];
  snprintf(buf, sizeof buf,
           "{"
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
           windGust,
           windSpeed,
           windDirection,
           solar_incidence,
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
  }
}

boolean fuzzyCompare(float compareValue, float value)
{
#define VARYVALUE 0.05

  if ((value > (compareValue * (1.0 - VARYVALUE))) && (value < (compareValue * (1.0 + VARYVALUE))))
  {
    return true;
  }
  return false;
}

float calculateDirection(float dirVoltage)
{
  if (fuzzyCompare(dirVoltage, (0.32 / 5) * Vcc))
  {
    return 112.5;
  }
  else if (fuzzyCompare(dirVoltage, (0.41 / 5) * Vcc))
  {
    return 67.5;
  }
  else if (fuzzyCompare(dirVoltage, (0.45 / 5) * Vcc))
  {
    return 90;
  }
  else if (fuzzyCompare(dirVoltage, (0.62 / 5) * Vcc))
  {
    return 157.5;
  }
  else if (fuzzyCompare(dirVoltage, (0.9 / 5) * Vcc))
  {
    return 135;
  }
  else if (fuzzyCompare(dirVoltage, (1.19 / 5) * Vcc))
  {
    return 202.5;
  }
  else if (fuzzyCompare(dirVoltage, (1.4 / 5) * Vcc))
  {
    return 180;
  }
  else if (fuzzyCompare(dirVoltage, (1.98 / 5) * Vcc))
  {
    return 22.5;
  }
  else if (fuzzyCompare(dirVoltage, (2.25 / 5) * Vcc))
  {
    return 45;
  }
  else if (fuzzyCompare(dirVoltage, (2.93 / 5) * Vcc))
  {
    return 247.5;
  }
  else if (fuzzyCompare(dirVoltage, (3.08 / 5) * Vcc))
  {
    return 225;
  }
  else if (fuzzyCompare(dirVoltage, (3.43 / 5) * Vcc))
  {
    return 337.5;
  }
  else if (fuzzyCompare(dirVoltage, (3.84 / 5) * Vcc))
  {
    return 0;
  }
  else if (fuzzyCompare(dirVoltage, (4.04 / 5) * Vcc))
  {
    return 292.5;
  }
  else if (fuzzyCompare(dirVoltage, (4.33 / 5) * Vcc))
  {
    return 315;
  }
  else
  {
    return 270;
  }
}

void readWindDir(void *z)
{
  if (windDirCount == 10)
  {
    windDirCount = 0;
  }

  float dirVoltage = ads1115.computeVolts(ads1115.readADC_SingleEnded(0));
  windDir[windDirCount] = calculateDirection(dirVoltage);
  printf("Tensao: %f | Direção: %f \n\n", dirVoltage, windDir[windDirCount]);
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
  WiFiMulti.addAP("wifi2", "senha2");

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