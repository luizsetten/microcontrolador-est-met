/*
  SDL_Weather_80422.cpp - Library for SwitchDoc Labs WeatherRack.
   SparkFun Weather Station Meters
  Argent Data Systems
  Created by SwitchDoc Labs July 27, 2014.
  Released into the public domain.
    Version 1.1 - updated constants to suppport 3.3V
    Version 1.6 - Support for ADS1015 in WeatherPiArduino Board February 7, 2015
*/

#include "Arduino.h"
#include <Time.h>
#include "SDL_Weather_80422.h"

unsigned long lastWindTime;

void serviceInterruptAnem()
{
  unsigned long currentTime = (unsigned long)(micros() - lastWindTime);

  lastWindTime = micros();
  if (currentTime > 1000) // debounce
  {
    SDL_Weather_80422::_currentWindCount++;
    if (currentTime < SDL_Weather_80422::_shortestWindTime)
    {
      SDL_Weather_80422::_shortestWindTime = currentTime;
    }
  }
}

long SDL_Weather_80422::_currentWindCount = 0;
unsigned long SDL_Weather_80422::_shortestWindTime = 0;

SDL_Weather_80422::SDL_Weather_80422(int pinAnem, int intAnem)
{
  // pinMode(pinAnem, INPUT);
  // pinMode(pinRain, INPUT);
  _pinAnem = pinAnem;

  _intAnem = intAnem;

  _currentWindCount = 0;
  _currentWindSpeed = 0.0;

  lastWindTime = 0;
  _shortestWindTime = 0xffffffff;

  _sampleTime = 5.0;
  _selectedMode = SDL_MODE_SAMPLE;

  _startSampleTime = micros();

  // set up interrupts

  pinMode(pinAnem, INPUT);     // pinAnem is input to which a switch is connected
  digitalWrite(pinAnem, HIGH); // Configure internal pull-up resistor
  attachInterrupt(_intAnem, serviceInterruptAnem, FALLING);
}

#define WIND_FACTOR 2.400
float SDL_Weather_80422::current_wind_speed() // in milliseconds
{

  if (_selectedMode == SDL_MODE_SAMPLE)
  {
    _currentWindSpeed = get_current_wind_speed_when_sampling();
  }
  else
  {
    // km/h * 1000 msec

    _currentWindCount = 0;
    delay(_sampleTime * 1000);
    _currentWindSpeed = ((float)_currentWindCount / _sampleTime) * WIND_FACTOR;
  }
  return _currentWindSpeed;
}

float SDL_Weather_80422::get_wind_gust()
{

  unsigned long latestTime;
  latestTime = _shortestWindTime;
  _shortestWindTime = 0xffffffff;
  double time = latestTime / 1000000.0; // in microseconds

  return (1 / (time)) * WIND_FACTOR / 2;
}

void SDL_Weather_80422::reset_wind_gust()
{
  _shortestWindTime = 0xffffffff;
}

// ongoing samples in wind
void SDL_Weather_80422::startWindSample(float sampleTime)
{

  _startSampleTime = micros();

  _sampleTime = sampleTime;
}

float SDL_Weather_80422::get_current_wind_speed_when_sampling()
{

  unsigned long compareValue;
  compareValue = _sampleTime * 1000000;

  if (micros() - _startSampleTime >= compareValue)
  {
    // sample time exceeded, calculate currentWindSpeed
    float _timeSpan;
    //     _timeSpan = (unsigned long)(micros() - _startSampleTime);
    _timeSpan = (micros() - _startSampleTime);

    _currentWindSpeed = ((float)_currentWindCount / (_timeSpan)) * WIND_FACTOR * 1000000;

    _currentWindCount = 0;

    _startSampleTime = micros();
  }

  return _currentWindSpeed;
}

void SDL_Weather_80422::setWindMode(int selectedMode, float sampleTime) // time in seconds
{

  _sampleTime = sampleTime;
  _selectedMode = selectedMode;

  if (_selectedMode == SDL_MODE_SAMPLE)
  {
    startWindSample(_sampleTime);
  }
}
