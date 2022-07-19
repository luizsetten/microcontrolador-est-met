/*
  SDL_Weather_80422.h - Library for Weather Sensor
  Designed for:  SwitchDoc Labs WeatherRack www.switchdoc.com
  Argent Data Systems
  SparkFun Weather Station Meters
  Created by SwitchDoc Labs July 27, 2014.
  Released into the public domain.
    Version 1.1 - updated constants to suppport 3.3V
    Version 1.6 - Support for ADS1015 in WeatherPiArduino Board February 7, 2015
*/
#ifndef SDL_Weather_80422_h
#define SDL_Weather_80422_h

// sample mode means return immediately.  THe wind speed is averaged at sampleTime or when you ask, whichever is longer
#define SDL_MODE_SAMPLE 0
// Delay mode means to wait for sampleTime and the average after that time.
#define SDL_MODE_DELAY 1


extern "C" void serviceInterruptAnem(void)  __attribute__ ((signal));
class SDL_Weather_80422
{
  public:
    SDL_Weather_80422(int pinAnem,  int intAnem);
  
    float current_wind_speed();
    float get_wind_gust();
    void reset_wind_gust();
  
    void setWindMode(int selectedMode, float sampleTime);  // time in seconds
    
    static unsigned long _shortestWindTime;
    static long _currentWindCount;
    

  friend void serviceInterruptAnem();
  
  private:
  

    int _pinAnem;  
    int _intAnem;
    float _sampleTime;
    int _selectedMode;
    
    
    unsigned long _startSampleTime;
    

    float _currentWindSpeed;

    void startWindSample(float sampleTime);
    float get_current_wind_speed_when_sampling();
};

#endif

