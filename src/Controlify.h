#ifndef CONTROLIFY_H
#define CONTROLIFY_H


#if defined(ARDUINO_ARCH_MBED_GIGA)
#define GIGA
#endif

#if defined(ARDUINO_AVR_MEGA2560)
#define MEGA
#endif

#if defined(ARDUINO_AVR_UNO)
#define UNO
#endif

#define UNUSED(arg) (void)(arg)

#define PIN_CONTROL_EXTRA_FEATURES
#define PIN_CONTROL_STORE_VALUES
#define PIN_MEASURE_EXTRA_FEATURES
#define PIN_MEASURE_STORE_VALUES

#include <inttypes.h>
#include <limits.h>
#include <math.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>


// ==================================================
// Utils
// ==================================================

float remap(float v, float a1, float b1, float a2, float b2, bool clamp = false);
float remap( long v,  long a1,  long b1, float a2, float b2, bool clamp = false);

 long remap(float v, float a1, float b1,  long a2,  long b2, bool clamp = false);
 long remap( long v,  long a1,  long b1,  long a2,  long b2, bool clamp = false);

void byteToHex(const uint8_t & byte, char & hhex, char & lhex);
void nibbleToHex(const uint8_t & nibble, char & hex);

class Timer{
public:
  Timer();
  Timer(unsigned long delta);
  Timer(unsigned long delta, unsigned long time);

  void setup(unsigned long delta);
  void reset(unsigned long time);
  bool check(unsigned long time);

private:
  unsigned long time;
  unsigned long delta;
};


// ==================================================
// Control
// ==================================================

class Integrator final
{
public:
  Integrator() {}
  ~Integrator() {}

  void init(float time_sampling);
  void reset();
  void reset(float x);
  void input(float u);
  void step();
  float output();
  float evolve(float u);
  
private:
  float ts;
  float x;
  float u;
};


class Filter final
{
public:
  void init(float time_sampling, float tau);
  void reset();
  void reset(float x);
  void input(float u);
  void step();
  float output();
  float evolve(float u);

private:
  float u;
  float x;

  float A = 0.0;
  float B = 0.0;
  float C = 0.0;
};


class PID final
{
public:
  void init(float time_sampling, 
    float err_deadzone, 
    float int_sat, float int_reset_err_thr, float int_reset_div, float int_reset_val, 
    float der_filter_pole, 
    bool bumpless);
  void setup(float kp, float ki, float kd);
  void reset();
  void reset(float xi, float xd);
  void input(float e);
  void step();
  float output();
  float evolve(float e);

private:
  void apply_saturation();

  float ts = 0.0;
  float err_deadzone;
  float int_sat;
  float int_rst_thr;
  float int_rst_div;
  float int_rst_val;
  float der_pole;
  bool bumpless = false;

  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;
  
  float e = 0.0;
  float xi = 0.0;
  float xd = 0.0;
  
  float A = 0.0;
  float B = 0.0;
  float C = 0.0;
  float D = 0.0;
};


// ==================================================
// Pin
// ==================================================

class PinControl {
public:
  PinControl(uint8_t pin);
  PinControl(uint8_t pin, float v1, float v2);

  uint8_t getPin();

  void setLimits(float v1, float v2);

  void set(bool state);
  void pwm(uint8_t pwm);
  void control(float value);

  #if defined(PIN_CONTROL_EXTRA_FEATURES)
  void feedback(float error);
  void feedback();

  void setPID(PID *pid);
  PID* getPID();
  #endif

  #if defined(PIN_CONTROL_STORE_VALUES)
  bool last_set();
  uint8_t last_pwm();
  float last_control();
  #endif

private:
  uint8_t pin;
  float v1;
  float v2;

  #if defined(PIN_CONTROL_EXTRA_FEATURES)
  PID *pid;
  #endif

  #if defined(PIN_CONTROL_STORE_VALUES)
  bool set_;
  uint8_t pwm_;
  #endif
};


class PinMeasure {
public:
  PinMeasure(uint8_t pin, bool pullup = false);
  PinMeasure(uint8_t pin, float v1, float v2, bool pullup = false);
  
  uint8_t getPin();
  
  void setLimits(float v1, float v2);

  bool state();
  uint16_t value();
  float measure();

  #if defined(PIN_MEASURE_EXTRA_FEATURES)
  float filter(bool readonly);
  float filter();

  void setFilter(Filter *filter);
  Filter* getFilter();
  #endif

  #if defined(PIN_MEASURE_STORE_VALUES)
  bool last_state();
  uint16_t last_value();
  float last_measure();
  #endif

private:
  uint8_t pin;
  float v1;
  float v2;

  #if defined(PIN_MEASURE_EXTRA_FEATURES)
  Filter* fil;
  #endif

  #if defined(PIN_MEASURE_STORE_VALUES)
  bool state_;
  uint16_t value_;
  #endif
};


// ==================================================
// PWM
// ==================================================

#if defined(UNO) || defined(MEGA)
class PWMfreq {
public:
  PWMfreq() = delete;
  ~PWMfreq() = delete;

#if defined(UNO)
  enum class UnoTimer0 : unsigned char{   // D5 & D6
    FREQ_62500_00 = 0b00000001, // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
    FREQ_7812_50  = 0b00000010, // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
    FREQ_976_56   = 0b00000011, // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (DEFAULT)
    FREQ_244_14   = 0b00000100, // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
    FREQ_61_04    = 0b00000101  // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
  };

  enum class UnoTimer1 : unsigned char{   // D9 & D10 
    FREQ_31372_55 = 0b00000001, // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class UnoTimer2 : unsigned char{   // D3 & D11
    FREQ_31372_55 = 0b00000001, // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_980_39   = 0b00000011, // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
    FREQ_490_20   = 0b00000100, // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_245_10   = 0b00000101, // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
    FREQ_122_55   = 0b00000110, // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000111  // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  };
#endif

#if defined(MEGA)
  enum class MegaTimer0 : unsigned char{   // D4 & D13
    FREQ_62500_00 = 0b00000001, // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
    FREQ_7812_50  = 0b00000010, // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
    FREQ_976_56   = 0b00000011, // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (DEFAULT)
    FREQ_244_14   = 0b00000100, // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
    FREQ_61_04    = 0b00000101  // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
  };

  enum class MegaTimer1 : unsigned char{   // D11 & D12 
    FREQ_31372_55 = 0b00000001, // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class MegaTimer2 : unsigned char{   // D9 & D10
    FREQ_31372_55 = 0b00000001, // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_980_39   = 0b00000011, // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
    FREQ_490_20   = 0b00000100, // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_245_10   = 0b00000101, // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
    FREQ_122_55   = 0b00000110, // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000111  // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class MegaTimer3 : unsigned char{   // D2, D3 & D5 
    FREQ_31372_55 = 0b00000001, // set timer 3 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 3 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 3 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 3 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 3 divisor to  1024 for PWM frequency of    30.64 Hz
  };  

  enum class MegaTimer4 : unsigned char{   // D6, D7 & D8 
    FREQ_31372_55 = 0b00000001, // set timer 4 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 4 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 4 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class MegaTimer5 : unsigned char{   // D44, D45 & D46 
    FREQ_31372_55 = 0b00000001, // set timer 5 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 5 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 5 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 5 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 5 divisor to  1024 for PWM frequency of    30.64 Hz
  };
#endif

  //NOTE: Changing timer 0 affects millis() and delay!

#if defined(UNO)
  static void set(UnoTimer0 freq);
  static void set(UnoTimer1 freq);
  static void set(UnoTimer2 freq);
#endif

#if defined(MEGA)
  static void set(MegaTimer0 freq);
  static void set(MegaTimer1 freq);
  static void set(MegaTimer2 freq);
  static void set(MegaTimer3 freq);
  static void set(MegaTimer4 freq);
  static void set(MegaTimer5 freq);
#endif
};
#endif


#endif //CONTROLIFY_H