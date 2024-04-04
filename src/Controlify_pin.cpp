#include "Controlify.h"


// ==================================================
// PinControl
// ==================================================

PinControl::PinControl(uint8_t pin){
  this->pin = pin;
  pinMode(pin, OUTPUT);
  setLimits(0.0f, 1.0f);
}

PinControl::PinControl(uint8_t pin, float v1, float v2){
  this->pin = pin;
  pinMode(pin, OUTPUT);
  setLimits(v1, v2);
}

void PinControl::setResolution(uint8_t bits) {
  if(bits >= 1 && bits <= 32) {
    bres = bits; 
  }
}

uint8_t PinControl::getPin(){
  return pin;
}

void PinControl::setLimits(float v1, float v2){
  if(v2 > v1) {
    this->v1 = v1;
    this->v2 = v2;
  }
}

void PinControl::setInverted(bool inv){
  this->inv = inv;
}

void PinControl::digitalW(bool value){
  #if defined(PIN_CONTROL_STORE_VALUES)
  d = value;
  a = value ? ((1l << bres) - 1l) : 0ul;
  #endif
  digitalWrite(pin, value ? HIGH : LOW);
}

void PinControl::analogW(uint32_t value){
  #if defined(PIN_CONTROL_STORE_VALUES)
  d = value;
  a = value >= ((1lu << (bres - 1ul)));
  #endif
  analogWrite(pin, value);
}

void PinControl::control(float value){
  analogW(remap(value, v1, v2, 0l, ((1l << bres) - 1l), true));
}

#if defined(PIN_CONTROL_EXTRA_FEATURES)
void PinControl::feedback(float error){
  if(pid != NULL) control(pid->evolve(error));
}

void PinControl::feedback(){
  if(pid != NULL) control(pid->output());
}

void PinControl::setPID(PID *pid){
  this->pid = pid;
}

PID* PinControl::getPID(){
  return pid;
}
#endif

#if defined(PIN_CONTROL_STORE_VALUES)
bool PinControl::lastDigital(){
  return d;
}

uint32_t PinControl::lastAnalog(){
  return a;
}

float PinControl::lastControl(){
  return remap(a, 0l, ((1l << bres) - 1l), v1, v2);
}
#endif


// ==================================================
// PinMeasure
// ==================================================

PinMeasure::PinMeasure(uint8_t pin, bool pullup){
  this->pin = pin;
  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
  setLimits(0.0f, 1.0f);
}

PinMeasure::PinMeasure(uint8_t pin, float v1, float v2, bool pullup){
  this->pin = pin;
  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
  setLimits(v1, v2);
}

void PinMeasure::setResolution(uint8_t bits) {
  if(bits >= 1 && bits <= 32) {
    bres = bits; 
  }
}

uint8_t PinMeasure::getPin(){
  return this->pin;
}

void PinMeasure::setLimits(float v1, float v2){
  if(v2 > v1) {
    this->v1 = v1;
    this->v2 = v2;
  }
}

void PinMeasure::setInverted(bool inv){
  this->inv = inv;
}

bool PinMeasure::digitalR(){
  #if defined(PIN_MEASURE_STORE_VALUES)
  d = digitalRead(pin) == HIGH;
  a = d ? ((1lu << bres) - 1ul) : 0ul;
  return d;
  #else
  return digitalRead(pin) == HIGH;
  #endif
}

uint32_t PinMeasure::analogR(){
  #if defined(PIN_MEASURE_STORE_VALUES)
  a = analogRead(pin);
  d = a >= ((1lu << (bres - 1ul)));
  return a;
  #else
  return analogRead(pin);
  #endif
}

float PinMeasure::measure(){
  return remap((long) analogR(), 0l, ((1l << bres) - 1l), v1, v2);
}

#if defined(PIN_MEASURE_EXTRA_FEATURES)
float PinMeasure::filter(bool readonly){
  return ((fil != NULL) ? (readonly ? fil->output() : fil->evolve(measure())) : measure());
}

float PinMeasure::filter(){
  return ((fil != NULL) ? fil->evolve(measure()) : measure());
}

void PinMeasure::setFilter(Filter *filter){
  this->fil = filter;
}

Filter* PinMeasure::getFilter(){
  return this->fil;
}
#endif

#if defined(PIN_MEASURE_STORE_VALUES)
bool PinMeasure::lastDigital(){
  return d;
}

uint32_t PinMeasure::lastAnalog(){
  return a;
}

float PinMeasure::lastMeasure(){
  return remap((long) a, 0l, ((1l << bres) - 1l), v1, v2);
}
#endif