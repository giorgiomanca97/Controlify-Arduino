#include "Controlify.h"


// ==================================================
// PinControl
// ==================================================

PinControl::PinControl(uint8_t pin){
  this->pin = pin;
  pinMode(pin, OUTPUT);
  setLimits(0.0, 0.0);
}

PinControl::PinControl(uint8_t pin, float v1, float v2){
  this->pin = pin;
  pinMode(pin, OUTPUT);
  setLimits(v1, v2);
}

uint8_t PinControl::getPin(){
  return this->pin;
}

void PinControl::setLimits(float v1, float v2){
  this->v1 = v1;
  this->v2 = v2;
}

void PinControl::set(bool state){
  #if defined(PIN_CONTROL_STORE_VALUES)
  this->set_ = state;
  #endif
  digitalWrite(pin, state ? HIGH : LOW);
}

void PinControl::pwm(uint8_t pwm){
  #if defined(PIN_CONTROL_STORE_VALUES)
  this->pwm_ = pwm;
  #endif
  analogWrite(pin, pwm);
}

void PinControl::control(float value){
  pwm(remap(value, v1, v2, 0l, 255l, true));
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
  return this->pid;
}
#endif

#if defined(PIN_CONTROL_STORE_VALUES)
bool PinControl::last_set(){
  return this->set_;
}

uint8_t PinControl::last_pwm(){
  return this->pwm_;
}

float PinControl::last_control(){
  return remap(this->pwm_, 0l, 255l, v1, v2);
}
#endif


// ==================================================
// PinMeasure
// ==================================================

PinMeasure::PinMeasure(uint8_t pin, bool pullup){
  this->pin = pin;
  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
  setLimits(0.0, 0.0);
}

PinMeasure::PinMeasure(uint8_t pin, float v1, float v2, bool pullup){
  this->pin = pin;
  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
  setLimits(v1, v2);
}

uint8_t PinMeasure::getPin(){
  return this->pin;
}

void PinMeasure::setLimits(float v1, float v2){
  this->v1 = v1;
  this->v2 = v2;
}

bool PinMeasure::state(){
  #if defined(PIN_MEASURE_STORE_VALUES)
  this->state_ = digitalRead(pin) == HIGH;
  return this->state_;
  #else
  return digitalRead(pin) == HIGH;
  #endif
}

uint16_t PinMeasure::value(){
  #if defined(PIN_MEASURE_STORE_VALUES)
  this->value_ = analogRead(pin);
  return this->value_;
  #else
  return analogRead(pin);
  #endif
}

float PinMeasure::measure(){
  return remap((long) value(), 0l, 1023l, v1, v2);
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
bool PinMeasure::last_state(){
  return this->state_;
}

uint16_t PinMeasure::last_value(){
  return this->value_;
}

float PinMeasure::last_measure(){
  return remap((long) this->value_, 0l, 1023l, v1, v2);
}
#endif