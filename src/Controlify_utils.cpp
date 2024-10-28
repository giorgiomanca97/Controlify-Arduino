#include "Controlify.h"


// ==================================================
// Functions
// ==================================================

float remap(float v, float a1, float b1, float a2, float b2, bool clamp) {
  float delta = (v - a1) / (b1 - a1);
  if(clamp) {
    delta = fmin(fmax(delta, 0.0f), 1.0f);
  }
  return a2 + delta * (b2 - a2);
}

float remap( long v,  long a1,  long b1, float a2, float b2, bool clamp){
  return remap((float) v, (float) a1, (float) b1, a2, b2, clamp);
}

long remap(float v, float a1, float b1,  long a2,  long b2, bool clamp){
  return round(remap(v, a1, b1, (float) a2, (float) b2, clamp));
}
long remap( long v,  long a1,  long b1,  long a2,  long b2, bool clamp){
  return round(remap((float) v, (float) a1, (float) b1, (float) a2, (float) b2, clamp));
}

uint32_t remap(float v, float a, float b, uint8_t bits) {
  uint32_t val = ((1ul << bits) - 1ul);
  float delta = fmin(fmax((v - a) / (b - a), 0.0f), 1.0f);
  return delta * val + 0.5f;
}

float remap(uint32_t v, uint8_t bits, float a, float b) {
  uint32_t val = ((1ul << bits) - 1ul);
  float delta = ((float) v) / ((float) val);
  return a + delta * (b - a);
}

void byteToHex(const uint8_t &byte, char &hhex, char &lhex) {
  nibbleToHex((byte & 0b00001111) >> 0, lhex);
  nibbleToHex((byte & 0b11110000) >> 4, hhex);
}

void nibbleToHex(const uint8_t &nibble, char &hex) {
  switch(nibble) {
    case 0u:
      hex = '0';
      break;
    case 1u:
      hex = '1';
      break;
    case 2u:
      hex = '2';
      break;
    case 3u:
      hex = '3';
      break;
    case 4u:
      hex = '4';
      break;
    case 5u:
      hex = '5';
      break;
    case 6u:
      hex = '6';
      break;
    case 7u:
      hex = '7';
      break;
    case 8u:
      hex = '8';
      break;
    case 9u:
      hex = '9';
      break;
    case 10u:
      hex = 'A';
      break;
    case 11u:
      hex = 'B';
      break;
    case 12u:
      hex = 'C';
      break;
    case 13u:
      hex = 'D';
      break;
    case 14u:
      hex = 'E';
      break;
    case 15u:
      hex = 'F';
      break;
  }
}

float rad2deg(float rad){
  return rad / PI * 180.0f;
}

float deg2rad(float deg){
  return deg / 180.0f * PI;
}


// ==================================================
// Timer
// ==================================================

Timer::Timer(){
  this->delta = 0;
  this->time = 0;
}

Timer::Timer(unsigned long delta){
  this->delta = delta;
  this->time = 0;
}

Timer::Timer(unsigned long delta, unsigned long time){
  this->delta = delta;
  this->time = time;
}

void Timer::setup(unsigned long delta){
  this->delta = delta;
}

void Timer::reset(unsigned long time){
  this->time = time;
}

bool Timer::check(unsigned long time){
  return diff(time) >= delta;
}

unsigned long Timer::cycle(unsigned long time, bool regular){
  unsigned long dt = diff(time);
  unsigned long count = dt/delta;

  if(count) {
    if(regular) {
      this->time += count * delta;
    } else {
      this->time = time;
    }
  }

  return count;
}

unsigned long Timer::diff(unsigned long time){
  unsigned long dt;

  if(time < this->time){
    dt = (0UL - 1UL) - this->time;
    dt = time + dt + 1;
  } else {
    dt = time - this->time;
  }

  return dt;
}