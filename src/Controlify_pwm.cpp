#include "Controlify.h"


// ==================================================
// PWMfreq
// ==================================================

#if defined(ARDUINO_AVR_UNO)
void PWMfreq::set(UnoTimer0 freq){
  TCCR0B = (TCCR0B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(UnoTimer1 freq){
  TCCR1B = (TCCR1B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(UnoTimer2 freq){
  TCCR2B = (TCCR2B & 0b11111000) | ((uint8_t) freq);
}
#endif

#if defined(ARDUINO_AVR_MEGA2560)
void PWMfreq::set(MegaTimer0 freq){
  TCCR0B = (TCCR0B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(MegaTimer1 freq){
  TCCR1B = (TCCR1B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(MegaTimer2 freq){
  TCCR2B = (TCCR2B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(MegaTimer3 freq){
  TCCR3B = (TCCR3B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(MegaTimer4 freq){
  TCCR4B = (TCCR4B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(MegaTimer5 freq){
  TCCR5B = (TCCR5B & 0b11111000) | ((uint8_t) freq);
}
#endif