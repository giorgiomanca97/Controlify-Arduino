#include "Controlify.h"


// ==================================================
// SerialComm
// ==================================================

#if defined(UNO) || defined(MEGA) || defined(GIGA)

HardwareSerial* SerialComm::port(uint8_t channel) {
  switch(channel){
    case 0:
      return &Serial;
    #if defined(MEGA) || defined(GIGA)
    case 1:
      return &Serial1;
    case 2:
      return &Serial2;
    case 3:
      return &Serial3;
    #endif
    default:
      return &Serial;
  }
}

void SerialComm::start(HardwareSerial* hwserial, uint32_t baudrate, uint8_t config) {
  hwserial->begin(baudrate, config);
  hwserial->flush();
}

void SerialComm::start(uint8_t channel, uint32_t baudrate, uint8_t config) {
  start(port(channel), baudrate, config);
}

void SerialComm::close(HardwareSerial* hwserial) {
  hwserial->flush();
  hwserial->end();
}

void SerialComm::close(uint8_t channel) {
  close(port(channel));
}

#endif