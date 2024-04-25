#pragma once

#if defined ( ARDUINO )
#include <Arduino.h>
#else
#include <stdint.h>
#include <stddef.h>
#define RASPBERRY_PI
#endif

enum ComsState {
    START_PACKET,
    READ_SIZE_PACKET,
    READ_PAYLOAD,
    CHECKSUM
};

class SerialCom {
    public:
        #ifdef ARDUINO
        SerialCom(HardwareSerial *serial, long int baudrate);
        #elif defined( RASPBERRY_PI )
        SerialCom(const char *serialPort, long int baudrate);
        ~SerialCom();
        #endif
        void sendSerialData(const uint8_t * data, size_t size);
        int readSerialData(uint8_t * data, size_t size);
        int readSerial(uint8_t *data, size_t size);
        int writeSerial(const uint8_t *data, size_t size);
        int available();
        void wait(unsigned long us);
    private:
        #ifdef RASPBERRY_PI
        int fd;
        #elif defined( ARDUINO )
        HardwareSerial * _hardwareSerial;
        #endif
        long int _baudRate;
};

