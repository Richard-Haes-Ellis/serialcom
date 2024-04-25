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
        void begin(HardwareSerial *serial, long int baudrate);
        #elif defined( RASPBERRY_PI )
        void begin(const char *serialPort, long int baudrate);
        ~SerialCom();
        #endif
        SerialCom(); // Default constructor
        void sendSerialData(const uint8_t * data, uint8_t size);
        int recieveSerialData(uint8_t * data, uint8_t size);
        int readSerial(uint8_t *data, uint8_t size);
        int writeSerial(const uint8_t *data, uint8_t size);
        int available(void);
        void wait(unsigned long us);
        void closeSerial(void);
    private:
        #ifdef RASPBERRY_PI
        int fd;
        #elif defined( ARDUINO )
        HardwareSerial * _hardwareSerial;
        #endif
        long int _baudRate;
};

