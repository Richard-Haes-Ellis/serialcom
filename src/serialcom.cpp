#include "serialcom.h"
#if defined( ARDUINO )
#include <Arduino.h>
#elif defined( RASPBERRY_PI )
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#endif


#if defined ( ARDUINO )
#pragma message("Compiling for Arduino")
#elif defined ( RASPBERRY_PI )
#pragma message("Compiling for Raspberry Pi")
#endif

SerialCom::SerialCom() {
  // Default constructor
}


#if defined( ARDUINO )
void SerialCom::begin(HardwareSerial* serial,long int baudrate){
    _hardwareSerial = serial;
    _baudRate = baudrate;
    // Initialize serial communication with the specified baud rate
    _hardwareSerial->begin(_baudRate);
}
#elif defined( RASPBERRY_PI )
void SerialCom::begin(const char *serialPort,long int baudrate){
  _baudRate = baudrate;
  // Open serial port
  this->fd = open(serialPort, O_RDWR | O_NOCTTY | O_NDELAY);
  if (this->fd == -1) {
    perror("Error opening serial port");
    exit(1);
  }

  // Configure serial port
  termios options;
  cfmakeraw(&options);

  // Set baud rate
  if(_baudRate == 9600) {
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
  } else if(_baudRate == 115200) {
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
  } else {
    printf("Unsupported baud rate\n");
    exit(1);
  }

  // Enable the receiver and set local mode
  options.c_cflag |= (CLOCAL | CREAD);

  // Set data bits to 8 and disable parity
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  options.c_iflag &= ~(IXOFF | IXANY);

  // set vtime, vmin, baud rate...
  options.c_cc[VMIN] = 0;  // you likely don't want to change this
  options.c_cc[VTIME] = 0; // or this

  // Apply the new serial port options
  if (tcsetattr(this->fd, TCSANOW, &options) != 0) {
    printf("Error setting serial port attributes\n");
    exit(1);
  }

  // Flush input and output buffers
  if (tcflush(this->fd, TCIOFLUSH) != 0) {
    printf("Error flushing serial port\n");
    exit(1); 
  }
}
#endif

#if defined( RASPBERRY_PI )
SerialCom::~SerialCom() {
  close(fd);
}
#endif

int SerialCom::readSerialData(uint8_t *data, size_t size) {
  if((unsigned int) SerialCom::available() < size + 3) {
    return -1; // No data available yet
  }

  // Calculate timout for 1 byte at current baud rate
  // 1 start bit + 8 data bits + 1 stop bit = 10 bits
  // unsigned long timeout_microseconds = ((10 * 1000000L) / this->baudRate)*1.1; // 10% margin 

  // Read start of packet marker
  const uint8_t SOP_MARKER = 0xFF;
  uint8_t sop = 0x00;

  if(SerialCom::readSerial(&sop, 1) != 1 || sop != SOP_MARKER) {
		return -2; // Failed to read start of packet marker or marker mismatch
  }
  
  // Read data length
  uint8_t dataSize;

  if(SerialCom::readSerial(&dataSize, 1) != 1 || dataSize > size) {
    return -3; // Failed to read data length or data exceeds buffer size
    
  }


  // Check if enough data is available
  if(SerialCom::available() < dataSize + 1) {
      return -4; // Not enough data available
  }

  // Read data
  if(SerialCom::readSerial(data, dataSize) != dataSize) {
		return -5; // Failed to read data byte
	}

  // Read checksum
  uint8_t checksum;
  if(SerialCom::readSerial(&checksum, 1) != 1) {
    return -6; // Failed to read checksum
  }

  // Verify checksum
  uint8_t computedChecksum = 0;
  const uint8_t *byteData = reinterpret_cast<const uint8_t *>(data);
  for (size_t i = 0; i < dataSize; ++i) {
    computedChecksum += byteData[i];
  }
  if (computedChecksum != checksum) {
    return -7; // Checksum mismatch
  }

  return true; // Packet received successfully
}

void SerialCom::sendSerialData(const uint8_t * data, size_t size) {
  uint8_t checksum = 0;
  const uint8_t SOP_MARKER = 0xFF;
  #if defined( ARDUINO )
  _hardwareSerial->write(SOP_MARKER);
  _hardwareSerial->write(size);
  for (size_t i = 0; i < size; i++){
    _hardwareSerial->write(data[i]);
    checksum += data[i];
  }
  _hardwareSerial->write(checksum);
  _hardwareSerial->flush();
  #elif defined( RASPBERRY_PI )
  write(fd, &SOP_MARKER, 1);
  write(fd, &size, 1);
  for (size_t i = 0; i < size; i++) {
    write(fd, &data[i], 1);
    checksum += data[i];
  }
  write(fd, &checksum, 1);
  #endif
}


int SerialCom::readSerial(uint8_t *data, size_t size) {
  #if defined( ARDUINO )
  return _hardwareSerial->readBytes(data, size);
  #elif defined( RASPBERRY_PI )
  return read(fd, data, size);
  #endif
}

int SerialCom::available(void) {
  #if defined( ARDUINO )
  return _hardwareSerial->available();
  #elif defined( RASPBERRY_PI )
  int nread;
  ioctl(fd, FIONREAD, &nread);
  return nread;
  #endif
}

int SerialCom::writeSerial(const uint8_t *data, size_t size) {
  #if defined( ARDUINO )
  return _hardwareSerial->write(data, size);
  #elif defined( RASPBERRY_PI )
  return write(fd, data, size);
  #endif
}

void SerialCom::wait(unsigned long us) {
  #if defined( ARDUINO )
  delayMicroseconds(us);
  #elif defined( RASPBERRY_PI )
  usleep(us);
  #endif
}

void SerialCom::closeSerial(void) {
  #if defined( RASPBERRY_PI )
  close(fd);
  #endif
}