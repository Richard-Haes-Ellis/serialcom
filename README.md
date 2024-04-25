# SerialCom Library

## Overview
The SerialCom library provides a platform-independent way to communicate over serial ports. It supports both Arduino and Raspberry Pi platforms, allowing you to easily interface with serial devices.

## Features
- Platform-independent: Works seamlessly on both Arduino and Raspberry Pi.
- Simple API: Provides easy-to-use functions for sending and receiving data.
- Efficient: Optimized for performance and resource usage.

## Usage
To use the SerialCom library in your project, follow these steps:

1. Include the `serialcom.h` header file in your project.
2. Instantiate a `SerialCom` object with the appropriate parameters, depending on your platform (Arduino or Raspberry Pi).
3. Use the `sendSerialData` function to send data over the serial port.
4. Use the `readSerialData` function to receive data from the serial port.

### Example (Arduino)
```cpp
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
#include "serialcom.h"

// These structs are provided as examples for sending and recieving
#pragma pack(1)
typedef struct {
    uint8_t command;
    float setpoint1;
    float setpoint2;
} CommandPacket;

#pragma pack(1)
typedef struct {
    uint8_t status;
    float var1;
    float var2;
    float var3;
    float var4;
} TelemetryPacket;

SoftwareSerial mySerial(7, 8); // RX, TX

int main() {
  init();

  // For debugging purposes
  mySerial.begin(115200);
  
  // Initialize the serial communication
  SerialCom serialCom(&Serial, 115200);

  // Create the packets that will be sent and received
  CommandPacket rxPacket;
  TelemetryPacket txPacket;

  long int previousMillis = micros();

  while(true)
  {
    // Check if theres data available and read it 
    if(serialCom.readSerialData((uint8_t*)&rxPacket, sizeof(CommandPacket)) == 1)
    {
      // Do something with the received data to send back
      txPacket.status = rxPacket.command;
      txPacket.var1 = rxPacket.setpoint1;
      txPacket.var2 = rxPacket.setpoint2;
      txPacket.var3 = rxPacket.setpoint1*2;
      txPacket.var4 = rxPacket.setpoint2*2;

      //Print the received data
      mySerial.print("time: ");
      mySerial.print(micros()-previousMillis);
      previousMillis = micros();
      mySerial.print(" Command: ");
      mySerial.print(rxPacket.command);
      mySerial.print(" Setpoint1: ");
      mySerial.print(rxPacket.setpoint1);
      mySerial.print(" Setpoint2: ");
      mySerial.println(rxPacket.setpoint2);

    } 

    // Send telemetry data every 50ms
    if(millis() % 50 == 0)
    {
      serialCom.sendSerialData((uint8_t*)&txPacket, sizeof(TelemetryPacket));
    }

    // Rest of code here //
    
  }
}

```

### Example (Raspberry Pi)
```cpp
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <cmath>
#include "serialcom.h"

#define ELAPSED_TIME_MS(start_time) \
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - (start_time)).count()

using namespace std;


// These structs are provided as examples for sending and recieving
#pragma pack(1)
typedef struct {
    uint8_t command;
    float setpoint1;
    float setpoint2;
} CommandPacket;

#pragma pack(1)
typedef struct {
    uint8_t status;
    float var1;
    float var2;
    float var3;
    float var4;
} TelemetryPacket;

int main() {

  // Serial comunications
  SerialCom serial("/dev/ttyACM0", 115200);

  // Command and telemetry packets
  CommandPacket txPacket = {1, 1.1, 2.3};
  TelemetryPacket rxPacket;

  // Timers 
  auto start_time = std::chrono::steady_clock::now();
  auto function_time = std::chrono::steady_clock::now();

  while (true) {

    // Read data from serial port
    if(serial.readSerialData((uint8_t *)&rxPacket,sizeof(TelemetryPacket)) == 1)
    {
      
      printf("Received data: %d %f %f %f %f\n", rxPacket.status, rxPacket.var1, rxPacket.var2, rxPacket.var3, rxPacket.var4);
      
    }

    // Send data every 50 ms
    if (ELAPSED_TIME_MS(start_time) >  50)
    {
      // Create sine wave with setpoints
      txPacket.command += 1;
      txPacket.setpoint1 = 1.0 + sin(ELAPSED_TIME_MS(function_time)/1000.0);
      txPacket.setpoint2 = 1.0 + cos(ELAPSED_TIME_MS(function_time)/1000.0);

      // Send data to serial port
      serial.sendSerialData((uint8_t *)&txPacket,sizeof(CommandPacket));

      // Reset timer
      start_time = std::chrono::steady_clock::now();
    }
    
    // Rest of code here //

  }

  return 0;
}
```

## Platform Support
- Arduino: Compatible with all Arduino boards.
- Raspberry Pi: Compatible with Raspberry Pi boards running Linux.

## License
This library is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

