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

    mySerial.begin(115200);
    
    SerialCom serialCom(&Serial, 115200);

    CommandPacket rxPacket;
    TelemetryPacket txPacket;

    while(true) {

        if(serialCom.readSerialData((uint8_t*)&rxPacket, sizeof(CommandPacket)) == 1)
        {
            // print the received data
            mySerial.print("Command: ");
            mySerial.print(rxPacket.command);
            mySerial.print(" Setpoint1: ");
            mySerial.print(rxPacket.setpoint1);
            mySerial.print(" Setpoint2: ");
            mySerial.println(rxPacket.setpoint2);
        }

        // Send sine wave in telemetry data
        txPacket.var1 = 1.0 + sin(millis() / 1000.0);
        txPacket.var2 = 2.0 + cos(millis() / 1000.0);
        txPacket.var3 = 3.0 + sin(millis() / 1000.0);
        txPacket.var4 = 4.0 + cos(millis() / 1000.0);

        serialCom.sendSerialData((uint8_t*)&txPacket, sizeof(TelemetryPacket));

        delay(20);
    }

    return 0;
}

```

### Example (Raspberry Pi)
```cpp
#include <iostream>
#include <unistd.h>
#include "serialcom.h"

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
    // Serial communications
    SerialCom serial("/dev/ttyACM0", 115200);

    // Command and telemetry packets
    CommandPacket txPacket = {3, 1.5, 3.2};
    TelemetryPacket rxPacket;

    while (true) {
        // Read data from serial port
        if (serial.readSerialData(reinterpret_cast<uint8_t*>(&rxPacket), sizeof(TelemetryPacket)) == 1) {
            // Do something with rxPacket
            // rxPacket.status ..
        }

        // Send data
        serial.sendSerialData((uint8_t *)&txPacket,sizeof(CommandPacket));

        // Sleep for 20 ms
        usleep(20000);
    }

    return 0;
}
```

## Platform Support
- Arduino: Compatible with all Arduino boards.
- Raspberry Pi: Compatible with Raspberry Pi boards running Linux.

## License
This library is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

