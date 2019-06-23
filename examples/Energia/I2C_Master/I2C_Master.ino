/*
 * Copyright (c) 2016 by Stefan van der Linden <spvdlinden@gmail.com>
 *
 * DWire: a library to provide full hardware-driven I2C functionality
 * to the TI MSP432 family of microcontrollers. It is possible to use
 * this library in Energia (the Arduino port for MSP microcontrollers)
 * or in other toolchains.
 *
 * The example code in this file demonstrates the use of DWire in
 * master mode. This is to be used together with the I2C_Slave.ino
 * example
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 
 * version 3, both as published by the Free Software Foundation.
 *
 */

#include "DWire.h"
#include "DSerial.h"

DSerial serial;

DWire wire(EUSCI_B0_BASE);
uint8_t i;

void setup( ) {
    wire.begin( );
    serial.begin( );
    
    // Give the slave some time to start up
    delay(200);
}

void loop( ) {

    // Start a frame
    wire.beginTransmission(0x42);

    // Write four bytes
    wire.write(i);
    wire.write(i + 1);
    wire.write(i + 2);
    wire.write(i + 3);
    i++;

    // Mark the frame as finished. This causes DWire to transmit the buffer's contents
    wire.endTransmission(true);

    delay(400);

    // Do a request
    if(wire.requestFrom(0x42, 4) == 4) {
      serial.println("Response: ");
      for(short i = 0; i < 4; i++)
        serial.print(wire.read(), DEC);
    }
    delay(400);
}