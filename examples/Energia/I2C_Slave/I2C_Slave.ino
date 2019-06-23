/*
 * Copyright (c) 2016 by Stefan van der Linden <spvdlinden@gmail.com>
 *
 * DWire: a library to provide full hardware-driven I2C functionality
 * to the TI MSP432 family of microcontrollers. It is possible to use
 * this library in Energia (the Arduino port for MSP microcontrollers)
 * or in other toolchains.
 *
 * The example code in this file demonstrates the use of DWire in
 * slave-mode. This is to be used together with the I2C_Master.ino
 * example
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 
 * version 3, both as published by the Free Software Foundation.
 *
 */

#include "DWire.h"
#include "DSerial.h"

// Initialise the 
DWire wire(EUSCI_B0_BASE);
DSerial serial;

uint8_t buff[4];
uint8_t i;

void setup()
{
  // Initialise the serial (COM) connection
  serial.begin( );
  delay(100);
  serial.println("Ready as slave...");

  // Start the module as a slave on address 0x42
  wire.begin(0x42);
  wire.onReceive(handleReceive);
  wire.onRequest(handleRequest);
}

void loop()
{
  // Simply don't do anything. Interrupts are driving the program
  delay(100);
}

/**
 * Receive interrupt handler
 * This interrupt is triggered by DWire when a full frame has been received
 * (i.e. after receiving a STOP)
 */
void handleReceive( uint8_t numBytes ) {

  serial.print("Got a message: ");

  // Get the rx buffer's contents from the DWire object
  for ( int i = 0; i < numBytes; i++ ) {
    buff[i] = wire.read( );

    // Print the contents of the received byte
    serial.print(buff[i]);
  }
    // End the line in preparation of the next receive event
    serial.println( );
}

/**
 * Request interrupt handler
 * This request is called on a read request from a master node.
 *
 */
void handleRequest( void ) {
  // Send back the data received from the master
  serial.println("Sending response.");
  wire.write(buff[0]);
  wire.write(buff[1]);
  wire.write(buff[2]);
  wire.write(buff[3]);
}
