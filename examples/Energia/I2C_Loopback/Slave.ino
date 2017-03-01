/*
 * Copyright (c) 2017 by Stefano Speretta <s.speretta@tudelft.nl>
 *
 * DWire: a library to provide full hardware-driven I2C functionality
 * to the TI MSP432 family of microcontrollers. It is possible to use
 * this library in Energia (the Arduino port for MSP microcontrollers)
 * or in other toolchains.
 *
 * The example code in this file demonstrates the use of DWire in
 * loopback with both a master and a slave. Pin P6.5 (SCL) should be 
 * connected to P1.7 and with a pull-up (3k3) to Vcc. Pin P6.4 (SDA) 
 * should be connected to P1.6 and with a pull-up (3k3) to Vcc.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 
 * version 3, both as published by the Free Software Foundation.
 *
 */
 
#include <DWire.h>

#define PRINT_RX true

DWire slave(0);

uint8_t slaveBuffer[BUFFER_LENGTH];
unsigned char received;

void setupSlave() 
{ 
  received = 0;
  
  // Initialize I2C
  slave.begin(SLAVE_ADDRESS);
  slave.onReceive(handleReceive);
  slave.onRequest(handleRequest);

  delay(200);
}

void loopSlave() 
{
  
}

/**
 * Receive interrupt handler
 * This interrupt is triggered by DWire when a full frame has been received
 * (i.e. after receiving a STOP)
 */
void handleReceive( uint8_t numBytes ) 
{
#if PRINT_RX
  serial.print("SLAVE RX (");
  serial.print(numBytes, DEC);
  serial.print("): ");
#endif

  received = numBytes;
  
  // Get the rx buffer's contents from the DWire object
  for (unsigned short i = 0; i < numBytes; i++ ) 
  {
    slaveBuffer[i] = slave.read( );
    
#if PRINT_RX
    // Print the contents of the received byte
    serial.print(slaveBuffer[i]);
#endif
  }
  
#if PRINT_RX
  // End the line in preparation of the next receive event
  serial.println( ); 
#endif
}

/**
 * Request interrupt handler
 * This request is called on a read request from a master node.
 *
 */
void handleRequest( void ) 
{
  // Send back the data received from the master
#if PRINT_RX
  serial.print("SLAVE TX: ");
#endif
  for(unsigned short i = 0; i < received; i++)
  {
#if PRINT_RX
    serial.print(slaveBuffer[i]);
#endif
    slave.write(slaveBuffer[i]);
  }
}
