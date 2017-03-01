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
#include <DSerial.h>
#include <I2CScanner.h>

#define SLAVE_ADDRESS 0x42
#define BUFFER_LENGTH 4

unsigned char TXbuffer[BUFFER_LENGTH];
DWire master(1);
DSerial serial;
uint8_t character;

void deviceFound(unsigned char device)
{
  serial.print("Found ");
  serial.print(device, HEX);
  serial.println();
}

void setup() 
{
  // initialize the UART
  serial.begin();

  // Initialize I2C master
  master.setFastMode();
  master.begin();

  character = '0';
  
  delay(200);
}

void send()
{
  // Start a frame
  master.beginTransmission(SLAVE_ADDRESS);

  serial.print("MASTER TX: ");

  for (unsigned short i = 0; i < BUFFER_LENGTH; i++)
  {
    TXbuffer[i] = character;
  }

  for (unsigned short i = 0; i < BUFFER_LENGTH; i++)
  {
    serial.print(TXbuffer[i]);
  }
  serial.println();

  for (unsigned short i = 0; i < BUFFER_LENGTH; i++)
  {
    master.write(TXbuffer[i]);
  }

  character++;
  // limit i within printable characters
  if (character > 'z')
  {
    character = '0';
  }

  unsigned char r = master.endTransmission();
  if (r)
  {
    serial.println("TX FAILURE");
  }
  else
  {
    serial.println("TX SUCCESS");
  }
}

void request()
{
  serial.print("Request: ");
  if(master.requestFrom(SLAVE_ADDRESS, BUFFER_LENGTH) == BUFFER_LENGTH) 
  {
    serial.print("MASTER RX: ");
    for(unsigned short k = 0; k < BUFFER_LENGTH; k++)
    {
      serial.print(master.read());
    }
    serial.println();
  }
  else
  {
    serial.println("Request FAILURE");
  }
}

void requestWithWrite()
{
  // Do a request with a write (use repeated start)
  serial.print("Request with repeated start: ");
  master.beginTransmission(SLAVE_ADDRESS);
  master.write('!');
  if(master.requestFrom(SLAVE_ADDRESS, BUFFER_LENGTH) == BUFFER_LENGTH) 
  {
    serial.print("MASTER RX: ");
    for(unsigned short k = 0; k < BUFFER_LENGTH; k++)
    {
      serial.print(master.read());
    }
    serial.println();
  }
  else
  {
    serial.println("Request FAILURE");
  }
}

void loop() 
{
  serial.println();
  send();

  serial.println();
  serial.println("Scanning...");
  I2CScanner::scan(master, deviceFound);
  serial.println("Scan done.");

  serial.println();
  request();

  serial.println();
  requestWithWrite();
  
  delay(400);
}

