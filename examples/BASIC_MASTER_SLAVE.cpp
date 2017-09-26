/*
 * Copyright (c) 2016 by Stefan van der Linden <spvdlinden@gmail.com>
 *
 * DWire: a library to provide full hardware-driven I2C functionality
 * to the TI MSP432 family of microcontrollers. It is possible to use
 * this library in Energia (the Arduino port for MSP microcontrollers)
 * or in other toolchains.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 
 * version 3, both as published by the Free Software Foundation.
 *
 */

// Comment out/#undef the next line if the code is to be used as a slave setup
#define I2C_MASTER

/* Custom Includes */
#include "DWire.h"
#include "DSerial.h"

DWire* wire;
DSerial * serial;

uint_fast8_t i = 0;

void handleReceive( uint8_t );
void handleRequest( void );

int main( void ) {
    /* Disabling the Watchdog */
    MAP_WDT_A_holdTimer( );

    /* Setting DCO to 48MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

    // Start the serial port
    serial = new DSerial( );
    serial->begin( );
    serial->println("Starting...");

    // Create a new instance of DWire on the EUSCI_B1 module
    wire = new DWire( 1 );

// The code for the master setup
#ifdef I2C_MASTER
    wire->begin( );
    serial->println("Ready!");

    // Loop through all ASCII values and write this to the slave

    while ( 1 ) {

        // Start a frame
        wire->beginTransmission(0x42);

        // Write three bytes
        wire->write(i);
        wire->write(i + 1);
        wire->write(i + 2);
        i++;
        serial->println("Sending message");

        // Mark the frame as finished. This causes DWire to transmit the buffer's contents
        wire->endTransmission( );

        // Wait a while until transmitting the next frame
        for ( int ii = 0; ii < 50000; ii++ )
            ;

        serial->println("Requesting data...");
        uint8_t dataSize = wire->requestFrom(0x42, 6);

        serial->print("Got: ");
        for ( int j = 0; j < dataSize; j++ ) {
            serial->print(wire->read());
        }
        serial->println( );

        for ( int ii = 0; ii < 5000000; ii++ )
            ;
    }

// slave code
#else
    // Start the module as a slave on address 0x48
    wire->begin(0x42);
    wire->onReceive(handleReceive);
    wire->onRequest(handleRequest);
    serial->println("Ready as slave.");

    // Loop, with interrupts running the different parts of the slave
    while ( 1 )
    ;
#endif
}

/**
 * Receive interrupt handler
 * This interrupt is triggered by DWire when a full frame has been received
 * (i.e. after receiving a STOP)
 */
void handleReceive( uint8_t numBytes ) {

    serial->print("Got a message: ");

    // Get the rx buffer's contents from the DWire object
    for ( int i = 0; i < numBytes; i++ ) {
        uint8_t byte = wire->read( );

        // Print the contents of the received byte
        serial->print(byte);
    }

    // End the line in preparation of the next receive event
    serial->println( );
}

/**
 * Request interrupt handler
 * This request is called on a read request from a master node.
 *
 */
void handleRequest( void ) {
    serial->println("Sending six bytes.");
    wire->write(0x41);
    wire->write(0x42);
    wire->write(0x43);
    wire->write(0x44);
    wire->write(0x45);
    wire->write(0x46);
}
