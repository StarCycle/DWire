/*
 * Copyright (c) 2016 by Stefan van der Linden <spvdlinden@gmail.com>
 *
 * DSerial: Serial library to provide Energia-like UART functionality
 * to non-Energia projects
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 
 * version 3, both as published by the Free Software Foundation.
 *
 */

/**** INCLUDES ****/

extern "C" {
#include<string.h>
}

#include <DSerial.h>

/**** PROTOTYPES ****/
void itoa( char *, uint8_t, uint32_t, uint8_t );


/**** GLOBAL VARIABLES ****/


/**** CONSTRUCTORS ****/
DSerial::DSerial( void ) {
    // Nothing
}

/**** PUBLIC METHODS ****/
void DSerial::begin( void ) {
    // Customise this to give module, baud rate, etc...
    /* Halting WDT  */
    MAP_WDT_A_holdTimer( );

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
    GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Setting DCO to 48MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);

}

/**
 * Transmit a single byte over the UART
 */
void DSerial::print( uint_fast8_t byte ) {
    MAP_UART_transmitData(EUSCI_A0_BASE, byte);
}

/**
 * Print a string over the UART
 */
void DSerial::print( const char * text ) {

    for ( int ii = 0; ii < strlen(text); ii++ ) {
        print(text[ii]);
    }
}

/**
 * Formats a number according to the type specified
 * Currently only integers are supported
 */
void DSerial::print( uint_fast32_t num, uint_fast8_t type ) {
    if(type < 2 || type > 16)
        return;

    if(num == 0) {
    	print(0x30);
    	return;
    }
    // Using a 10 char buffer, as an int does not have more characters than that
    char str[10];

    itoa(str, 10, num, type);

    // Filter out all the leading zeroes
    bool reachedStart = false;
    for(int i = 0; i < 10; i++) {
        if(str[i] != '0')
            reachedStart = true;
        if(reachedStart)
            print(str[i]);
    }

}

/**
 * Transmit a carriage return
 */
void DSerial::println( void ) {
    MAP_UART_transmitData(EUSCI_A0_BASE, '\r');
    MAP_UART_transmitData(EUSCI_A0_BASE, '\n');
}

/**
 * Transmit a single byte and end with a newline
 */
void DSerial::println( uint_fast8_t byte ) {
    // The same as print, but add a carriage return after the message
    print(byte);
    println( );
}

/**
 * Print text and end with a newline
 */
void DSerial::println( const char * text ) {
    // The same as print, but add a carriage return after the message
    print(text);
    println( );
}

/**** PRIVATE METHODS ****/

/**
 * Convert a given integer into a corresponding string
 */
// This method is adapted from http://stackoverflow.com/a/10011878/6399671
void itoa( char * str, uint8_t len, uint32_t val, uint8_t base ) {
    uint8_t i;

    for ( i = 1; i <= len; i++ ) {
        str[len - i] = (uint8_t) ((val % base));
        if (str[len - i] > 9)
        {
        	str[len - i] += 'A' - 10;
        }
        else
        {
        	str[len - i] += '0';
        }
        val /= base;
    }
    str[i - 1] = '\0';
}
