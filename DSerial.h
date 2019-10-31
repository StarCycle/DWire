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

#ifndef DWIRE_DSERIAL_H_
#define DWIRE_DSERIAL_H_

#include <driverlib.h>

#ifndef DEC
#define DEC 10
#endif

#ifndef HEX
#define HEX 16
#endif

/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
// TODO: make this configurable
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                     // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

class DSerial {
private:

public:
    DSerial( void );
    void begin( void );
    void print( uint_fast8_t );
    void print( const char * );
    void print( uint_fast8_t, uint_fast8_t );
    void println( void );
    void println( uint_fast8_t );
    void println( const char * );
};

#endif /* DWIRE_DSERIAL_H_ */
