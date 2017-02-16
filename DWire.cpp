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

#include <msp.h>
#include "DWire.h"

volatile unsigned char stopCounter = 0;

#define TIMEOUTLIMIT 0xFFFF

/**** MACROs ****/

/**
 * Create the buffers for the specified module number (e.g. M = 0 for EUSCI_B0_BASE)
 */
#define CREATEBUFFERS(M) \
uint8_t EUSCIB ## M ## _txBuffer[TX_BUFFER_SIZE]; \
uint8_t EUSCIB ## M ## _txBufferIndex = 0; \
uint8_t EUSCIB ## M ## _txBufferSize = 0; \
uint8_t EUSCIB ## M ## _rxBuffer[RX_BUFFER_SIZE]; \
uint8_t EUSCIB ## M ## _rxBufferIndex = 0; \
uint8_t EUSCIB ## M ## _rxBufferSize = 0;

/**
 * The main (global) interrupt  handler
 * It ain't pretty, but using this as a macro should increase the performance tremendously
 */
#define IRQHANDLER(M) \
            uint_fast16_t status; \
            status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B ## M ##_BASE); \
            MAP_I2C_clearInterruptFlag(EUSCI_B ## M ##_BASE, status); \
            \
            /* Get a reference to the correct instance */ \
            DWire * instance = instances[M]; \
            /* add if instance false, return */ \
            \
            /* Handle a NAK */ \
            if ( status & EUSCI_B_I2C_NAK_INTERRUPT ) { \
                stopCounter++; \
                /* Disable all other interrupts */ \
                MAP_I2C_disableInterrupt(EUSCI_B## M ##_BASE, \
                        EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_TRANSMIT_INTERRUPT0 \
                                | EUSCI_B_I2C_NAK_INTERRUPT); \
                                \
                EUSCIB## M ##_txBufferIndex = 0; \
                EUSCIB## M ##_rxBufferIndex = 0; \
                if ( instance ) \
                    instance->_finishRequest(true); \
            } \
            \
			/* Check for clock low interrupt: if it is low for too long, then reset the I2C peripheral */ \
			if( status & EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT) { \
				ResetCtl_initiateHardReset(); \
			} \
			\
            /* RXIFG */ \
            /* Triggered when data has been received */ \
            if ( status & EUSCI_B_I2C_RECEIVE_INTERRUPT0 ) \
            { \
                /* If we're a master, then we're handling the slave response after/during a request */ \
                /* instance is used without checking it is non zero */ \
                if ( instance->isMaster( ) ) \
                { \
                /* do range checking around this block to avoid possible errors */ \
                    EUSCIB## M ##_rxBuffer[EUSCIB## M ##_rxBufferIndex] = \
                    MAP_I2C_masterReceiveMultiByteNext(EUSCI_B## M ##_BASE); \
                    EUSCIB## M ##_rxBufferIndex++; \
                    \
                    /* if we only need to read 1 more byte, start sending a stop */ \
                    if ( EUSCIB## M ##_rxBufferIndex == EUSCIB## M ##_rxBufferSize - 1 ) \
                    { \
                        MAP_I2C_masterReceiveMultiByteStop(EUSCI_B## M ##_BASE); \
                        stopCounter++; \
                    } \
                    \
                    if ( EUSCIB## M ##_rxBufferIndex == EUSCIB## M ##_rxBufferSize ) \
                    { \
                        if ( instance ) \
                        { \
                            /* Disable the RX interrupt */ \
                            MAP_I2C_disableInterrupt(EUSCI_B## M ##_BASE, \
                            EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_NAK_INTERRUPT); \
                            /* Mark the request as done */ \
                            instance->_finishRequest( ); \
                        } \
                    } \
                    /* If we're a slave, then we're receiving data from the master */ \
                } else \
                { \
                    EUSCIB## M ##_rxBuffer[EUSCIB## M ##_rxBufferIndex] = MAP_I2C_slaveGetData( \
                            EUSCI_B## M ##_BASE); \
                    EUSCIB1_rxBufferIndex++; \
                } \
            } \
            \
            /* As master: triggered when a byte has been transmitted */ \
            if ( status & EUSCI_B_I2C_TRANSMIT_INTERRUPT0 ) \
            { \
                /* If the module is setup as a master, then we're transmitting data */ \
                if ( instance->isMaster( ) ) \
                { \
                    if ( EUSCIB## M ##_txBufferIndex == 1 ) \
                    { \
                        /* Send a STOP condition if required */ \
                        if ( instance->_isSendStop( ) ) \
                        { \
                            MAP_I2C_masterSendMultiByteStop(EUSCI_B## M ##_BASE); \
                        } \
                        /* Disable the TX interrupt */ \
                        MAP_I2C_disableInterrupt(EUSCI_B## M ##_BASE, \
                        EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT); \
                        EUSCIB1_txBufferIndex--; \
                        \
                    } else if ( EUSCIB## M ##_txBufferIndex > 1 ) \
                    { \
                        /* If we still have data left in the buffer, then transmit that */ \
                        MAP_I2C_masterSendMultiByteNext(EUSCI_B## M ##_BASE, \
                                EUSCIB## M ##_txBuffer[(EUSCIB## M ##_txBufferSize) \
                                        - (EUSCIB## M ##_txBufferIndex) + 1]); \
                        EUSCIB## M ##_txBufferIndex--; \
                    } \
                    /* If we're a slave, then we're handling a request from the master */ \
                } else \
                { \
                    instance->_handleRequestSlave( ); \
                } \
            } \
            \
            /* STPIFG: Called when a STOP is received */ \
            if ( status & EUSCI_B_I2C_STOP_INTERRUPT ) \
            { \
                if ( instance ) \
                { \
                    if ( EUSCIB## M ##_txBufferIndex != 0 && !instance->isMaster( ) ) \
                    { \
                        EUSCIB## M ##_rxBufferIndex = 0; \
                        EUSCIB## M ##_rxBufferSize = 0; \
                    } else if ( EUSCIB## M ##_rxBufferIndex != 0 ) \
                    { \
                        instance->_handleReceive(EUSCIB## M ##_rxBuffer); \
                    } \
                } \
            } \
        }

/**** PROTOTYPES AND CLASSES ****/

/* A data structure containing pointers to relevant buffers
 * to be used by ISRs. */
typedef struct {
        uint32_t module;
        uint8_t * rxBuffer;
        uint8_t * rxBufferIndex;
        uint8_t * rxBufferSize;
        uint8_t * txBuffer;
        uint8_t * txBufferIndex;
        uint8_t * txBufferSize;
} IRQParam;

/**** GLOBAL VARIABLES ****/

/* A reference list of DWire instances */
DWire * instances[4];

// The buffers need to be declared globally, as the interrupts are too
#ifdef USING_EUSCI_B0
CREATEBUFFERS(0)
#endif

#ifdef USING_EUSCI_B1
CREATEBUFFERS( 1 )
#endif

#ifdef USING_EUSCI_B2
CREATEBUFFERS(2)
#endif

#ifdef USING_EUSCI_B3
CREATEBUFFERS(3)
#endif

// Fast mode eUSCI settings
const eUSCI_I2C_MasterConfig i2cConfigFastMode = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
        MAP_CS_getSMCLK( ),                     // Get the SMCLK clock frequency
        EUSCI_B_I2C_SET_DATA_RATE_400KBPS,       // Desired I2C Clock of 400khz
        0,                                       // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                 // No Autostop
        };

// Fast mode plus eUSCI settings
const eUSCI_I2C_MasterConfig i2cConfigFastModePlus = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
        MAP_CS_getSMCLK( ),                     // Get the SMCLK clock frequency
        EUSCI_B_I2C_SET_DATA_RATE_1MBPS,       // Desired I2C Clock of 1MHz
        0,                                       // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                 // No Autostop
        };

// Standard mode eUSCI settings
const eUSCI_I2C_MasterConfig i2cConfigStandardMode = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
        MAP_CS_getSMCLK( ),                     // Get the SMCLK clock frequency
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,       // Desired I2C Clock of 100khz
        0,                                       // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                 // No Autostop
        };

/**** CONSTRUCTORS ****/
DWire::DWire( uint_fast32_t module ) {
    this->module = module;
    this->mode = FAST;
}

DWire::DWire( ) {
    this->module = EUSCI_B1_BASE;
    this->mode = FAST;
}

DWire::~DWire( ) {
    // Deregister from the moduleMap
    // Using a switch statement now, but we can make this simpler by using module = 0,1,2,3
    switch (module) {
        case EUSCI_B0_BASE:
            instances[0] = 0;
            break;
        case EUSCI_B1_BASE:
            instances[1] = 0;
            break;
        case EUSCI_B2_BASE:
            instances[2] = 0;
            break;
        case EUSCI_B3_BASE:
            instances[3] = 0;
            break;
    }
}

/**** PUBLIC METHODS ****/

void DWire::begin( ) {

    // Initialising the given module as a master
    busRole = BUS_ROLE_MASTER;
    slaveAddress = 0;
    _initMain( );

    if (mode == FAST) {
        _initMaster( &i2cConfigFastMode );
    } else if(mode == FASTPLUS) {
        _initMaster( &i2cConfigFastModePlus );
    } else {
        _initMaster( &i2cConfigStandardMode );
    }

    // calculate the number of iterations of a loop to generate
    // a delay based on clock speed
    // this is needed to handle NACKs in a way that is independent
    // of CPU speed and OS (Energia or not)
    delayCycles = MAP_CS_getMCLK( ) * 30 / 7905857;
}

void DWire::setStandardMode( ) {
    this->mode = STANDARD;
}

void DWire::setFastMode( ) {
    this->mode = FAST;
}

void DWire::setFastModePlus( ) {
    this->mode = FASTPLUS;
}

void DWire::begin( uint8_t address ) {

    // Initialising the given module as a slave
    busRole = BUS_ROLE_SLAVE;
    slaveAddress = address;

    _initMain( );

    _initSlave( );
}

/**
 * Begin a transmission as a master
 */
void DWire::beginTransmission( uint_fast8_t slaveAddress ) {
    // Starting a transmission as a master to the slave at slaveAddress
    if (busRole != BUS_ROLE_MASTER)
        return;

    // Wait in case a previous message is still being sent
    while (*pTxBufferIndex > 0)
        ;

    if (slaveAddress != this->slaveAddress)
        _setSlaveAddress( slaveAddress );
}

/**
 * Write a single byte
 */
void DWire::write( uint_fast8_t dataByte ) {
    // Add data to the tx buffer
    pTxBuffer[*pTxBufferIndex] = dataByte;
    (*pTxBufferIndex)++;
}

bool DWire::endTransmission( void ) {
    return endTransmission( true );
}

/**
 * End the transmission and transmit the tx buffer's contents over the bus
 */
bool DWire::endTransmission( bool sendStop ) {
    uint32_t timeout;
    // return, if there is nothing to transmit
    if (!*pTxBufferIndex) {
        return 0;
    }

    // Wait until any ongoing (incoming) transmissions are finished
    timeout = 0xFFFF;
    while ( MAP_I2C_masterIsStopSent( module ) == EUSCI_B_I2C_SENDING_STOP
            && timeout)
        timeout--;

    if (!timeout) {
        /* If we can't start the transmission, then reset everything */
        _resetBus( );
        return false;
    }

    this->sendStop = sendStop;
    gotNAK = false;

    // Clear the interrupt flags and enable
    MAP_I2C_clearInterruptFlag( module,
    EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT );

    MAP_I2C_enableInterrupt( module,
    EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT );

    // Set the master into transmit mode
    MAP_I2C_setMode( module, EUSCI_B_I2C_TRANSMIT_MODE );

    // Send the start condition and initial byte
    (*pTxBufferSize) = *pTxBufferIndex;

    // Send the first byte, triggering the TX interrupt
    MAP_I2C_masterSendMultiByteStartWithTimeout( module, pTxBuffer[0],
    TIMEOUTLIMIT );

    // make sure the transmitter buffer has been flushed
    timeout = TIMEOUTLIMIT;
    while (*pTxBufferIndex && timeout)
        timeout--;

    if (!timeout) {
        _resetBus( );
        return false;
    }

    if (gotNAK) {
        _I2CDelay( );
        MAP_I2C_masterReceiveMultiByteStop( module );
    }
    return gotNAK;
}

/**
 * Request data from a SLAVE as a MASTER
 */
uint8_t DWire::requestFrom( uint_fast8_t slaveAddress, uint_fast8_t numBytes ) {
    uint32_t timeout;
    // No point of doing anything else if there we're not a MASTER
    if (busRole != BUS_ROLE_MASTER)
        return 0;

    // still something to send? Flush the TX buffer but do not send a STOP
    if (*pTxBufferIndex > 0) {
        endTransmission( false );
    } else {
        // Wait until any request is finished
        timeout = TIMEOUTLIMIT;
        while ( MAP_I2C_masterIsStopSent( module ) == EUSCI_B_I2C_SENDING_STOP
                && timeout)
            timeout--;
    }

    if (!timeout) {
        /* If we get a timeout, then reset everything */
        _resetBus( );
        return 0;
    }

    // Re-initialise the rx buffer
    // and make sure we never request 1 byte only
    // this is an anomalous behaviour of the MSP432 related to the double
    // buffering of I2C. This is a workaround.
    if (numBytes == 1) {
        *pRxBufferSize = 2;
    } else {
        *pRxBufferSize = numBytes;
    }
    *pRxBufferIndex = 0;

    // Configure the correct slave
    MAP_I2C_setSlaveAddress( module, slaveAddress );
    this->slaveAddress = slaveAddress;

    MAP_I2C_clearInterruptFlag( module,
    EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_NAK_INTERRUPT );
    MAP_I2C_enableInterrupt( module,
    EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_NAK_INTERRUPT );

    // Set the master into receive mode
    MAP_I2C_setMode( module, EUSCI_B_I2C_RECEIVE_MODE );

    // Initialize the flag showing the status of the request
    requestDone = false;
    gotNAK = false;

    // Send the START
    MAP_I2C_masterReceiveStart( module );

    // Send a stop early if we're only requesting one byte
    // to prevent timing issues
    //if ( numBytes == 1 ) {
    //while ( MAP_I2C_masterIsStartSent(module) )
    //    ;
    //    MAP_I2C_masterReceiveMultiByteStop(module);
    //}

    // Wait until the request is done
    timeout = TIMEOUTLIMIT;
    while (!requestDone && timeout)
        timeout--;

    if (!timeout) {
        /* If we get a timeout, then reset everything */
        _resetBus( );
        return 0;
    }

    // Reset the buffer
    (*pRxBufferIndex) = 0;
    (*pRxBufferSize) = 0;

    stopCounter = 0;

    if (gotNAK) {
        _I2CDelay( );
        MAP_I2C_masterReceiveMultiByteStop( module );
        return 0;
    } else {
        if (numBytes == 1)
            return --rxReadLength;
        else
            return rxReadLength;
    }
}

/**
 * Reads a single byte from the rx buffer
 */
uint8_t DWire::read( void ) {

    // Return a 0 if there is nothing to read
    if (rxReadLength == 0)
        return 0;

    uint8_t byte = rxLocalBuffer[rxReadIndex];

    // Check whether this was the last byte. If so, reset.
        rxReadIndex = 0;
        rxReadLength = 0;
    }
    return byte;
}

/**
 * Register the user's interrupt handler
 */
void DWire::onRequest( void (*islHandle)( void ) ) {
    user_onRequest = islHandle;
}

/**
 * Register the interrupt handler
 * The argument contains the number of bytes received
 */
void DWire::onReceive( void (*islHandle)( uint8_t ) ) {
    user_onReceive = islHandle;
}

/**
 * Returns true if the module is configured as a master
 */
    if (busRole == BUS_ROLE_MASTER) {
        return true;
    } else {
        return false;
    }
}

/**** PRIVATE METHODS ****/

/**
 * The main initialisation method to setup pins and interrupts
 */
void DWire::_initMain( void ) {

    // Initialise the receiver buffer and related variables
    rxReadIndex = 0;
    rxReadLength = 0;

    requestDone = false;
    sendStop = true;

    switch (module) {
#ifdef USING_EUSCI_B0
        case EUSCI_B0_BASE:

        instances[0] = this;

        pTxBuffer = EUSCIB0_txBuffer;
        pTxBufferIndex = &EUSCIB0_txBufferIndex;
        pTxBufferSize = &EUSCIB0_txBufferSize;

        pRxBuffer = EUSCIB0_rxBuffer;
        pRxBufferIndex = &EUSCIB0_rxBufferIndex;
        pRxBufferSize = &EUSCIB0_rxBufferSize;

        modulePort = EUSCI_B0_PORT;
        modulePins = EUSCI_B0_PINS;

        intModule = INT_EUSCIB0;

        MAP_I2C_registerInterrupt(module, EUSCIB0_IRQHandler);
        break;
#endif
#ifdef USING_EUSCI_B1
        case EUSCI_B1_BASE:

            instances[1] = this;

            pTxBuffer = EUSCIB1_txBuffer;
            pTxBufferIndex = &EUSCIB1_txBufferIndex;
            pTxBufferSize = &EUSCIB1_txBufferSize;

            pRxBuffer = EUSCIB1_rxBuffer;
            pRxBufferIndex = &EUSCIB1_rxBufferIndex;
            pRxBufferSize = &EUSCIB1_rxBufferSize;

            modulePort = EUSCI_B1_PORT;
            modulePins = EUSCI_B1_PINS;

            intModule = INT_EUSCIB1;

            MAP_I2C_registerInterrupt( module, EUSCIB1_IRQHandler );
            break;
#endif
#ifdef USING_EUSCI_B2
            case EUSCI_B2_BASE:

            instances[2] = this;

            pTxBuffer = EUSCIB2_txBuffer;
            pTxBufferIndex = &EUSCIB2_txBufferIndex;
            pTxBufferSize = &EUSCIB2_txBufferSize;

            pRxBuffer = EUSCIB2_rxBuffer;
            pRxBufferIndex = &EUSCIB2_rxBufferIndex;
            pRxBufferSize = &EUSCIB2_rxBufferSize;

            modulePort = EUSCI_B2_PORT;
            modulePins = EUSCI_B2_PINS;

            intModule = INT_EUSCIB2;

            MAP_I2C_registerInterrupt(module, EUSCIB2_IRQHandler);
            break;
#endif
#ifdef USING_EUSCI_B3
            case EUSCI_B3_BASE:

            instances[3] = this;

            pTxBuffer = EUSCIB3_txBuffer;
            pTxBufferIndex = &EUSCIB3_txBufferIndex;
            pTxBufferSize = &EUSCIB3_txBufferSize;

            pRxBuffer = EUSCIB3_rxBuffer;
            pRxBufferIndex = &EUSCIB3_rxBufferIndex;
            pRxBufferSize = &EUSCIB3_rxBufferSize;

            modulePort = EUSCI_B3_PORT
            ;
            modulePins = EUSCI_B3_PINS;

            intModule = INT_EUSCIB3;

            MAP_I2C_registerInterrupt(module, EUSCIB3_IRQHandler);
            break;
#endif
        default:
            return;
    }
}

/**
 * Called to set the eUSCI module in 'master' mode
 */
void DWire::_initMaster( const eUSCI_I2C_MasterConfig * i2cConfig ) {

    // Initialise the pins
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin( modulePort, modulePins,
    GPIO_PRIMARY_MODULE_FUNCTION );

    // Initializing I2C Master to SMCLK with no autostop
    MAP_I2C_initMaster( module, i2cConfig );

    // Specify slave address
    MAP_I2C_setSlaveAddress( module, slaveAddress );

    // Set Master in transmit mode
    MAP_I2C_setMode( module, EUSCI_B_I2C_TRANSMIT_MODE );

    // Enable I2C Module to start operations
    MAP_I2C_enableModule( module );

    // Clear the interrupt flag
    MAP_I2C_clearInterruptFlag( module,
            EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT
                    + EUSCI_B_I2C_RECEIVE_INTERRUPT0 );

    // Register the interrupts on the correct module
    MAP_Interrupt_enableInterrupt( intModule );
    MAP_Interrupt_enableMaster( );
}

void DWire::_initSlave( void ) {
    // Init the pins
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin( modulePort, modulePins,
    GPIO_PRIMARY_MODULE_FUNCTION );

    // initialise driverlib
    MAP_I2C_initSlave( module, slaveAddress, EUSCI_B_I2C_OWN_ADDRESS_OFFSET0,
    EUSCI_B_I2C_OWN_ADDRESS_ENABLE );

    // Enable the module and enable interrupts
    MAP_I2C_enableModule( module );
    MAP_I2C_clearInterruptFlag( module,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_STOP_INTERRUPT
                    | EUSCI_B_I2C_TRANSMIT_INTERRUPT0 | EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT );
    MAP_I2C_enableInterrupt( module,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_STOP_INTERRUPT
                    | EUSCI_B_I2C_TRANSMIT_INTERRUPT0 | EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT );

    /* Enable the clock low timeout */
    EUSCI_B_CMSIS( module )->CTLW1 = (EUSCI_B_CMSIS( module )->CTLW1
            & ~EUSCI_B_CTLW1_CLTO_MASK) | 0xC0;

    MAP_Interrupt_enableInterrupt( intModule );
    MAP_Interrupt_enableMaster( );
}

/**
 * Re-set the slave address (the target address when master or the slave's address when slave)
 */
void DWire::_setSlaveAddress( uint_fast8_t newAddress ) {
    slaveAddress = newAddress;
    MAP_I2C_setSlaveAddress( module, newAddress );
}

/**
 * Handle a request ISL as a slave
 */
void DWire::_handleRequestSlave( void ) {
    // Check whether a user interrupt has been set
    //if ( !user_onRequest )
    //    return;

    // If no message has been set, then call the user interrupt to set
    if (!(*pTxBufferIndex)) {
        user_onRequest( );

        *pTxBufferSize = *pTxBufferIndex - 1;
        *pTxBufferIndex = 0;
    }

    // If we've transmitted the entire message, then reset the tx buffer
    if (*pTxBufferIndex > *pTxBufferSize) {
        *pTxBufferIndex = 0;
        *pTxBufferSize = 0;
    } else {
        // Transmit a byte
        MAP_I2C_slavePutData( module, pTxBuffer[*pTxBufferIndex] );
        (*pTxBufferIndex)++;
    }
}

/**
 * Internal process handling the rx buffers, and calling the user's interrupt handles
 */
void DWire::_handleReceive( uint8_t * rxBuffer ) {
    // No need to do anything if there is no handler registered
    if (!user_onReceive)
        return;

    // Check whether the user application is still reading out the local buffer.
    // This needs to be tested to make sure it doesn't give any problems.
    if (rxReadIndex != 0 && rxReadLength != 0)
        return;

    // Copy the main buffer into a local buffer
    rxReadLength = *pRxBufferIndex;
    rxReadIndex = 0;

    for (int i = 0; i < rxReadLength; i++)
        this->rxLocalBuffer[i] = rxBuffer[i];

    // Reset the main buffer
    (*pRxBufferIndex) = 0;

    user_onReceive( rxReadLength );
}

void DWire::_finishRequest( void ) {
    for (int i = 0; i <= *pRxBufferSize; i++) {
        this->rxLocalBuffer[i] = pRxBuffer[i];
    }
    rxReadIndex = 0;
    rxReadLength = *pRxBufferSize;
    requestDone = true;
}

void DWire::_finishRequest( bool NAK ) {
    gotNAK = NAK;
    rxReadIndex = 0;
    rxReadLength = 0;
    requestDone = true;
}

bool DWire::_isSendStop( void ) {
    return sendStop;
}

void DWire::_I2CDelay( void ) {
    // delay for 1.5 byte-times and send the stop
    // this is needed because the MSP432 ignores any
    // stop if the byte is being received / transmitted

    // if we are in STANDARD mode we need ~120us (4x 30us)
    unsigned char loops = 4;

    if (this->mode == FAST) {
        // if we are in FAST mode, we only need a delay of 30us (~1.5 bytes at 400kHz)
        loops = 1;
    }
    for (unsigned char x = 0; x < loops; x++) {
        for (int i = 0; i < delayCycles; i++) {
            __no_operation();
        }
    }
}

void DWire::_resetBus( void ) {
    /* Reset buffers */
    *pTxBufferIndex = 0;
    *pTxBufferSize = 0;
    *pRxBufferIndex = 0;
    *pRxBufferSize = 0;

    /* Reset the module */
    MAP_I2C_disableModule( module );

    /* Perform bus clear according to UM10204 section 3.1.16 */
    if (this->isMaster( )) {
        uint_fast16_t moduleSclPin = modulePins & (GPIO_PIN5 + GPIO_PIN7); // FIXME
        MAP_GPIO_setOutputLowOnPin( modulePort, moduleSclPin );
        for (uint_fast8_t i = 0; i < 9; i++) {
            MAP_GPIO_setAsOutputPin( modulePort, moduleSclPin );
            this->_I2CDelay( );
            MAP_GPIO_setAsInputPin( modulePort, moduleSclPin );
            this->_I2CDelay( );
        }
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin( modulePort,
                moduleSclPin, GPIO_PRIMARY_MODULE_FUNCTION );
    }

    /* Re-enable the module */
    MAP_I2C_enableModule( module );
}

/**** ISR/IRQ Handles ****/

#ifdef USING_EUSCI_B0
/*
 * Handle everything on EUSCI_B0
 */
extern "C" {
    void EUSCIB0_IRQHandler( void ) {
        IRQHANDLER(0)
    }
}

/* USING_EUSCI_B0 */
#endif

#ifdef USING_EUSCI_B1
/*
 * Handle everything on EUSCI_B1
 */
extern "C" {
void EUSCIB1_IRQHandler( void ) {
    IRQHANDLER( 1 )
}

/* USING_EUSCI_B1 */
#endif

#ifdef USING_EUSCI_B2
/*
 * Handle everything on EUSCI_B2
 */
extern "C" {
    void EUSCIB2_IRQHandler( void ) {
        IRQHANDLER(2)
    }
}
/* USING_EUSCI_B2 */
#endif

#ifdef USING_EUSCI_B3
/*
 * Handle everything on EUSCI_B3
 */
extern "C" {
    void EUSCIB3_IRQHandler( void ) {
        IRQHANDLER(3)
    }
}

/* USING_EUSCI_B3 */
#endif
