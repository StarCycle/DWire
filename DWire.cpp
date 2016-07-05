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

#include "DWire.h"

#include "modulemap.h"

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

/* The main IRQ handling function */
void IRQHandler(IRQParam);

/**** GLOBAL VARIABLES ****/

// The buffers need to be declared globally, as the interrupts are too
#ifdef USING_EUSCI_B0

uint8_t EUSCIB0_txBuffer[TX_BUFFER_SIZE];
uint8_t EUSCIB0_txBufferIndex = 0;
uint8_t EUSCIB0_txBufferSize = 0;

uint8_t EUSCIB0_rxBuffer[RX_BUFFER_SIZE];
uint8_t EUSCIB0_rxBufferIndex = 0;
uint8_t EUSCIB0_rxBufferSize = 0;
#endif

#ifdef USING_EUSCI_B1

uint8_t EUSCIB1_txBuffer[TX_BUFFER_SIZE];
uint8_t EUSCIB1_txBufferIndex = 0;
uint8_t EUSCIB1_txBufferSize = 0;

uint8_t EUSCIB1_rxBuffer[RX_BUFFER_SIZE];
uint8_t EUSCIB1_rxBufferIndex = 0;
uint8_t EUSCIB1_rxBufferSize = 0;
#endif

#ifdef USING_EUSCI_B2

uint8_t EUSCIB2_txBuffer[TX_BUFFER_SIZE];
uint8_t EUSCIB2_txBufferIndex = 0;
uint8_t EUSCIB2_txBufferSize = 0;

uint8_t EUSCIB2_rxBuffer[RX_BUFFER_SIZE];
uint8_t EUSCIB2_rxBufferIndex = 0;
uint8_t EUSCIB2_rxBufferSize = 0;
#endif

#ifdef USING_EUSCI_B3

uint8_t EUSCIB3_txBuffer[TX_BUFFER_SIZE];
uint8_t EUSCIB3_txBufferIndex = 0;
uint8_t EUSCIB3_txBufferSize = 0;

uint8_t EUSCIB3_rxBuffer[RX_BUFFER_SIZE];
uint8_t EUSCIB3_rxBufferIndex = 0;
uint8_t EUSCIB3_rxBufferSize = 0;
#endif

// Fast mode eUSCI settings
const eUSCI_I2C_MasterConfig i2cConfigFastMode = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,                   // SMCLK Clock Source
		MAP_CS_getSMCLK(),                       // Get the SMCLK clock frequency
		EUSCI_B_I2C_SET_DATA_RATE_400KBPS,       // Desired I2C Clock of 400khz
		0,                                       // No byte counter threshold
		EUSCI_B_I2C_NO_AUTO_STOP                 // No Autostop
		};

// Standard mode eUSCI settings
const eUSCI_I2C_MasterConfig i2cConfigStandardMode = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,                   // SMCLK Clock Source
		MAP_CS_getSMCLK(),                       // Get the SMCLK clock frequency
		EUSCI_B_I2C_SET_DATA_RATE_100KBPS,       // Desired I2C Clock of 100khz
		0,                                       // No byte counter threshold
		EUSCI_B_I2C_NO_AUTO_STOP                 // No Autostop
		};
		
/**** CONSTRUCTORS ****/

DWire::DWire( uint_fast32_t module ) {
	this->module = module;
	this->mode = FAST;
}

DWire::DWire(  ) {
	this->module = EUSCI_B1_BASE;
	this->mode = FAST;
}

DWire::~DWire() {
	// Deregister from the moduleMap
	unregisterModule(this);
}

/**** PUBLIC METHODS ****/

void DWire::begin() {

	// Initialising the given module as a master
	busRole = BUS_ROLE_MASTER;
	slaveAddress = 0;
	_initMain();

	if (mode == FAST)
	{
		_initMaster(&i2cConfigFastMode);
	}
	else
	{
		_initMaster(&i2cConfigStandardMode);
	}
}

void DWire::setStandardMode(  ) {
	this->mode = STANDARD;
}

void DWire::setFastMode(  ) {
	this->mode = FAST;
}
    
void DWire::begin(uint8_t address) {

	// Initialising the given module as a slave
	busRole = BUS_ROLE_SLAVE;
	slaveAddress = address;

	_initMain();

	_initSlave();
}

/**
 * Begin a transmission as a master
 */
void DWire::beginTransmission(uint_fast8_t slaveAddress) {
	// Starting a transmission as a master to the slave at slaveAddress
	if (busRole != BUS_ROLE_MASTER)
		return;

	// Wait in case a previous message is still being sent
	while ( MAP_I2C_masterIsStopSent(module) == EUSCI_B_I2C_SENDING_STOP)
		;

	if (slaveAddress != this->slaveAddress)
		_setSlaveAddress(slaveAddress);
}

/**
 * Write a single byte
 */
void DWire::write(uint8_t dataByte) {
	// Add data to the tx buffer
	pTxBuffer[*pTxBufferIndex] = dataByte;
	(*pTxBufferIndex)++;
}

void DWire::endTransmission(void) {
	endTransmission(true);
}

/**
 * End the transmission and transmit the tx buffer's contents over the bus
 */
void DWire::endTransmission(bool sendStop) {

	if (!*pTxBufferIndex) {
		return;
	}

	// Wait until any ongoing (incoming) transmissions are finished
	//while ( MAP_I2C_isBusBusy(module) == EUSCI_B_I2C_BUS_BUSY )
	//    ;

	this->sendStop = sendStop;

	// Send the start condition and initial byte
	(*pTxBufferSize) = *pTxBufferIndex;
	(*pTxBufferIndex)--;

	// Send the first byte, triggering the TX interrupt
	MAP_I2C_masterSendMultiByteStart(module, pTxBuffer[0]);
}

/**
 * Request data from a SLAVE as a MASTER
 */
uint8_t DWire::requestFrom(uint_fast8_t slaveAddress, uint_fast8_t numBytes) {
	// No point of doing anything else if there we're not a MASTER
	if (busRole != BUS_ROLE_MASTER)
		return 0;

	if (*pTxBufferIndex > 0) {
		endTransmission(false);
	}

	while (!sendStop)
		;

	// Re-initialise the rx buffer
	*pRxBufferSize = numBytes;
	*pRxBufferIndex = 0;

	// Configure the correct slave
	MAP_I2C_setSlaveAddress(module, slaveAddress);
	this->slaveAddress = slaveAddress;

	MAP_I2C_disableInterrupt(module, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);

	// Set the master into receive mode
	MAP_I2C_setMode(module, EUSCI_B_I2C_RECEIVE_MODE);

	// Send the START
	MAP_I2C_masterReceiveStart(module);

	// Send a stop early if we're only requesting one byte
	// to prevent timing issues
	if (numBytes == 1) {
		MAP_I2C_masterReceiveMultiByteStop(module);
	}

	// Initialize the flag showing the status of the request
	requestDone = false;
	gotNAK = false;

	// Wait until the request is done
	while (!requestDone)
		;

	MAP_I2C_setMode(module, EUSCI_B_I2C_TRANSMIT_MODE);

	MAP_I2C_enableInterrupt(module, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
	MAP_I2C_clearInterruptFlag(module, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);

	// Reset the buffer
	(*pRxBufferIndex) = 0;
	(*pRxBufferSize) = 0;

	if (gotNAK) {
		return 0;
	} else {
		return rxReadLength;
	}
}

/**
 * Reads a single byte from the rx buffer
 */
uint8_t DWire::read(void) {

	// Wait if there is nothing to read
	while (rxReadIndex == 0 && rxReadLength == 0)
		;

	uint8_t byte = rxLocalBuffer[rxReadIndex];
	rxReadIndex++;

	// Check whether this was the last byte. If so, reset.
	if (rxReadIndex == rxReadLength) {
		rxReadIndex = 0;
		rxReadLength = 0;
	}
	return byte;
}

/**
 * Register the user's interrupt handler
 */
void DWire::onRequest(void (*islHandle)(void)) {
	user_onRequest = islHandle;
}

/**
 * Register the interrupt handler
 * The argument contains the number of bytes received
 */
void DWire::onReceive(void (*islHandle)(uint8_t)) {
	user_onReceive = islHandle;
}

/**
 * Returns true if the module is configured as a master
 */
bool DWire::isMaster(void) {
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
		pTxBuffer = EUSCIB1_txBuffer;
		pTxBufferIndex = &EUSCIB1_txBufferIndex;
		pTxBufferSize = &EUSCIB1_txBufferSize;

		pRxBuffer = EUSCIB1_rxBuffer;
		pRxBufferIndex = &EUSCIB1_rxBufferIndex;
		pRxBufferSize = &EUSCIB1_rxBufferSize;

		modulePort = EUSCI_B1_PORT;
		modulePins = EUSCI_B1_PINS;

		intModule = INT_EUSCIB1;

		MAP_I2C_registerInterrupt(module, EUSCIB1_IRQHandler);
		break;
#endif
#ifdef USING_EUSCI_B2
	case EUSCI_B2_BASE:
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
		pTxBuffer = EUSCIB3_txBuffer;
		pTxBufferIndex = &EUSCIB3_txBufferIndex;
		pTxBufferSize = &EUSCIB3_txBufferSize;

		pRxBuffer = EUSCIB3_rxBuffer;
		pRxBufferIndex = &EUSCIB3_rxBufferIndex;
		pRxBufferSize = &EUSCIB3_rxBufferSize;

		modulePort = EUSCI_B3_PORT;
		modulePins = EUSCI_B3_PINS;

		intModule = INT_EUSCIB3;

		MAP_I2C_registerInterrupt(module, EUSCIB3_IRQHandler);
		break;
#endif
	default:
		return;
	}

	// Register this instance in the 'moduleMap'
	registerModule(this);
}


/**
 * Called to set the eUSCI module in 'master' mode
 */
void DWire::_initMaster(const eUSCI_I2C_MasterConfig * i2cConfig) {

	// Initialise the pins
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(modulePort, modulePins,
	GPIO_PRIMARY_MODULE_FUNCTION);

	// Initializing I2C Master to SMCLK with no autostop
	MAP_I2C_initMaster(module, i2cConfig);

	// Specify slave address
	MAP_I2C_setSlaveAddress(module, slaveAddress);

	// Set Master in transmit mode
	MAP_I2C_setMode(module, EUSCI_B_I2C_TRANSMIT_MODE);

	// Enable I2C Module to start operations
	MAP_I2C_enableModule(module);

	// Enable and clear the interrupt flag
	MAP_I2C_clearInterruptFlag(module,
			EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT
					+ EUSCI_B_I2C_RECEIVE_INTERRUPT0);

	// Enable master interrupts
	MAP_I2C_enableInterrupt(module,
			EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT
					+ EUSCI_B_I2C_RECEIVE_INTERRUPT0);

	// Register the interrupts on the correct module
	MAP_Interrupt_enableInterrupt(intModule);
	MAP_Interrupt_enableMaster();
}

void DWire::_initSlave(void) {
	// Init the pins
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(modulePort, modulePins,
	GPIO_PRIMARY_MODULE_FUNCTION);

	// initialise driverlib
	MAP_I2C_initSlave(module, slaveAddress, EUSCI_B_I2C_OWN_ADDRESS_OFFSET0,
	EUSCI_B_I2C_OWN_ADDRESS_ENABLE);

	// Enable the module
	MAP_I2C_enableModule(module);

	// Enable the module and enable interrupts
	MAP_I2C_enableModule(module);
	MAP_I2C_clearInterruptFlag(module,
			EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_STOP_INTERRUPT
					| EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
	MAP_I2C_enableInterrupt(module,
			EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_STOP_INTERRUPT
					| EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
	//MAP_Interrupt_enableSleepOnIsrExit();
	MAP_Interrupt_enableInterrupt(intModule);
	MAP_Interrupt_enableMaster();
}

/**
 * Re-set the slave address (the target address when master or the slave's address when slave)
 */
void DWire::_setSlaveAddress(uint_fast8_t newAddress) {
	slaveAddress = newAddress;
	MAP_I2C_setSlaveAddress(module, newAddress);
}

/**
 * Handle a request ISL as a slave
 */
void DWire::_handleRequestSlave(void) {
	// Check whether a user interrupt has been set
	if (!user_onRequest)
		return;

	// If no message has been set, then call the user interrupt to set
	if (!(*pTxBufferIndex)) {
		user_onRequest();

		*pTxBufferSize = *pTxBufferIndex;
		*pTxBufferIndex = 0;
	}

	// If we've transmitted the entire message, then reset the tx buffer
	if (*pTxBufferIndex > *pTxBufferSize) {
		*pTxBufferIndex = 0;
		*pTxBufferSize = 0;
	} else {
		// Transmit a byte
		MAP_I2C_slavePutData(module, pTxBuffer[*pTxBufferIndex]);
		(*pTxBufferIndex)++;
	}
}

/**
 * Internal process handling the rx buffers, and calling the user's interrupt handles
 */
void DWire::_handleReceive(uint8_t * rxBuffer) {
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

	user_onReceive(rxReadLength);
}

void DWire::_finishRequest(void) {
	for (int i = 0; i <= *pRxBufferSize; i++) {
		this->rxLocalBuffer[i] = pRxBuffer[i];
	}
	rxReadIndex = 0;
	rxReadLength = *pRxBufferSize;
	requestDone = true;
}

void DWire::_finishRequest(bool NAK) {
	gotNAK = NAK;
	requestDone = true;
}

bool DWire::_isSendStop(bool resetAfterwards) {
	if (!sendStop) {
		if (resetAfterwards)
			sendStop = true;
		return false;
	} else {
		return true;
	}
}

/**** ISR/IRQ Handles ****/

/**
 * The main (global) interrupt  handler
 */
void IRQHandler(IRQParam param) {

	uint_fast16_t status;

	status = MAP_I2C_getEnabledInterruptStatus(param.module);
	MAP_I2C_clearInterruptFlag(param.module, status);

	/* RXIFG */
	// Triggered when data has been received
	if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0) {

		// If the rxBufferSize > 0, then we're a master performing a request
		if (*param.rxBufferSize > 0) {
			param.rxBuffer[*param.rxBufferIndex] =
			MAP_I2C_masterReceiveMultiByteNext(param.module);
			(*param.rxBufferIndex)++;

			if (*param.rxBufferIndex == *param.rxBufferSize) {
				DWire * instance = getInstance(param.module);
				if (instance) {
					instance->_finishRequest();
				}
			}

			// Otherwise we're a slave receiving data
		} else {
			param.rxBuffer[*param.rxBufferIndex] = MAP_I2C_slaveGetData(
					param.module);
			(*param.rxBufferIndex)++;
		}
	}

	// As master: triggered when a byte has been transmitted
	// As slave: triggered on request */
	if (status & EUSCI_B_I2C_TRANSMIT_INTERRUPT0) {
		DWire * instance = getInstance(param.module);

		if (instance) {

			// If the module is setup as a master, then we're transmitting data
			if (instance->isMaster()) {
				// If we've transmitted the last byte from the buffer, then send a stop
				if (!(*param.txBufferIndex)) {
					if (instance->_isSendStop(false))
						MAP_I2C_masterSendMultiByteStop(param.module);
					instance->_isSendStop(true);

				} else {
					// If we still have data left in the buffer, then transmit that
					MAP_I2C_masterSendMultiByteNext(param.module,
							param.txBuffer[(*param.txBufferSize)
									- (*param.txBufferIndex)]);
					(*param.txBufferIndex)--;
				}
				// Otherwise we're a slave and a master is requesting data
			} else {
				instance->_handleRequestSlave();
			}
		}
	}

	// Handle a NAK
	if (status & EUSCI_B_I2C_NAK_INTERRUPT) {
		//MAP_I2C_masterSendStart(param.module);
		DWire * instance = getInstance(param.module);
		//*param.txBufferIndex = 0;
		if (instance)
			if (*param.rxBufferSize > 0) {
			}
		instance->_finishRequest(true);
	}

	/* STPIFG
	 * Called when a STOP is received
	 */
	if (status & EUSCI_B_I2C_STOP_INTERRUPT) {
		DWire * instance = getInstance(param.module);
		if (instance) {
			if (*param.txBufferIndex != 0 && !instance->isMaster()) {
				MAP_I2C_slavePutData(instance->module, 0);
				*param.rxBufferIndex = 0;
				*param.rxBufferSize = 0;
			} else if (*param.rxBufferIndex != 0) {
				instance->_handleReceive(param.rxBuffer);
			}
		}
	}
}

#ifdef USING_EUSCI_B0
/*
 * Handle everything on EUSCI_B0
 */
extern "C" {
void EUSCIB0_IRQHandler(void) {

	// Send a STOP if we're done in request mode
	// This is done here as triggering the interrupt takes too long
	if ( MAP_I2C_getInterruptStatus(EUSCI_B0_BASE,
	EUSCI_B_I2C_RECEIVE_INTERRUPT0)
			&& (EUSCIB0_rxBufferIndex == EUSCIB0_rxBufferSize - 1)
			&& EUSCIB0_rxBufferIndex != 0) {
		MAP_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
	}

	IRQParam param;
	param.module = EUSCI_B0_BASE;
	param.rxBuffer = EUSCIB0_rxBuffer;
	param.rxBufferIndex = &EUSCIB0_rxBufferIndex;
	param.rxBufferSize = &EUSCIB0_rxBufferSize;
	param.txBuffer = EUSCIB0_txBuffer;
	param.txBufferIndex = &EUSCIB0_txBufferIndex;
	param.txBufferSize = &EUSCIB0_txBufferSize;

	IRQHandler(param);
}
}

/* USING_EUSCI_B0 */
#endif

#ifdef USING_EUSCI_B1
/*
 * Handle everything on EUSCI_B1
 */
extern "C" {
void EUSCIB1_IRQHandler(void) {

	// Send a STOP if we're done in request mode
	// This is done here as triggering the interrupt takes too long
	if ( MAP_I2C_getInterruptStatus(EUSCI_B1_BASE,
	EUSCI_B_I2C_RECEIVE_INTERRUPT0)
			&& (EUSCIB1_rxBufferIndex == EUSCIB1_rxBufferSize - 1) ) {
			//&& EUSCIB1_rxBufferIndex != 0) {
		MAP_I2C_masterReceiveMultiByteStop(EUSCI_B1_BASE);
	}

	IRQParam param;
	param.module = EUSCI_B1_BASE;
	param.rxBuffer = EUSCIB1_rxBuffer;
	param.rxBufferIndex = &EUSCIB1_rxBufferIndex;
	param.rxBufferSize = &EUSCIB1_rxBufferSize;
	param.txBuffer = EUSCIB1_txBuffer;
	param.txBufferIndex = &EUSCIB1_txBufferIndex;
	param.txBufferSize = &EUSCIB1_txBufferSize;

	IRQHandler(param);
}
}

/* USING_EUSCI_B1 */
#endif

#ifdef USING_EUSCI_B2
/*
 * Handle everything on EUSCI_B2
 */
extern "C" {
void EUSCIB2_IRQHandler(void) {

	// Send a STOP if we're done in request mode
	// This is done here as triggering the interrupt takes too long
	if ( MAP_I2C_getInterruptStatus(EUSCI_B2_BASE,
	EUSCI_B_I2C_RECEIVE_INTERRUPT0)
			&& (EUSCIB2_rxBufferIndex == EUSCIB2_rxBufferSize - 1)
			&& EUSCIB2_rxBufferIndex != 0) {
		MAP_I2C_masterReceiveMultiByteStop(EUSCI_B2_BASE);
	}

	IRQParam param;
	param.module = EUSCI_B2_BASE;
	param.rxBuffer = EUSCIB2_rxBuffer;
	param.rxBufferIndex = &EUSCIB2_rxBufferIndex;
	param.rxBufferSize = &EUSCIB2_rxBufferSize;
	param.txBuffer = EUSCIB2_txBuffer;
	param.txBufferIndex = &EUSCIB2_txBufferIndex;
	param.txBufferSize = &EUSCIB2_txBufferSize;

	IRQHandler(param);
}
}

/* USING_EUSCI_B2 */
#endif

#ifdef USING_EUSCI_B3
/*
 * Handle everything on EUSCI_B3
 */
extern "C" {
void EUSCIB3_IRQHandler(void) {

	// Send a STOP if we're done in request mode
	// This is done here as triggering the interrupt takes too long
	if ( MAP_I2C_getInterruptStatus(EUSCI_B3_BASE,
	EUSCI_B_I2C_RECEIVE_INTERRUPT0)
			&& (EUSCIB3_rxBufferIndex == EUSCIB3_rxBufferSize - 1)
			&& EUSCIB3_rxBufferIndex != 0) {
		MAP_I2C_masterReceiveMultiByteStop(EUSCI_B3_BASE);
	}

	IRQParam param;
	param.module = EUSCI_B3_BASE;
	param.rxBuffer = EUSCIB3_rxBuffer;
	param.rxBufferIndex = &EUSCIB3_rxBufferIndex;
	param.rxBufferSize = &EUSCIB3_rxBufferSize;
	param.txBuffer = EUSCIB3_txBuffer;
	param.txBufferIndex = &EUSCIB3_txBufferIndex;
	param.txBufferSize = &EUSCIB3_txBufferSize;

	IRQHandler(param);
}
}

/* USING_EUSCI_B3 */
#endif
