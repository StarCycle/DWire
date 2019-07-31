/*
 * Copyright (c) 2016 by Stefano Speretta <s.speretta@tudelft.nl>
 *
 * I2CScanner: a library to provide high-level functions to scan 
 * the I2C bus. It is designed to use Energia (the Arduino port 
 * for MSP microcontrollers) on an MSP432.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 
 * version 3, both as published by the Free Software Foundation.
 *
 */

#include "I2CScanner.h"

/**
 *
 *   Scan the I2C bus and list the available devices.
 *   The available addresses are printed to the Serial console
 *   
 *   Returns:
 *   unsigned char        number of devices found
 *
 */
unsigned char I2CScanner::scan(DWire &i2c)
{
    unsigned char devices = 0;
    for(unsigned char address = 1; address < 126; address++ )
    {
        if (i2c.requestFrom(address, 1))  
        {
            devices++;
        }
    }   
    
    return devices;
}

unsigned char I2CScanner::scan(DWire &i2c, void (*devFoundHandler)( unsigned char ))
{
    unsigned char devices = 0;
    for(unsigned char address = 1; address < 126; address++ )
    {
        if (i2c.requestFrom(address, 1)) 
        {
            devices++;
            // if an handler is defined, notify which address was found
            if (devFoundHandler)
            {
                devFoundHandler(address);
            }
        }
    }   
    
    return devices;
}

unsigned char I2CScanner::scan(DWire &i2c, unsigned char start, unsigned char end, void (*devFoundHandler)( unsigned char ))
{
    unsigned char devices = 0;
    for(unsigned char address = start; address < end; address++ )
    {
        if (i2c.requestFrom(address, 1)) 
        {
            devices++;
            // if an handler is defined, notify which address was found
            if (devFoundHandler)
            {
                devFoundHandler(address);
            }
        }
    }   
    
    return devices;
}
