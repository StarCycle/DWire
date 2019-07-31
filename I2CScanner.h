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
 
#ifndef __I2CSCANNER_H__
#define __I2CSCANNER_H__

#include <DWire.h>

class I2CScanner
{

public:
    static unsigned char scan( DWire& );
    static unsigned char scan(DWire&, void (*)( unsigned char ));
    static unsigned char scan(DWire &, unsigned char, unsigned char, void (*)( unsigned char ));        
};

#endif
