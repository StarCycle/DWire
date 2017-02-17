/*
 * dwire_msp432p401r.h
 *
 *  Created on: 30 May 2016
 *      Author: Stefan van der Linden
 */

#ifndef INCLUDE_DWIRE_MSP432P401R_H_
#define INCLUDE_DWIRE_MSP432P401R_H_

#ifdef USING_EUSCI_B0
#define EUSCI_B0_PORT GPIO_PORT_P1
#define EUSCI_B0_PINS (GPIO_PIN6 + GPIO_PIN7)
#endif

#ifdef USING_EUSCI_B1
#define EUSCI_B1_PORT GPIO_PORT_P6
#define EUSCI_B1_PINS (GPIO_PIN4 + GPIO_PIN5)
#endif

#ifdef USING_EUSCI_B2
#define EUSCI_B2_PORT GPIO_PORT_P3
#define EUSCI_B2_PINS (GPIO_PIN6 + GPIO_PIN7)
#endif

#ifdef USING_EUSCI_B3
#define EUSCI_B3_PORT GPIO_PORT_P6;
#define EUSCI_B3_PINS (GPIO_PIN6 + GPIO_PIN7)
#endif

#endif /* INCLUDE_DWIRE_MSP432P401R_H_ */
