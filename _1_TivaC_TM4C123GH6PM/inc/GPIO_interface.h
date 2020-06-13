/*
 * interface.h
 *
 *  Created on: May 3, 2020
 *      Author: Mohamed  Ebead
 */

#ifndef INTERFACE_H_
#define INTERFACE_H_


void delay (void) ;

void RedLed_GPIO_Initialize (void) ;
void BlueLed_GPIO_Initialize (void) ;
void GreenLed_GPIO_Initialize (void) ;

void RedLed_GPIO_Toggle (void) ;

void PushButton2_GPIO_INT_Initialize (void) ;







#endif /* INTERFACE_H_ */
