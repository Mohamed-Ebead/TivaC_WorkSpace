/*
 * SSI_program.c
 *
 *  Created on: May 4, 2020
 *      Author: Mohamed  Ebead
 */


#include "drivers/inc/TivaC_peripherals.h"



void SSI_GPIO_Initialize (void)
{

    GPIO_Handler_t SSI0_Pins ;
    SSI0_Pins.pGPIOx = SSI0_GPIO ;

    SSI0_Pins.pGPIO_PinConfig->GPIO_PinMode = GPIO_MODE_ALTFN ;
    SSI0_Pins.pGPIO_PinConfig->GPIO_PinAltFunMode = 5 ;
    SSI0_Pins.pGPIO_PinConfig->GPIO_PinOPType = GPIO_SIGNAL_DIGITAL ;
    SSI0_Pins.pGPIO_PinConfig->GPIO_PinDrivStren = GPIO_DRIVSTR_MIN ;
    SSI0_Pins.pGPIO_PinConfig->GPIO_PinOPType= GPIO_OP_TYPE_NOPUPDOD ;
    SSI0_Pins.pGPIO_PinConfig->GPIO_PinIntMode= GPIO_INT_DIS ;

    //SCK
    SSI0_Pins.pGPIO_PinConfig->GPIO_PinNumber = GPIOA_PIN_2 ;
    GPIO_Init (&SSI0_Pins) ;


    //CS0
    SSI0_Pins.pGPIO_PinConfig->GPIO_PinNumber = GPIOA_PIN_3 ;
    GPIO_Init (&SSI0_Pins) ;

    // MISO
    SSI0_Pins.pGPIO_PinConfig->GPIO_PinNumber = GPIOA_PIN_4 ;
    GPIO_Init (&SSI0_Pins) ;

    //MOSI
    SSI0_Pins.pGPIO_PinConfig->GPIO_PinNumber = GPIOA_PIN_5 ;
    GPIO_Init (&SSI0_Pins) ;


}


/**------------------------------------------------------------------------**/

void SSI_Initialize (void)
{
    SSI_Handler_t SSI0_Handle ;

    SSI0_Handle.pSSIx = SSI0 ;
    SSI0_Handle.pSSI_Config->SSI_DeviceMode = SSI_DEVICE_MODE_MASTER ;
    SSI0_Handle.pSSI_Config->SSI_DFF = SSI_FF_FREESCALE_SPI ;
    SSI0_Handle.pSSI_Config->SSI_DSS = SSI_DSS_8BITS ;
    SSI0_Handle.pSSI_Config->SSI_SClkRate = SSI_SCLK_RATE_DIV2 ;
    SSI0_Handle.pSSI_Config->SSI_CPHA = SSI_CPHA_LOW;
    SSI0_Handle.pSSI_Config->SSI_CPOL = SSI_CPOL_LOW ;

    SSI_Init(&SSI0_Handle) ;


}











