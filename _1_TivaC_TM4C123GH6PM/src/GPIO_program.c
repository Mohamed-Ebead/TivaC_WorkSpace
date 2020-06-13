/*
 * program.c
 *
 *  Created on: May 3, 2020
 *      Author: Mohamed  Ebead
 */

#include "drivers/inc/STD_TYPES.h"
#include "drivers/inc/GPIO_driver_interface.h"


/*---------------------------------------------------------------------*/

// delay half second for 16MHZ clock
void delay (void)
{
    u_int_32 i = 0 ;
    for (i = 0 ; i<500000 ; i++ ) ;
}

/*---------------------------------------------------------------------*/
void RedLed_GPIO_Initialize (void)
{
    // RED LED PF1
      GPIO_Handler_t RedLed ;
      RedLed.pGPIOx = GPIOF ;
      RedLed.pGPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_1 ;
      RedLed.pGPIO_PinConfig->GPIO_PinMode = GPIO_MODE_GPIO ;
      RedLed.pGPIO_PinConfig->GPIO_PinDir = GPIO_DIR_OUT ;
      RedLed.pGPIO_PinConfig->GPIO_PinOPType = GPIO_SIGNAL_DIGITAL ;
      RedLed.pGPIO_PinConfig->GPIO_PinDrivStren = GPIO_DRIVSTR_MIN ;
      RedLed.pGPIO_PinConfig->GPIO_PinOPType= GPIO_OP_TYPE_PD ;
      RedLed.pGPIO_PinConfig->GPIO_PinIntMode= GPIO_INT_DIS ;

      GPIO_Init (&RedLed) ;
}

/*---------------------------------------------------------------------*/
void BlueLed_GPIO_Initialize (void)
{
    // RED LED PF1
      GPIO_Handler_t BlueLed ;
      BlueLed.pGPIOx = GPIOF ;
      BlueLed.pGPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_2 ;
      BlueLed.pGPIO_PinConfig->GPIO_PinMode = GPIO_MODE_GPIO ;
      BlueLed.pGPIO_PinConfig->GPIO_PinDir = GPIO_DIR_OUT ;
      BlueLed.pGPIO_PinConfig->GPIO_PinOPType = GPIO_SIGNAL_DIGITAL ;
      BlueLed.pGPIO_PinConfig->GPIO_PinDrivStren = GPIO_DRIVSTR_MIN ;
      BlueLed.pGPIO_PinConfig->GPIO_PinOPType= GPIO_OP_TYPE_PD ;
      BlueLed.pGPIO_PinConfig->GPIO_PinIntMode= GPIO_INT_DIS ;

      GPIO_Init (&BlueLed) ;
}

/*---------------------------------------------------------------------*/

void GreenLed_GPIO_Initialize (void)
{
    // RED LED PF1
      GPIO_Handler_t GreenLed ;
      GreenLed.pGPIOx = GPIOF ;
      GreenLed.pGPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_3 ;
      GreenLed.pGPIO_PinConfig->GPIO_PinMode = GPIO_MODE_GPIO ;
      GreenLed.pGPIO_PinConfig->GPIO_PinDir = GPIO_DIR_OUT ;
      GreenLed.pGPIO_PinConfig->GPIO_PinOPType = GPIO_SIGNAL_DIGITAL ;
      GreenLed.pGPIO_PinConfig->GPIO_PinDrivStren = GPIO_DRIVSTR_MIN ;
      GreenLed.pGPIO_PinConfig->GPIO_PinOPType= GPIO_OP_TYPE_PD ;
      GreenLed.pGPIO_PinConfig->GPIO_PinIntMode= GPIO_INT_DIS ;

      GPIO_Init (&GreenLed) ;
}

/*---------------------------------------------------------------------*/
void RedLed_GPIO_Toggle (void)
{
    GPIO_ToggleOutputPin(GPIOF, GPIO_PIN_NO_1) ;
}


/*---------------------------------------------------------------------*/

void PushButton2_GPIO_INT_Initialize (void)
{
    GPIO_Handler_t Button ;
    Button.pGPIOx = GPIOF ;
    Button.pGPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_0 ;      //SW2
    Button.pGPIO_PinConfig->GPIO_PinMode = GPIO_MODE_GPIO ;
    Button.pGPIO_PinConfig->GPIO_PinDir = GPIO_DIR_IN ;
    Button.pGPIO_PinConfig->GPIO_PinOPType = GPIO_SIGNAL_DIGITAL ;
    Button.pGPIO_PinConfig->GPIO_PinDrivStren = GPIO_DRIVSTR_MIN ;
    Button.pGPIO_PinConfig->GPIO_PinOPType= GPIO_OP_TYPE_PU ;
    Button.pGPIO_PinConfig->GPIO_PinIntMode= GPIO_INT_FT ;

    GPIO_Init (&Button) ;
    GLOBAL_INT_EN();

}
/*---------------------------------------------------------------------*/












