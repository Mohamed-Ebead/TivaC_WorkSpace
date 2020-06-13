/*
 * app.c
 *
 *  Created on: May 2, 2020
 *      Author: Mohamed  Ebead
 */




/**
 * main.c
 */


#include "drivers/inc/TivaC_peripherals.h"

#include <string.h>
#include "inc/GPIO_interface.h"
#include "inc/SSI_interface.h"






void main(void)
{

/*   SSI Prog
    char user_data[] = "Hello World" ;
      SPI_SendData_Blocking(SSI0 ,(u_int_8*)user_data,strlen(user_data) ) ;
*/

   while (1)
   {



   }






}





void GPIOF_Handler (void)
{

    GPIOx_IRQHandling( GPIOF , GPIO_PIN_NO_0) ;

}



