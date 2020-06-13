/*
 * GPIO_driver.c
 *
 *  Created on: Apr 27, 2020
 *      Author: Mohamed  Ebead
 */

#include "drivers/inc/STD_TYPES.h"
#include "drivers/inc/GPIO_driver_interface.h"
//#include "TivaC_peripherals.h"


/*---------------------------------------------------------------------------------------------------*/
// Peripheral Clock Setup
/*****************************************************************************************************
 * @fn          :   GPIO_ClockCtrl
 * @breif       :   this function enables or disables peripheral clock for given GPIO
 * @param[in]   :   base address of the GPIO peripheral
 * @param[in]   :   macro : " ENABLE " or " DISABLE "
 * @return      :   none
 * @note        :
 *****************************************************************************************************
 */
void GPIO_ClockCtrl (GPIO_RegDef_t *pGPIOx , u_int_8 EnOrDis)
{
    if ( EnOrDis == ENABLE )
    {
        if ( pGPIOx == GPIOA )
        {
            GPIOA_PCLK_EN() ;
        }
        else if  ( pGPIOx == GPIOB )
        {
            GPIOB_PCLK_EN() ;
        }
        else if  ( pGPIOx == GPIOC )
        {
            GPIOC_PCLK_EN() ;
        }
        else if  ( pGPIOx == GPIOD )
        {
            GPIOD_PCLK_EN() ;
        }
        else if  ( pGPIOx == GPIOE )
        {
            GPIOE_PCLK_EN() ;
        }
        else if  ( pGPIOx == GPIOF )
        {
            GPIOF_PCLK_EN() ;
        }
        else
        {
            // error passing invalid GPIO parameter
        }
    }
    else if  ( EnOrDis == DISABLE )
    {
        if ( pGPIOx == GPIOA )
        {
            GPIOA_PCLK_DIS() ;
        }
        else if  ( pGPIOx == GPIOB )
        {
            GPIOB_PCLK_DIS() ;
        }
        else if  ( pGPIOx == GPIOC )
        {
            GPIOC_PCLK_DIS() ;
        }
        else if  ( pGPIOx == GPIOD )
        {
            GPIOD_PCLK_DIS() ;
        }
        else if  ( pGPIOx == GPIOE )
        {
            GPIOE_PCLK_DIS() ;
        }
        else if  ( pGPIOx == GPIOF )
        {
            GPIOF_PCLK_DIS() ;
        }
        else
        {
            // error passing invalid GPIO parameter
        }
    }
    else
    {
            // error passing invalid EnOrDis macro parameter
     }


    asm(" NOP ") ;   // No Operation just wait for clock stable

}

/*---------------------------------------------------------------------------------------------------*/

// Initialize and DeInitialize
/*****************************************************************************************************
 * @fn          :   GPIO_Init
 * @breif       :   this function initializes a GPIO peripheral
 * @param[in]   :
 * @return      :   none
 * @note        :
 *****************************************************************************************************
 */
void GPIO_Init   (GPIO_Handler_t *pGPIOHandler )
{
    //0. unlock GPIO Commit (GPIOCR) register for write access on GPIOAFSEL, GPIOPUR, GPIOPDR, or GPIODEN
    pGPIOHandler->pGPIOx->LOCK = 0x4C4F434B ;
    pGPIOHandler->pGPIOx->CR  |= ( 1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber) ) ;

    //1. enable the clock
    GPIO_ClockCtrl ( pGPIOHandler->pGPIOx  ,ENABLE) ;
/*--------------------------------------------------------------------------------------------------------------*/
    //2. set the direction  (GPIODIR)  default : input
	if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinDir) == GPIO_DIR_IN )
	{
	    pGPIOHandler->pGPIOx->DIR &= ~(1<< (pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;
	}
	else if ( ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinDir) == GPIO_DIR_OUT ))
	{
	    pGPIOHandler->pGPIOx->DIR |= (1<< (pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;
	}
	else
	{
	    // error configurating PIN direction
	}

/*--------------------------------------------------------------------------------------------------------------*/

    //3. configure each bit type (GPIOAFSEL for GPIO or Alternate pin) , GPIOADCCTL for Anaolg , GPIODMACTL for uDMA default GPIO
	if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinMode) == GPIO_MODE_GPIO )
	{
	    pGPIOHandler->pGPIOx->AFSEL &= ~(1<< (pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;
	}
	else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinMode) == GPIO_MODE_ALTFN )
	{
	    pGPIOHandler->pGPIOx->AFSEL |= (1<< (pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;
	}
	else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinMode) == GPIO_MODE_ANALOG )
    {
        pGPIOHandler->pGPIOx->ADCCTL |= (1<< (pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;
    }
    else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinMode) == GPIO_MODE_UDMA )
    {
        pGPIOHandler->pGPIOx->DMACTL |= (1<< (pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;
    }
	else
	{
	    // error configurating PIN mode
	}

/*--------------------------------------------------------------------------------------------------------------*/

    //4. Set the drive strength for each of the pins through the GPIODR2R, GPIODR4R, and GPIODR8R default 2 mA
	if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinDrivStren) == GPIO_DRIVSTR_MIN )
    {
        pGPIOHandler->pGPIOx->DR2R |= (1<< (pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;
    }
    else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinDrivStren) == GPIO_DRIVSTR_MID )
    {
        pGPIOHandler->pGPIOx->DR4R |= (1<< (pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;
    }
    else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinDrivStren) == GPIO_DRIVSTR_MAX )
    {
        pGPIOHandler->pGPIOx->DR8R |= (1<< (pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;
    }
    else
    {
        // error configurating PIN drive strength
    }

/*--------------------------------------------------------------------------------------------------------------*/

    //5. program each pad for pull up , down or open drain
	if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinOPType) == GPIO_OP_TYPE_NOPUPDOD )
	{
	    pGPIOHandler->pGPIOx->PUR &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber )) ;
	    pGPIOHandler->pGPIOx->PDR &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber )) ;
	    pGPIOHandler->pGPIOx->ODR &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber )) ;
	}
	else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinOPType) == GPIO_OP_TYPE_PU )
	{
	    pGPIOHandler->pGPIOx->PUR |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber )) ;
	}
	else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinOPType) == GPIO_OP_TYPE_PD )
	{
	    pGPIOHandler->pGPIOx->PDR |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber )) ;
	}
	else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinOPType) == GPIO_OP_TYPE_OD )
	{
	    pGPIOHandler->pGPIOx->ODR |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber )) ;
	}
	else
	{
	    // error configurating output type
	}

/*--------------------------------------------------------------------------------------------------------------*/

    //6. enable the GPIO pin ad digital (GPIODEN) or analog (GPIOAMSEL)
	if ((pGPIOHandler->pGPIO_PinConfig->GPIO_PinSignalType) == GPIO_SIGNAL_DIGITAL )
	{
	    pGPIOHandler->pGPIOx->DEN |= ( 1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber) ) ;
	}
	else if ((pGPIOHandler->pGPIO_PinConfig->GPIO_PinSignalType) == GPIO_SIGNAL_ANALOG )
	{
	    pGPIOHandler->pGPIOx->DEN &= ~( 1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber) ) ; //disable digital
	    pGPIOHandler->pGPIOx->AMSEL = ( 1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber) ) ; // disable isolation of ADC

	}
	else
	{
	    // error configurating signal type
	}

/*--------------------------------------------------------------------------------------------------------------*/
	//7. Interrupt configurations
	/*
	 * steps :
	 * 1. mask the interrupt
	 * 2. configure it
	 * 3. clear roe interrupt status
	 * 4. unmask the interrupt
	 */

	if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinIntMode) == GPIO_INT_DIS )
	{
	    pGPIOHandler->pGPIOx->IM  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ; // mask the interrupt
	}
	else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinIntMode) == GPIO_INT_FT )
	{
        pGPIOHandler->pGPIOx->IM  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // mask the interrupt
	    pGPIOHandler->pGPIOx->IS  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // edge senstive
	    pGPIOHandler->pGPIOx->IBE &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // one edge
	    pGPIOHandler->pGPIOx->IEV &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // falling edge
        pGPIOHandler->pGPIOx->RIS &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // clear interrupts
	    pGPIOHandler->pGPIOx->IM |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber))   ;  // unmask the interrupt
	}
	else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinIntMode) == GPIO_INT_RT )
	{
        pGPIOHandler->pGPIOx->IM  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;   // mask the interrupt
        pGPIOHandler->pGPIOx->IS  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // edge senstive
        pGPIOHandler->pGPIOx->IBE &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // one edge
        pGPIOHandler->pGPIOx->IEV |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber))  ;   // rising edge
        pGPIOHandler->pGPIOx->RIS &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // clear interrupts
        pGPIOHandler->pGPIOx->IM |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber))   ;  // unmask the interrupt

	}
	else if ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinIntMode) == GPIO_INT_RFT )
	{
        pGPIOHandler->pGPIOx->IM  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // mask the interrupt
        pGPIOHandler->pGPIOx->IS  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // edge senstive
        pGPIOHandler->pGPIOx->IBE |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber))  ;  // both edges
        pGPIOHandler->pGPIOx->RIS &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // clear interrupts
        pGPIOHandler->pGPIOx->IM  |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber))  ;  // unmask the interrupt

	}
	else if  ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinIntMode) == GPIO_INT_HL )
	{
        pGPIOHandler->pGPIOx->IM  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // mask the interrupt
        pGPIOHandler->pGPIOx->IS  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // edge senstive
        pGPIOHandler->pGPIOx->IEV |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber))  ;  // high level
        pGPIOHandler->pGPIOx->RIS &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // clear interrupts
        pGPIOHandler->pGPIOx->IM |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber))   ;  // unmask the interrupt

	}
	else if  ( (pGPIOHandler->pGPIO_PinConfig->GPIO_PinIntMode) == GPIO_INT_LL )
	{
        pGPIOHandler->pGPIOx->IM  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ; // mask the interrupt
        pGPIOHandler->pGPIOx->IS  &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // edge senstive
        pGPIOHandler->pGPIOx->IEV &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // low level
        pGPIOHandler->pGPIOx->RIS &= ~(1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber)) ;  // clear interrupts
        pGPIOHandler->pGPIOx->IM  |= (1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber))  ; // unmask the interrupt

	}
	else
	{
	    // error configurating interrupt mode
	}

	 //8. lock GPIO Commit (GPIOCR) register for write access on GPIOAFSEL, GPIOPUR, GPIOPDR, or GPIODEN
	pGPIOHandler->pGPIOx->CR   &= ~( 1<<(pGPIOHandler->pGPIO_PinConfig->GPIO_PinNumber) ) ;
	pGPIOHandler->pGPIOx->LOCK = 0 ;



}

/*---------------------------------------------------------------------------------------------------*/
/*****************************************************************************************************
 * @fn          :   GPIO_DeInit
 * @breif       :   this function Deinitializes a GPIO peripheral
 * @param[in]   :
 * @return      :   none
 * @note        :
 *****************************************************************************************************
 */
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx )
{

}

/*---------------------------------------------------------------------------------------------------*/

// Data Read and Write
/*****************************************************************************************************
 * @fn          :   GPIO_ReadFromInputPin
 * @breif       :   this function reads ( 0 or 1 ) from an input pin
 * @param[in]   :   base address of the GPIO peripheral
 * @param[in]   :   Pin Number to be read
 * @return      :   ' 0 ' or ' 1 '
 * @note        :
 *****************************************************************************************************
 */

u_int_8  GPIO_ReadFromInputPin  (GPIO_RegDef_t *pGPIOx , u_int_8 u8_PinNumber)
{
    return (u_int_8)( ((pGPIOx->DATA) >> u8_PinNumber ) & 0x00000001 ) ;
}

/*---------------------------------------------------------------------------------------------------*/

/*****************************************************************************************************
 * @fn          :   GPIO_ReadFromInputPort
 * @breif       :   this function reads from an input port
 * @param[in]   :   base address of the GPIO peripheral
 * @return      :   8-bit +ve value
 * @note        :
 *****************************************************************************************************
 */

u_int_8 GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx)
{
    return (u_int_8) pGPIOx->DATA ;
}

/*---------------------------------------------------------------------------------------------------*/

/*****************************************************************************************************
 * @fn          :   GPIO_WriteToOutputPin
 * @breif       :   this function writes 1 to an output pin
 * @param[in]   :   base address of the GPIO peripheral
 * @param[in]   :   Pin Number to be set
 * @return      :   none
 * @note        :
 *****************************************************************************************************
 */

void     GPIO_SetOutputPin  (GPIO_RegDef_t *pGPIOx , u_int_8 u8_PinNumber )
{
    pGPIOx->DATA |= (1<<u8_PinNumber ) ;
}
/*---------------------------------------------------------------------------------------------------*/

/*****************************************************************************************************
 * @fn          :   GPIO_WriteToOutputPin
 * @breif       :   this function writes 0  to an output pin
 * @param[in]   :   base address of the GPIO peripheral
 * @param[in]   :   Pin Number to be reset
 * @return      :   none
 * @note        :
 *****************************************************************************************************
 */

void     GPIO_ResetOutputPin  (GPIO_RegDef_t *pGPIOx , u_int_8 u8_PinNumber )
{
    pGPIOx->DATA &=~ (1<<u8_PinNumber ) ;
}

/*---------------------------------------------------------------------------------------------------*/

/*****************************************************************************************************
 * @fn          :   GPIO_WriteToOutputPort
 * @breif       :   this function writes 16-bit +ve value to an output port
 * @param[in]   :   base address of the GPIO peripheral
 * @param[in]   :   8-bit +ve value
 * @return      :   none
 * @note        :
 *****************************************************************************************************
 */

void     GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx , u_int_8 u8_Value)
{
    pGPIOx->DATA = u8_Value ;
}

/*---------------------------------------------------------------------------------------------------*/

/*****************************************************************************************************
 * @fn          :   GPIO_ToggleOutputPin
 * @breif       :
 * @param[in]   :   base address of the GPIO peripheral
 * @param[in]   :   Pin Number to be toggeled
 * @return      :   none
 * @note        :
 *****************************************************************************************************
 */

void     GPIO_ToggleOutputPin   (GPIO_RegDef_t *pGPIOx , u_int_8 u8_PinNumber )
{
    pGPIOx->DATA ^= (1<<u8_PinNumber ) ;
}

/*---------------------------------------------------------------------------------------------------*/

// IRQ configuration and ISR handling
/*****************************************************************************************************
 * @fn          :
 * @breif       :
 * @param[in]   :
 * @param[in]   :
 * @param[in]   :
 * @return      :
 * @note        :
 *****************************************************************************************************
 */

void GPIO_IRQInterruptConfig (u_int_8 IRQNumber , u_int_8 EnOrDis)
{
    if (EnOrDis == ENABLE )
    {
 /*---------------------------------------------------------------------------*/
        if (IRQNumber <= 31)
        {
            NVIC->ISER[0] |= ( 1<<IRQNumber ) ;

        }else if ((IRQNumber > 31)&&(IRQNumber <= 63))
        {
            NVIC->ISER[1] |= (1<<(IRQNumber %32)) ;

        }else if ((IRQNumber > 63)&&(IRQNumber <= 95))
        {
            NVIC->ISER[2] |= (1<<(IRQNumber %64)) ;

        }else if ((IRQNumber > 96)&&(IRQNumber <= 127))
        {
            NVIC->ISER[3] |= (1<<(IRQNumber %96)) ;

        }else if ((IRQNumber > 127)&&(IRQNumber <= 138))
        {
            NVIC->ISER[4] |= (1<<(IRQNumber %128)) ;

        }else
        {
            // invalid IRQNumber
        }
/*---------------------------------------------------------------------------*/
    }else if (EnOrDis == DISABLE )
    {
        if (IRQNumber <= 31)
        {
            NVIC->ICER[0] |= (1<<IRQNumber) ;

        }else if ((IRQNumber > 31)&&(IRQNumber <= 63))
        {
            NVIC->ICER[1] |= (1<<(IRQNumber %32)) ;


        }else if ((IRQNumber > 63)&&(IRQNumber <= 95))
        {
            NVIC->ICER[2] |= (1<<(IRQNumber %64)) ;


        }else if ((IRQNumber > 96)&&(IRQNumber <= 127))
        {
            NVIC->ICER[3] |= (1<<(IRQNumber %96)) ;


        }else if ((IRQNumber > 127)&&(IRQNumber <= 138))
        {
            NVIC->ICER[4] |= (1<<(IRQNumber %128)) ;


        }else
        {
            // invalid IRQNumber
        }
/*---------------------------------------------------------------------------*/
    }else
    {
        // invalid EnOrDis macro
    }

}

/*---------------------------------------------------------------------------------------------------*/

/*****************************************************************************************************
 * @fn          :
 * @breif       :
 * @param[in]   :
 * @param[in]   :
 * @param[in]   :
 * @return      :
 * @note        :
 *****************************************************************************************************
 */
void GPIO_IRQPriorityConfig (u_int_8 IRQNumber ,u_int_8 IRQPriority)
{
    u_int_8 IPRx = IRQNumber / 4 ;
    u_int_8 IPRx_Section = IRQNumber % 4 ;
    u_int_8 ShiftAmount = (8 * IPRx_Section ) + (8 - NB_PR_BITS_RESERVED ) ;
    NVIC->IPR[IPRx] |= (IRQPriority << ShiftAmount ) ;

}

/*---------------------------------------------------------------------------------------------------*/

/*****************************************************************************************************
 * @fn          :
 * @breif       :
 * @param[in]   :
 * @param[in]   :
 * @param[in]   :
 * @return      :
 * @note        :
 *****************************************************************************************************
 */

void GPIOx_IRQHandling( GPIO_RegDef_t *pGPIOx , u_int_8 u8_PinNumber)
{
    // Enter your program to be executed in ISR


    // clear GPIO Interrupt flag
    pGPIOx->ICR |= (1<<u8_PinNumber) ;

}

/*---------------------------------------------------------------------------------------------------*/


//asm ("CPSIE") ;   // global interrupt enable
//asm ("CPSID") ;  // global interrupt disable






