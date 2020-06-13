/*
 * SSI_driver_prog.c
 *
 *  Created on: May 4, 2020
 *      Author: Mohamed  Ebead
 */

#include "drivers/inc/TivaC_peripherals.h"



/*-------------------------------------------------------------------------------------*/
// Peripheral Clock Setup
void SSI_ClockCtrl (SSI_RegDef_t *pSSIx , u_int_8 EnOrDis)
{
    if (EnOrDis == ENABLE)
    {
        if (pSSIx == SSI0)
        {
            SSI0_PCLK_EN()   ;

        }
        else if (pSSIx == SSI1)
        {
            SSI1_PCLK_EN()   ;

        }
        else if (pSSIx == SSI2)
        {
            SSI2_PCLK_EN()   ;

        }
        else if (pSSIx == SSI3)
        {
            SSI3_PCLK_EN()   ;

        }
        else
        {
            //error SSI base address
        }

    }
    else if (EnOrDis == DISABLE)
    {
        if (pSSIx == SSI0)
        {
            SSI0_PCLK_DIS()   ;

        }
        else if (pSSIx == SSI1)
        {
            SSI1_PCLK_DIS()   ;

        }
        else if (pSSIx == SSI2)
        {
            SSI2_PCLK_DIS()   ;

        }
        else if (pSSIx == SSI3)
        {
            SSI3_PCLK_DIS()   ;

        }
        else
        {
            //error SSI base address
        }

    }
    else
    {
        // error ENABLE or DISABLE macro
    }

    asm(" NOP ") ;   // No Operation just wait for clock stable

}

/*-------------------------------------------------------------------------------------*/

// Initialize and DeInitialize
void SSI_Init   (SSI_Handler_t *pSSIxHandler )
{
    // 1. Enable the SSI module using the RCGCSSI register
    SSI_ClockCtrl (pSSIxHandler->pSSIx , ENABLE) ;


    /*------------------------------------------------------------------------*/

    /*------------------------------------------------------------------------*/

    // (B) SSI Config

    // 0. disable SSI until you finish the configuration
    pSSIxHandler->pSSIx->CR1 &= ~(1<<SSI_CR1_SSE) ;

    // 1. select device mode  : Master or Slave
    pSSIxHandler->pSSIx->CR1 |= ( (pSSIxHandler->pSSI_Config->SSI_DeviceMode) << SSI_CR1_MS ) ;


    // 2. Bus configuration



    // 3. Clock Rate

    /*  BR( bit rate )=SysClk/(CPSDVSR * (1 + SCR))
     *  we will keep SCR as 0 and configure CPSDVSR prescaler
     *  then  bit rate =SysClk/(CPSDVSR)
     *  CPSDVSR stands for clock prescaler devider
     */
    pSSIxHandler->pSSIx->CPSR =( (pSSIxHandler->pSSI_Config->SSI_SClkRate) << 0 ) ;


    // 4. Data Frame Format
    if ( ( pSSIxHandler->pSSI_Config->SSI_DFF ) == SSI_FF_FREESCALE_SPI )
    {
        pSSIxHandler->pSSIx->CR0 &= ~( (1<<4) | (1<<5) ) ;    // Freescale SPI Frame Format

    }
    else if ( ( pSSIxHandler->pSSI_Config->SSI_DFF ) == SSI_FF_TEXAS_INSTRUMENTS )
    {
        pSSIxHandler->pSSIx->CR0 |=  (1<<4)  ;
        pSSIxHandler->pSSIx->CR0 &= ~(1<<5)  ;    // Texas Instruments Synchronous Serial Frame Format

    }
    else if ( ( pSSIxHandler->pSSI_Config->SSI_DFF ) == SSI_FF_MICROWAVE )
    {
        pSSIxHandler->pSSIx->CR0 |= ( (1<<4) | (1<<5) ) ;    // MICROWIRE Frame Format

    }
    else
    {
        // invalid frame format
    }

    // 5. Data size select
    pSSIxHandler->pSSIx->CR0 |= ( (pSSIxHandler->pSSI_Config->SSI_DSS)<< 0 ) ;

    // 6. clock polarity
    // when data is not being transferred.
    pSSIxHandler->pSSIx->CR0  |= ( (pSSIxHandler->pSSI_Config->SSI_CPOL) << SSI_CR0_SPO ) ;

    // 7. clock phase
    pSSIxHandler->pSSIx->CR0  |= ( (pSSIxHandler->pSSI_Config->SSI_CPHA) << SSI_CR0_SPH ) ;


    // 8. Software Slave Management







    // finally , enable SSI
    pSSIxHandler->pSSIx->CR1 |= (1<<SSI_CR1_SSE) ;












}

/*-------------------------------------------------------------------------------------*/

void SSI_DeInit (SSI_RegDef_t *pSSIx )
{
    //  disable SSI
    pSSIx->CR1 &= ~(1<<SSI_CR1_SSE) ;

    //  DISABLE the SSI module using the RCGCSSI register
    SSI_ClockCtrl (pSSIx , DISABLE) ;


}

/*-------------------------------------------------------------------------------------*/

// Data Send and Recive
void SPI_SendData_Blocking (SSI_RegDef_t *pSSIx , u_int_8 *pTXBuffer , u_int_32 Length )
{
    while (Length > 0 )
    {

        while ( !(pSSIx->SR & (1<< SSI_SR_TFE) ) ) ; // Wait for The transmit FIFO becomes empty

        if ( ( (pSSIx->CR0)&0x000F) == SSI_DSS_8BITS )
        {
            pSSIx->DR = *pTXBuffer ;
            Length -- ;
            (u_int_16*)pTXBuffer ++ ;

        }
        else if ( ( (pSSIx->CR0)&0x000F) == SSI_DSS_16BITS )
        {
            pSSIx->DR = *((u_int_16*)pTXBuffer) ;
            Length -- ;
            Length -- ;
            (u_int_16*)pTXBuffer ++ ;
        }
        else
        {
            // handle other data size selected
        }


    }


}

/*-------------------------------------------------------------------------------------*/

void SPI_RecieveData_Blocking (SSI_RegDef_t *pSSIx , u_int_8 *pRXBuffer , u_int_32 Length )
{
    while (Length > 0 )
    {

        while ( !(pSSIx->SR & (1<< SSI_SR_RFF) ) ) ; // Wait for The receive FIFO is full

        if ( ( (pSSIx->CR0)&0x000F) == SSI_DSS_8BITS )
        {
            // load data from DR to RX buffer
             *pRXBuffer = pSSIx->DR ;
            Length -- ;
            (u_int_16*)pRXBuffer ++ ;

        }
        else if ( ( (pSSIx->CR0)&0x000F) == SSI_DSS_16BITS )
        {
            *((u_int_16*)pRXBuffer) = pSSIx->DR  ;
            Length -- ;
            Length -- ;
            (u_int_16*)pRXBuffer ++ ;
        }
        else
        {
            // handle other data size selected
        }


       }


}

/*-------------------------------------------------------------------------------------*/
// not completed ................

void SPI_SendData_NonBlocking (SSI_Handler_t *pSSIxHandler , u_int_8 *pTXBuffer , u_int_32 Length )
{

}

/*-------------------------------------------------------------------------------------*/

void SPI_RecieveData_NonBlocking (SSI_Handler_t *pSSIxHandler , u_int_8 *pRXBuffer , u_int_32 Length )
{

}

/*-------------------------------------------------------------------------------------*/


// IRQ configuration and ISR handling
void SSI_IRQInterruptConfig (u_int_8 IRQNumber , u_int_8 EnOrDis)
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

/*-------------------------------------------------------------------------------------*/

void SSI_IRQPriorityConfig (u_int_8 IRQNumber , u_int_8 IRQPriority)
{
    u_int_8 IPRx = IRQNumber / 4 ;
    u_int_8 IPRx_Section = IRQNumber % 4 ;
    u_int_8 ShiftAmount = (8 * IPRx_Section ) + (8 - NB_PR_BITS_RESERVED ) ;
    NVIC->IPR[IPRx] |= (IRQPriority << ShiftAmount ) ;

}

/*-------------------------------------------------------------------------------------*/

void SSIx_IRQHandling (SSI_Handler_t *pSSIxHandler )
{

}

/*-------------------------------------------------------------------------------------*/


