/*
 * CAN_driver_prog.c
 *
 *  Created on: May 9, 2020
 *      Author: Mohamed  Ebead
 */

#include  "drivers/inc/TivaC_peripherals.h"



/*-----------------------------------------------------------------------------------------------*/

void CAN_ClkCtrl (CAN_RegDef_t *pCANx , u_int_8 ENOrDIS )
{
    if (ENOrDIS == ENABLE)
    {
        if ( pCANx == CAN0 )
        {
            CAN0_PCLK_EN();

        }
        else if ( pCANx == CAN1 )
        {
            CAN1_PCLK_EN();

        }
        else
        {
            // invalid CAN peripheral
        }

    }
    else if (ENOrDIS == DISABLE)
    {
        if ( pCANx == CAN0 )
        {
            CAN0_PCLK_DIS();

        }
        else if ( pCANx == CAN1 )
        {
            CAN1_PCLK_DIS();

        }
        else
        {
            // invalid CAN peripheral
        }

    }
    else
    {
        // invalid ENABLE or DISABLE macro
    }

}

/*-----------------------------------------------------------------------------------------------*/

void CAN_Init (CAN_Handler_t *pCANx_Handler)
{
    // 0. enable clock for CAN peripheral
    CAN_ClkCtrl (pCANx_Handler->CAN , ENABLE ) ;

    // 1. set the INIT bit in the CAN Control (CANCTL) register
    pCANx_Handler->CAN->CTL |= (1<<CAN_CTL_INIT) ;  // Initialization started.

    // 2.  set the CAN Bit Timing (CANBIT) register and configure each message object.
    // If a message object is not needed, label it as not valid by clearing the MSGVAL bit in CANIFnARB2
    if ( (pCANx_Handler->CAN_Config->MsgObject) == CAN_MSG_OBJ_NEEDED)
    {
        //  BRP   = BRP
       pCANx_Handler->CAN->BIT |= ( (pCANx_Handler->CAN_Config->BRP) <<CAN_BIT_PRB ) ;

       // SJW   = SJW - 1
       u_int_8 SJW = ((pCANx_Handler->CAN_Config->SJW) -1) ;
       pCANx_Handler->CAN->BIT |= ( (SJW) <<CAN_BIT_SJW ) ;

       // TSEG1  = Prop + Phase1 - 1
       u_int_8 TSEG1 = ((pCANx_Handler->CAN_Config->Prob)+(pCANx_Handler->CAN_Config->Phase1) -1) ;
       pCANx_Handler->CAN->BIT |= ( (TSEG1) <<CAN_BIT_TSEG1 ) ;

       // TSEG2  = Phase2 - 1
       u_int_8 TSEG2 = ((pCANx_Handler->CAN_Config->Phase2) -1) ;
       pCANx_Handler->CAN->BIT |= ( (TSEG2) <<CAN_BIT_TSEG2 ) ;

    }
    else
    {
        if ( (pCANx_Handler->CAN) == CAN0 )
        {
            pCANx_Handler->CAN->IF1ARB2 &= ~(1<< CAN_IF1ARB2_MSGVAL) ;

        }
        else if ( (pCANx_Handler->CAN) == CAN1 )
        {
            pCANx_Handler->CAN->IF1ARB2 &= ~(1<< CAN_IF2ARB2_MSGVAL) ;

        }
        else
        {
            // invalid CAN peripheral
        }
    }

    /*  Both the INIT and CCE bits in the CANCTL register must be set in order to access the
        CANBIT register and the CAN Baud Rate Prescaler Extension (CANBRPE) register to configure
        the bit timing
    */

    // set CCE bits in the CANCTL register
    pCANx_Handler->CAN->CTL |= (1<<CAN_CTL_CCE) ;

    // configure the CAN Baud Rate Prescaler Extension
    pCANx_Handler->CAN->BRPE = ( pCANx_Handler->CAN_Config->BRPE ) ;

    //  To leave the initialization state, the INIT bit must be cleared
    pCANx_Handler->CAN->CTL &= ~(1<<CAN_CTL_INIT) ;


}

/*-----------------------------------------------------------------------------------------------*/
