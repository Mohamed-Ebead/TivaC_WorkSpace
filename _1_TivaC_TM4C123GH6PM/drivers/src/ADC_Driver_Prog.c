/*
 * ADC_Driver_Prog.c
 *
 *  Created on: May 7, 2020
 *      Author: Mohamed  Ebead
 */

#include "drivers/inc/TivaC_peripherals.h"


/*--------------------------------------------------------------*/

void ADC_ClkCtrl ( ADC_RegDef_t *pADCx )
{
    if ( pADCx == ADC0 )
    {
        ADC0_PCLK_DIS() ;

    }
    else if ( pADCx == ADC1 )
    {
        ADC1_PCLK_DIS() ;

    }
    else
    {
        // invalid ADC peripheral
    }

}

/*--------------------------------------------------------------*/

void ADC_Init ( ADC_Handler_t *pADCx_handler )
{
    /*---------------------------------------------------------------------------*/
    // local variables to hold number of samples in each sample sequencer
    u_int_8 SS0_Sample_NB = 0 ;    // max 8
    u_int_8 SS1_Sample_NB = 0 ;    // max 4
    u_int_8 SS2_Sample_NB = 0 ;    // max 4
    u_int_8 SS3_Sample_NB = 0 ;    // max 1

    // 0. enable clock for ADC peripheral
    ADC_ClkCtrl ( pADCx_handler->pADC ) ;

    // initialize GPIO ADC pin

    // Sample Sequencer Configuration
    // 0. Disable the sample sequenser before configuring it
    pADCx_handler->pADC->ACTSS &= ~(1<< (pADCx_handler->pADC_Config->ADC_SampleSequencer) ) ;

    /*---------------------------------------------------------------------------*/

    // 1.configure the trigger event
    pADCx_handler->pADC->EMUX |= ( (pADCx_handler->pADC_Config->ADC_Trigger)<<(4*(pADCx_handler->pADC_Config->ADC_SampleSequencer)) ) ;

    /*---------------------------------------------------------------------------*/

    // 2. if PWM trigger is enabled , select which PWM module triggers (default : PWM0)
    if ( (pADCx_handler->pADC_Config->ADC_Trigger) == ADC_TRIG_PWM0 )
    {
        pADCx_handler->pADC->TSSEL |= ( (pADCx_handler->pADC_Config->ADC_Trig_PWM_Module) <<AD_TSSEL_PS0 ) ;
    }
    else if ( (pADCx_handler->pADC_Config->ADC_Trigger) == ADC_TRIG_PWM1 )
    {
        pADCx_handler->pADC->TSSEL |= ( (pADCx_handler->pADC_Config->ADC_Trig_PWM_Module) <<AD_TSSEL_PS1 ) ;
    }
    else if ( (pADCx_handler->pADC_Config->ADC_Trigger) == ADC_TRIG_PWM2 )
    {
        pADCx_handler->pADC->TSSEL |= ( (pADCx_handler->pADC_Config->ADC_Trig_PWM_Module) <<AD_TSSEL_PS2 ) ;
    }
    else if ( (pADCx_handler->pADC_Config->ADC_Trigger) == ADC_TRIG_PWM3 )
    {
        pADCx_handler->pADC->TSSEL |= ( (pADCx_handler->pADC_Config->ADC_Trig_PWM_Module) <<AD_TSSEL_PS3 ) ;
    }
    else
    {
        // non PWM Triggered
    }
    /*---------------------------------------------------------------------------*/
    // 3. Select the ADC input channel using the ADCSSMUXn register
    if ( (pADCx_handler->pADC_Config->ADC_SampleSequencer) == ADC_SAMPSEQ_0 )
    {
        if (SS0_Sample_NB <8)
        {
            pADCx_handler->pADC->SSMUX0 |= ( (pADCx_handler->pADC_Config->ADC_Channel) << (4* SS0_Sample_NB)) ;

            //  set END as it is the  last nipple
            if ( ( pADCx_handler->pADC_Config->ADC_LastSample ) == ADC_LAST_SAMPLE )
            {
                pADCx_handler->pADC->DCCTL0 |= (1<< ((4* SS0_Sample_NB)+1)) ;

            }
            else
            {
                // not last nipple
            }

            SS0_Sample_NB ++ ;
        }
        else
        {
            //  sample sequencer is full with 8 samples
        }

    }
    else if ( (pADCx_handler->pADC_Config->ADC_SampleSequencer) == ADC_SAMPSEQ_1 )
    {
        if (SS1_Sample_NB <4)
        {
            pADCx_handler->pADC->SSMUX1 |= ( (pADCx_handler->pADC_Config->ADC_Channel) << (4* SS1_Sample_NB)) ;
            //  set END as it is the  last nipple
            if ( ( pADCx_handler->pADC_Config->ADC_LastSample ) == ADC_LAST_SAMPLE )
            {
                pADCx_handler->pADC->DCCTL1 |= (1<< ((4* SS0_Sample_NB)+1)) ;

            }
            else
            {
                // not last nipple
            }
            SS1_Sample_NB ++ ;
        }
        else
        {
            //  sample sequencer is full with 8 samples
        }

    }
    else if ( (pADCx_handler->pADC_Config->ADC_SampleSequencer) == ADC_SAMPSEQ_2 )
    {
        if (SS2_Sample_NB <4)
        {
            pADCx_handler->pADC->SSMUX2 |= ( (pADCx_handler->pADC_Config->ADC_Channel) << (4* SS2_Sample_NB)) ;
            //  set END as it is the  last nipple
            if ( ( pADCx_handler->pADC_Config->ADC_LastSample ) == ADC_LAST_SAMPLE )
            {
                pADCx_handler->pADC->DCCTL2 |= (1<< ((4* SS0_Sample_NB)+1)) ;

            }
            else
            {
                // not last nipple
            }
            SS2_Sample_NB ++ ;
        }
        else
        {
            //  sample sequencer is full with 8 samples
        }

    }
    else if ( (pADCx_handler->pADC_Config->ADC_SampleSequencer) == ADC_SAMPSEQ_3 )
    {
        if (SS3_Sample_NB <1)
        {
            pADCx_handler->pADC->SSMUX3 |= ( (pADCx_handler->pADC_Config->ADC_Channel) << (4* SS3_Sample_NB)) ;
            //  set END as it is the  last nipple
            if ( ( pADCx_handler->pADC_Config->ADC_LastSample ) == ADC_LAST_SAMPLE )
            {
                pADCx_handler->pADC->DCCTL3 |= (1<< ((4* SS0_Sample_NB)+1)) ;

            }
            else
            {
                // not last nipple
            }
            SS3_Sample_NB ++ ;
        }
        else
        {
            //  sample sequencer is full with 8 samples
        }

    }
    else
    {
        // invalid sample sequencer
    }

    /*---------------------------------------------------------------------------*/
    // 4.






    // finally Enable the sample sequencer logic by setting the corresponding ASENn bit in the ADCACTSS register
    pADCx_handler->pADC->ACTSS|= (1<< (pADCx_handler->pADC_Config->ADC_SampleSequencer) ) ;




}

/*--------------------------------------------------------------*/

u_int_16 ADC_Read_Blocking ( ADC_Handler_t *pADCx_handler )
{

    u_int_16 ADC_Read = 0 ;

// assign ADC_Read


return ADC_Read ;
}

/*--------------------------------------------------------------*/

void ADC_Read_NonBlocking ( ADC_Handler_t *pADCx_handler )
{

}

/*--------------------------------------------------------------*/



