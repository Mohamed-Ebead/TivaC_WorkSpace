/*
 * Timer_Driver.c
 *
 *  Created on: May 6, 2020
 *      Author: Mohamed  Ebead
 */

#include "drivers/inc/TivaC_peripherals.h"


/*-------------------------------------------------------------------------------------------------------*/

void Timer_ClkControl (Timer_RegDef_t *Timerx , u_int_8 ENOrDIS)
{
    if (ENOrDIS == ENABLE)
    {
        if (Timerx == TIMER0)
        {
            TIMER0_PCLK_EN();

        }
        else if (Timerx == TIMER1)
        {
            TIMER1_PCLK_EN();

        }
        else if (Timerx == TIMER2)
        {
            TIMER2_PCLK_EN();

        }
        else if (Timerx == TIMER3)
        {
            TIMER3_PCLK_EN();

        }
        else if (Timerx == TIMER4)
        {
            TIMER4_PCLK_EN();

        }
        else if (Timerx == TIMER5)
        {
            TIMER5_PCLK_EN();

        }
        else
        {
            // error Timer base address
        }

    }
    else if (ENOrDIS == DISABLE)
    {
        if (Timerx == TIMER0)
        {
            TIMER0_PCLK_DIS();

        }
        else if (Timerx == TIMER1)
        {
            TIMER1_PCLK_DIS();

        }
        else if (Timerx == TIMER2)
        {
            TIMER2_PCLK_DIS();

        }
        else if (Timerx == TIMER3)
        {
            TIMER3_PCLK_DIS();

        }
        else if (Timerx == TIMER4)
        {
            TIMER4_PCLK_DIS();

        }
        else if (Timerx == TIMER5)
        {
            TIMER5_PCLK_DIS();

        }

        else
        {
            // error Timer base address
        }

    }
    else
    {
        // invalid ENABLE or DISABLE  macro
    }

    asm(" NOP ") ;   // No Operation just wait for clock stable


}


/*-------------------------------------------------------------------------------------------------------*/
void W_Timer_ClkControl (W_Timer_RegDef_t *Timerx , u_int_8 ENOrDIS)
{
    if (ENOrDIS == ENABLE)
    {
        if (Timerx == WTIMER0)
        {
            WTIMER0_PCLK_EN();

        }
        else if (Timerx == WTIMER1)
        {
            WTIMER1_PCLK_EN();

        }
        else if (Timerx == WTIMER2)
        {
            WTIMER2_PCLK_EN();

        }
        else if (Timerx == WTIMER3)
        {
            WTIMER3_PCLK_EN();

        }
        else if (Timerx == WTIMER4)
        {
            WTIMER4_PCLK_EN();

        }
        else if (Timerx == WTIMER5)
        {
            WTIMER5_PCLK_EN();

        }
        else
        {
            // error Timer base address
        }

    }
    else if (ENOrDIS == DISABLE)
    {
        if (Timerx == WTIMER0)
        {
            WTIMER0_PCLK_DIS();

        }
        else if (Timerx == WTIMER1)
        {
            WTIMER1_PCLK_DIS();

        }
        else if (Timerx == WTIMER2)
        {
            WTIMER2_PCLK_DIS();

        }
        else if (Timerx == WTIMER3)
        {
            WTIMER3_PCLK_DIS();

        }
        else if (Timerx == WTIMER4)
        {
            WTIMER4_PCLK_DIS();

        }
        else if (Timerx == WTIMER5)
        {
            WTIMER5_PCLK_DIS();

        }
        else
        {
            // error Timer base address
        }

    }
    else
    {
       // invalid ENABLE or DISABLE  macro
    }

    asm(" NOP ") ;   // No Operation just wait for clock stable


}

/*-------------------------------------------------------------------------------------------------------*/


void Timer_Init_OneShot_Periodic (Timer_Handler_t *pTimerHandler )
{
    // 0. Enable Clock for timer peripheral
    Timer_ClkControl(pTimerHandler->pTimerx , ENABLE) ;

    // 1. Ensure the timer is disabled (the TnEN bit in the GPTMCTL register is cleared) before making any changes.

    pTimerHandler->pTimerx->CTL &= ~(1<<TIMER_CTL_TAEN) ; //disable Timer A
    pTimerHandler->pTimerx->CTL &= ~(1<<TIMER_CTL_TBEN) ; //disable Timer B


    if ( (pTimerHandler->pTimerx_Config->SplitOrConct) == TIMER_CFG_CONCT )
    {
        // Write the GPTM Configuration Register (GPTMCFG) with a value of 0x0000.0000
        pTimerHandler->pTimerx->CFG = ( 0 << TIMER_CFG_GPTMCFG ) ;  // 32 bit timer

        // set count direction
        if ( (pTimerHandler->pTimerx_Config->CountDir) == TIMER_COUNTDIR_UP )
        {
            pTimerHandler->pTimerx->TAMR |= (1<<TIMER_TAMR_TACDIR) ;
        }
        else
        {
            pTimerHandler->pTimerx->TAMR &= ~(1<<TIMER_TAMR_TACDIR) ;
        }

        // mode : one shot or periodic
        /* .Configure the TnMR field in the GPTM Timer n Mode Register (GPTMTnMR):
            a. Write a value of 0x1 for One-Shot mode.
            b. Write a value of 0x2 for Periodic mode.   */
        if ( (pTimerHandler->pTimerx_Config->Mode) == TIMER_MODE_ONESHOT )
        {
            pTimerHandler->pTimerx->TAMR |= (1<<TIMER_TAMR_TAMR ) ;
        }
        else if ( (pTimerHandler->pTimerx_Config->Mode) == TIMER_MODE_PERIODIC )
        {
            pTimerHandler->pTimerx->TBMR &= ~(1<<TIMER_TAMR_TAMR ) ;
        }
        else
        {
            // other configs
        }

        // prescaler can be used in 16 bit mode


        // Load the start value into the GPTM Timer n Interval Load Register (GPTMTnILR).
           pTimerHandler->pTimerx->TAILR = pTimerHandler->pTimerx_Config->Interval_Load ;



        // Set the TnEN bit in the GPTMCTL register to enable the timer and start counting.
        pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TAEN) ; //enable Timer A
        pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TBEN) ; //enable Timer B


    }
    /*----------------------------------------------------------------------------------*/
    else if ((pTimerHandler->pTimerx_Config->SplitOrConct) == TIMER_CFG_SPLIT)
    {
        // Write the GPTM Configuration Register (GPTMCFG) with a value of 0x0000.0000
        pTimerHandler->pTimerx->CFG = ( 0x4 << TIMER_CFG_GPTMCFG ) ;     // 16 bit timer
        /*----------------------------------------------------------------------------------*/

        // set count direction
        if ( (pTimerHandler->pTimerx_Config->Used_Timer) == TIMER_USED_TIMER_A )
        {
            // set count direction
            if ( (pTimerHandler->pTimerx_Config->CountDir) == TIMER_COUNTDIR_UP )
            {
                pTimerHandler->pTimerx->TAMR |= (1<<TIMER_TAMR_TACDIR) ;  // count up
            }
            else
            {
                pTimerHandler->pTimerx->TAMR &= ~(1<<TIMER_TAMR_TACDIR) ;  // count down
            }

            // mode : one shot or periodic
            if ( (pTimerHandler->pTimerx_Config->Mode) == TIMER_MODE_ONESHOT )
               {
                   pTimerHandler->pTimerx->TAMR |= (1<<TIMER_TAMR_TAMR ) ;
               }
               else if ( (pTimerHandler->pTimerx_Config->Mode) == TIMER_MODE_PERIODIC )
               {
                   pTimerHandler->pTimerx->TAMR &= ~(1<<TIMER_TAMR_TAMR ) ;
               }
               else
               {
                   // other configs
               }

            // prescaler
            pTimerHandler->pTimerx->TAPR = ((pTimerHandler->pTimerx_Config->Prescaler) << 0) ;

            // Load the start value into the GPTM Timer n Interval Load Register (GPTMTnILR).
              pTimerHandler->pTimerx->TAILR =(u_int_16 )( pTimerHandler->pTimerx_Config->Interval_Load ) ;


            pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TAEN) ; //enable Timer A
        }
        /*----------------------------------------------------------------------------------*/

        else if ( (pTimerHandler->pTimerx_Config->Used_Timer) == TIMER_USED_TIMER_B )
        {
            // set count direction
            if ( (pTimerHandler->pTimerx_Config->CountDir) == TIMER_COUNTDIR_UP )
            {
                pTimerHandler->pTimerx->TBMR |= (1<<TIMER_TBMR_TBCDIR) ;  // count up
            }
            else
            {
                pTimerHandler->pTimerx->TBMR &= ~(1<<TIMER_TBMR_TBCDIR) ;  // count down
            }

            // mode : one shot or periodic
            if ( (pTimerHandler->pTimerx_Config->Mode) == TIMER_MODE_ONESHOT )
                {
                    pTimerHandler->pTimerx->TBMR |= (1<<TIMER_TAMR_TAMR ) ;
                }
                else if ( (pTimerHandler->pTimerx_Config->Mode) == TIMER_MODE_PERIODIC )
                {
                    pTimerHandler->pTimerx->TBMR &= ~(1<<TIMER_TBMR_TBMR ) ;
                }
                else
                {
                    // other configs
                }

            // prescaler
            pTimerHandler->pTimerx->TBPR = ((pTimerHandler->pTimerx_Config->Prescaler) << 0) ;

            // Load the start value into the GPTM Timer n Interval Load Register (GPTMTnILR).
            pTimerHandler->pTimerx->TBILR = (u_int_16 )( pTimerHandler->pTimerx_Config->Interval_Load );


            //enable Timer B
            pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TBEN) ;
        }
        /*----------------------------------------------------------------------------------*/

        else
        {
            // invalid used timer
        }

    }
    /*----------------------------------------------------------------------------------*/

    else
    {
        // invalid configuration
    }

    /*----------------------------------------------------------------------------------*/



    // INT_6. If interrupts are required, set the appropriate bits in the GPTM Interrupt Mask Register (GPTMIMR).




    /* INT_8. Poll the GPTMRIS register or wait for the interrupt to be generated (if enabled). In both cases,
              the status flags are cleared by writing a 1 to the appropriate bit of the GPTM Interrupt Clear
              Register (GPTMICR).*/


}

/*-------------------------------------------------------------------------------------------------------*/

void Timer_Init_PWM (Timer_Handler_t *pTimerHandler )
{

    // 0. Enable Clock for timer peripheral
    Timer_ClkControl(pTimerHandler->pTimerx , ENABLE) ;

    // 1. Ensure the timer is disabled (the TnEN bit in the GPTMCTL register is cleared) before making any changes.

    pTimerHandler->pTimerx->CTL &= ~(1<<TIMER_CTL_TAEN) ; //disable Timer A
    pTimerHandler->pTimerx->CTL &= ~(1<<TIMER_CTL_TBEN) ; //disable Timer B


    if ( (pTimerHandler->pTimerx_Config->SplitOrConct) == TIMER_CFG_CONCT )
    {
        // Write the GPTM Configuration Register (GPTMCFG) with a value of 0x0000.0000
        pTimerHandler->pTimerx->CFG = ( 0 << TIMER_CFG_GPTMCFG ) ;  // 32 bit timer

        // count down  ONLY COUNT DOWN IS AVAIALABLE FOR PWM MODE
        pTimerHandler->pTimerx->TAMR &= ~(1<<TIMER_TAMR_TACDIR) ;

       // PWM enable
        pTimerHandler->pTimerx->TAMR |= (1 << TIMER_TAMR_TAAMS) ;

        // edge count mode
        pTimerHandler->pTimerx->TAMR |= (0 << TIMER_TAMR_TACMR) ;

        // periodic mode
        pTimerHandler->pTimerx->TAMR |= (0x2 << TIMER_TAMR_TAMR) ;


        // PWM  status
        if ( (pTimerHandler->pTimerx_Config->PWM_Ststus) == TIMER_PWM_STATUS_INVERTED )
        {
            pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TAPWML) ;  // uneffected mode
        }
        else
        {
            pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TAPWML) ;  // inverted mode
        }

        // prescaler can be used only in 16-bit mode

        // Load the start value into the GPTM Timer n Interval Load Register (GPTMTnILR).
        pTimerHandler->pTimerx->TAILR = pTimerHandler->pTimerx_Config->Interval_Load  ;

        //duty cycle
        pTimerHandler->pTimerx->TAMATCHR =  (pTimerHandler->pTimerx_Config->PWM_DytyCycle);

        // Set the TnEN bit in the GPTMCTL register to enable the timer and start counting.
        pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TAEN) ; //enable Timer A
        pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TBEN) ; //enable Timer B


    }
    /*----------------------------------------------------------------------------------*/
    else if ((pTimerHandler->pTimerx_Config->SplitOrConct) == TIMER_CFG_SPLIT)
    {
        // Write the GPTM Configuration Register (GPTMCFG) with a value of 0x0000.0004
        pTimerHandler->pTimerx->CFG = ( 0x4 << TIMER_CFG_GPTMCFG ) ;     // 16 bit timer
        /*----------------------------------------------------------------------------------*/

        // set count direction
        if ( (pTimerHandler->pTimerx_Config->Used_Timer) == TIMER_USED_TIMER_A )
        {
            // count down  ONLY COUNT DOWN IS AVAIALABLE FOR PWM MODE
             pTimerHandler->pTimerx->TAMR &= ~(1<<TIMER_TAMR_TACDIR) ;


            // PWM enable
             pTimerHandler->pTimerx->TAMR |= (1 << TIMER_TAMR_TAAMS) ;

             // edge count mode
             pTimerHandler->pTimerx->TAMR |= (0 << TIMER_TAMR_TACMR) ;

             // periodic mode
             pTimerHandler->pTimerx->TAMR |= (0x2 << TIMER_TAMR_TAMR) ;


            // prescaler
            pTimerHandler->pTimerx->TAPR = ((pTimerHandler->pTimerx_Config->Prescaler) << 0) ;

            // PWM  status
            if ( (pTimerHandler->pTimerx_Config->PWM_Ststus) == TIMER_PWM_STATUS_INVERTED )
            {
                pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TAPWML) ;  // uneffected mode
            }
            else
            {
                pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TAPWML) ;  // inverted mode
            }

            // Load the start value into the GPTM Timer n Interval Load Register (GPTMTnILR).
              pTimerHandler->pTimerx->TAILR =(u_int_16 )( pTimerHandler->pTimerx_Config->Interval_Load ) ;

              //duty cycle
              pTimerHandler->pTimerx->TAMATCHR = (u_int_16) (pTimerHandler->pTimerx_Config->PWM_DytyCycle);


            pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TAEN) ; //enable Timer A
        }
        /*----------------------------------------------------------------------------------*/

        else if ( (pTimerHandler->pTimerx_Config->Used_Timer) == TIMER_USED_TIMER_B )
        {
            // count down  ONLY COUNT DOWN IS AVAIALABLE FOR PWM MODE
            pTimerHandler->pTimerx->TBMR &= ~(1<<TIMER_TBMR_TBCDIR) ;


            // PWM enable
             pTimerHandler->pTimerx->TBMR |= (1 << TIMER_TBMR_TBAMS) ;

             // edge count mode
             pTimerHandler->pTimerx->TBMR |= (0 << TIMER_TBMR_TBCMR) ;

             // periodic mode
             pTimerHandler->pTimerx->TBMR |= (0x2 << TIMER_TBMR_TBMR) ;

            // prescaler
            pTimerHandler->pTimerx->TBPR = ((pTimerHandler->pTimerx_Config->Prescaler) << 0) ;

            // PWM  status
            if ( (pTimerHandler->pTimerx_Config->PWM_Ststus) == TIMER_PWM_STATUS_INVERTED )
            {
                pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TAPWML) ;  // uneffected mode
            }
            else
            {
                pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TBPWML) ;  // inverted mode
            }

            // Load the start value into the GPTM Timer n Interval Load Register (GPTMTnILR).
            pTimerHandler->pTimerx->TBILR = (u_int_16 )( pTimerHandler->pTimerx_Config->Interval_Load );

            //duty cycle
            pTimerHandler->pTimerx->TBMATCHR = (u_int_16) (pTimerHandler->pTimerx_Config->PWM_DytyCycle);


            //enable Timer B
            pTimerHandler->pTimerx->CTL |= (1<<TIMER_CTL_TBEN) ;
        }
        /*----------------------------------------------------------------------------------*/

        else
        {
            // invalid used timer
        }

    }
    /*----------------------------------------------------------------------------------*/

    else
    {
        // invalid configuration
    }

    /*----------------------------------------------------------------------------------*/



    // INT_6. If interrupts are required, set the appropriate bits in the GPTM Interrupt Mask Register (GPTMIMR).




    /* INT_8. Poll the GPTMRIS register or wait for the interrupt to be generated (if enabled). In both cases,
              the status flags are cleared by writing a 1 to the appropriate bit of the GPTM Interrupt Clear
              Register (GPTMICR).*/



}








