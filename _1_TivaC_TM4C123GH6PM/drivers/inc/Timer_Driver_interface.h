/*
 * Timer_Driver_interface.h
 *
 *  Created on: May 6, 2020
 *      Author: Mohamed  Ebead
 */

#ifndef DRIVERS_INC_TIMER_DRIVER_INTERFACE_H_
#define DRIVERS_INC_TIMER_DRIVER_INTERFACE_H_

typedef struct
{
    u_int_32  PWM_DytyCycle ;      // PWM mode only
    u_int_32 Interval_Load ;       // value to cont from zero up to it or from it down to zero
    u_int_16 Prescaler  ;         // used with split mode only (individual)
    u_int_8  Mode ;               // @_Timer_Mode_OneShot_or_periodic
    u_int_8  SplitOrConct ;       // @_Timer_Split_Or_Concate
    u_int_8  Used_Timer  ;        //  @_Timer_Used_A_Or_B      only used when split timer
    u_int_8  CountDir ;           // @_Timer_CountDirection_Up_or_Down
    u_int_8  PWM_Ststus ;         // @_Timer_Status_Unaffected_Or_Inverted  in PWM mode only




}Timer_Config_t;


typedef struct
{
    Timer_RegDef_t *pTimerx ;
    Timer_Config_t *pTimerx_Config ;

}Timer_Handler_t;


// configuration macros

// @_Timer_Split_Or_Concate
#define TIMER_CFG_CONCT  0
#define TIMER_CFG_SPLIT  1


// @_Timer_Mode_OneShot_or_periodic
#define TIMER_MODE_ONESHOT   0
#define TIMER_MODE_PERIODIC  1

// @_Timer_CountDirection_Up_or_Down
#define TIMER_COUNTDIR_DOWN  0
#define TIMER_COUNTDIR_UP    1

//  @_Timer_Used_A_Or_B
#define TIMER_USED_TIMER_A   0
#define TIMER_USED_TIMER_B   1

// @_Timer_Status_Unaffected_Or_Inverted
#define TIMER_PWM_STATUS_UNAFFECTED   0
#define TIMER_PWM_STATUS_INVERTED     1


void Timer_ClkControl (Timer_RegDef_t *Timerx , u_int_8 ENOrDIS) ;
void W_Timer_ClkControl (W_Timer_RegDef_t *Timerx , u_int_8 ENOrDIS) ;

void Timer_Init_OneShot_Periodic (Timer_Handler_t *pTimerHandler ) ;
void Timer_Init_PWM (Timer_Handler_t *pTimerHandler ) ;
void W_Timer_Init_OneShot_Periodic (Timer_Handler_t *pTimerHandler ) ;




















#endif /* DRIVERS_INC_TIMER_DRIVER_INTERFACE_H_ */
