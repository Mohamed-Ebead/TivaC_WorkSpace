/*
 * ADC_Driver_interface.h
 *
 *  Created on: May 7, 2020
 *      Author: Mohamed  Ebead
 */

#ifndef DRIVERS_INC_ADC_DRIVER_INTERFACE_H_
#define DRIVERS_INC_ADC_DRIVER_INTERFACE_H_



typedef struct
{
    u_int_8 ADC_SampleSequencer ;      // @_ADC_Sample_sequencer
    u_int_8 ADC_Trigger         ;      // @_ADC_Trigger_select
    u_int_8 ADC_Trig_PWM_Module ;      // @_ADC_Trigger_PWM_Module  // use it only when triggering ADC with PWM generator
    u_int_8 ADC_Channel         ;      // @_ADC_Channels  from 0 to 11
    u_int_8 ADC_DiffOrSingleEnd ;      // @_ADC_Differential_or_singleended
    u_int_8 ADC_LastSample      ;      // @_ADC_Last_Sample



}ADC_Config_t;



typedef struct
{
    ADC_RegDef_t *pADC ;
    ADC_Config_t *pADC_Config ;


}ADC_Handler_t;

/*--------------------------------------------------------------------------*/
// ADC MACROS

// WARNING !!! DON'T EVER CHANGE MACROS VALUES

//// @_ADC_Sample_sequencer
#define ADC_SAMPSEQ_0     0
#define ADC_SAMPSEQ_1     1
#define ADC_SAMPSEQ_2     2
#define ADC_SAMPSEQ_3     3


// @_ADC_Trigger_select
#define ADC_TRIG_PROCESSOR        0x0
#define ADC_TRIG_ANALOG_COMP0     0x1
#define ADC_TRIG_ANALOG_COMP1     0x2
#define ADC_TRIG_EXT_GPIO         0x4
#define ADC_TRIG_TIMER            0x5
#define ADC_TRIG_PWM0             0x6
#define ADC_TRIG_PWM1             0x7
#define ADC_TRIG_PWM2             0x8
#define ADC_TRIG_PWM3             0x9
#define ADC_TRIG_ALWAYS           0xF


// @_ADC_Trigger_PWM_Module
#define ADC_TRIG_PWM_MOD_0     0x0
#define ADC_TRIG_PWM_MOD_1     0x1

// @_ADC_Channels  from 0 to 11
#define ADC_AIN0   0
#define ADC_AIN1   1
#define ADC_AIN2   2
#define ADC_AIN3   3
#define ADC_AIN4   4
#define ADC_AIN5   5
#define ADC_AIN6   6
#define ADC_AIN7   7
#define ADC_AIN8   8
#define ADC_AIN9   9
#define ADC_AIN10  10
#define ADC_AIN11  11


// @_ADC_Differential_or_singleended
#define ADC_SINGLE_ENDED  0
#define ADC_DIFFERENTIAL  1


// @_ADC_Last_Sample
#define ADC_NOT_LAST_SAMPLE   0
#define ADC_LAST_SAMPLE       1





/*-------------------------------------------------------------------------*/
/*--------------------        ADC Related APIs       ----------------------*/
/*-------------------------------------------------------------------------*/

void ADC_ClkCtrl ( ADC_RegDef_t *pADCx ) ;

void ADC_Init ( ADC_Handler_t *pADCx_handler ) ;

u_int_16 ADC_Read_Blocking ( ADC_Handler_t *pADCx_handler ) ;
void ADC_Read_NonBlocking ( ADC_Handler_t *pADCx_handler ) ;









/*------------   ADC channels ------------*/

#define ADC_CH_0       GPIOE_PIN3
#define ADC_CH_1       GPIOE_PIN2
#define ADC_CH_2       GPIOE_PIN1
#define ADC_CH_3       GPIOE_PIN0
#define ADC_CH_4       GPIOD_PIN3
#define ADC_CH_5       GPIOD_PIN2
#define ADC_CH_6       GPIOD_PIN1
#define ADC_CH_7       GPIOD_PIN0
#define ADC_CH_8       GPIOE_PIN5
#define ADC_CH_9       GPIOE_PIN4
#define ADC_CH_10      GPIOB_PIN4
#define ADC_CH_11      GPIOB_PIN5

#define GPIOE_PIN3    GPIO_PIN_NO_3
#define GPIOE_PIN2    GPIO_PIN_NO_2
#define GPIOE_PIN1    GPIO_PIN_NO_1
#define GPIOE_PIN0    GPIO_PIN_NO_0
#define GPIOD_PIN3    GPIO_PIN_NO_3
#define GPIOD_PIN2    GPIO_PIN_NO_2
#define GPIOD_PIN1    GPIO_PIN_NO_1
#define GPIOD_PIN0    GPIO_PIN_NO_0
#define GPIOE_PIN5    GPIO_PIN_NO_5
#define GPIOE_PIN4    GPIO_PIN_NO_4
#define GPIOB_PIN4    GPIO_PIN_NO_4
#define GPIOB_PIN5    GPIO_PIN_NO_5

/*-----------------------------------------------------*/













#endif /* DRIVERS_INC_ADC_DRIVER_INTERFACE_H_ */
