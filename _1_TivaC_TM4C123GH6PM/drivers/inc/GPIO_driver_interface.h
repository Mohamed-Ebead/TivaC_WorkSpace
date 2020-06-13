/*
 * GPIO_driver_interface.h
 *
 *  Created on: Apr 27, 2020
 *      Author: Mohamed  Ebead
 */

#ifndef GPIO_DRIVER_INTERFACE_H_
#define GPIO_DRIVER_INTERFACE_H_

#include "STD_TYPES.h"
#include "TivaC_peripherals.h"






// GPIO configuration structure

typedef struct
{
    u_int_8 GPIO_PinNumber ;         // @_GPIO_possible_PIN_numbers
	u_int_8 GPIO_PinDir ;            // @_GPIO_possible_PIN_Directions  (default : INPUT )
    u_int_8 GPIO_PinMode ;           // @_GPIO_possible_Modes GPIO / Altern / Analog / uDMA  (default : GPIO )
    u_int_8 GPIO_PinDrivStren ;      // @_GPIO_possible_Drive_Strengthes  (2mA , 4mA , 8mA  ; default :2mA)
    u_int_8 GPIO_PinOPType ;         // @_GPIO_possible_output_types pull up / pull down / open drain (defult : all are disabled  )
    u_int_8 GPIO_PinSignalType ;     // @_GPIO_possible_SignalType    digital / analog (default : both are disabled )
    u_int_8 GPIO_PinIntMode ;        // @_GPIO_possible_Interrupt_Modes  falling / rising / both edges triggered (default : GPIO )
    u_int_8 GPIO_PinAltFunMode ;

} GPIO_Config_t ;





// GPIO handler structure

typedef struct
{
    GPIO_RegDef_t *pGPIOx ;          /* !< holds the base address of the GPIO where the pin belongs */
    GPIO_Config_t *pGPIO_PinConfig ; /* !<  */



}GPIO_Handler_t;


// @_GPIO_possible_PIN_numbers
#define GPIO_PIN_NO_0    0
#define GPIO_PIN_NO_1    1
#define GPIO_PIN_NO_2    2
#define GPIO_PIN_NO_3    3
#define GPIO_PIN_NO_4    4
#define GPIO_PIN_NO_5    5
#define GPIO_PIN_NO_6    6
#define GPIO_PIN_NO_7    7


// @_GPIO_possible_PIN_Directions 
#define GPIO_DIR_IN        0       //  input mode
#define GPIO_DIR_OUT       1       //  output mode


// @_GPIO_possible_Modes
#define GPIO_MODE_GPIO      0       //  GPIO mode
#define GPIO_MODE_ALTFN     1       //  alternating function mode
#define GPIO_MODE_ANALOG    2       //  analog mode
#define GPIO_MODE_UDMA      3       //  u DMA

// @_GPIO_possible_Drive_Strengthes 
#define GPIO_DRIVSTR_MIN      0       //  2-mA 
#define GPIO_DRIVSTR_MID      1       //  4-mA
#define GPIO_DRIVSTR_MAX      2       //  8-mA

// @_GPIO_possible_PUPDOD_types
#define GPIO_OP_TYPE_NOPUPDOD    0        // pull UP output
#define GPIO_OP_TYPE_PU          1        // pull UP output
#define GPIO_OP_TYPE_PD          2        // pull DOWN output
#define GPIO_OP_TYPE_OD          3        // open drain output

// @_GPIO_possible_SignalType    digital / analog
#define GPIO_SIGNAL_DIGITAL    0        // digital signal 
#define GPIO_SIGNAL_ANALOG     1        // analog signal 


// @_GPIO_possible_Interrupt_Modes
#define GPIO_INT_DIS    0       //  Disable interrupt mode
#define GPIO_INT_FT     1       //  interrupt mode , falling edge triggered
#define GPIO_INT_RT     2       //  interrupt mode , rising edge triggered
#define GPIO_INT_RFT    3       //  interrupt mode , falling & rising edge triggered
#define GPIO_INT_HL     4       //  interrupt mode , high level triggered
#define GPIO_INT_LL     5       //  interrupt mode , low level triggered











/*--------------------------- Interrupt MACROs ---------------------------*/


/*         Change Processor State.
 * CPSID i ; Disable interrupts and configurable fault handlers (set PRIMASK)
 * CPSID f ; Disable interrupts and all fault handlers (set FAULTMASK)
 * CPSIE i ; Enable interrupts and configurable fault handlers (clear PRIMASK)
 * CPSIE f ; Enable interrupts and fault handlers (clear FAULTMASK)
*/

// Global Interrupt enable and disable
#define GLOBAL_INT_EN()       __asm__ (" CPSIE i ")
#define GLOBAL_INT_DIS()      __asm__ (" CPSID i ")

// IRQ_Possible_Priorities 0 >> 7
#define NVIC_IRQ_PRI_0     0
#define NVIC_IRQ_PRI_1     1
#define NVIC_IRQ_PRI_2     2
#define NVIC_IRQ_PRI_3     3
#define NVIC_IRQ_PRI_4     4
#define NVIC_IRQ_PRI_5     5
#define NVIC_IRQ_PRI_6     6
#define NVIC_IRQ_PRI_7     7




/***************************************************************************
 *            --------  APIs sUPPORTS BY THIS DRIVER --------
 *
 * *************************************************************************/

// Peripheral Clock Setup
void GPIO_ClockCtrl (GPIO_RegDef_t *pGPIOx , u_int_8 EnOrDis) ;

// Initialize and DeInitialize
void GPIO_Init   (GPIO_Handler_t *pGPIOHandle )   ;
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx )   ;

// Data Read and Write
u_int_8  GPIO_ReadFromInputPin  (GPIO_RegDef_t *pGPIOx , u_int_8 u8_PinNumber)                       ;
u_int_8  GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx)                                              ;
void     GPIO_SetOutputPin  (GPIO_RegDef_t *pGPIOx , u_int_8 u8_PinNumber )                          ;
void     GPIO_ResetOutputPin  (GPIO_RegDef_t *pGPIOx , u_int_8 u8_PinNumber )                        ;
void     GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx , u_int_8 u8_Value)                           ;
void     GPIO_ToggleOutputPin   (GPIO_RegDef_t *pGPIOx , u_int_8 u8_PinNumber )                      ;

// IRQ configuration and ISR handling
void GPIO_IRQInterruptConfig (u_int_8 IRQNumber , u_int_8 EnOrDis) ;
void GPIO_IRQPriorityConfig (u_int_8 IRQNumber , u_int_8 IRQPriority) ;
void GPIOx_IRQHandling (GPIO_RegDef_t *pGPIOx , u_int_8 u8_PinNumber)   ;

// IRQ handlers from startup code
void GPIOA_Handler ( void )   ;
void GPIOB_Handler ( void )   ;
void GPIOC_Handler ( void )   ;
void GPIOD_Handler ( void )   ;
void GPIOE_Handler ( void )   ;
void GPIOF_Handler ( void )   ;
void GPIOG_Handler ( void )   ;
void GPIOH_Handler ( void )   ;












#endif /* GPIO_DRIVER_INTERFACE_H_ */
