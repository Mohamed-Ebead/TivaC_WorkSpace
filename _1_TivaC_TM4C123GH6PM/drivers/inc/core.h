/*
 * core.h
 *
 *  Created on: May 3, 2020
 *      Author: Mohamed  Ebead
 */

#ifndef CORE_H_
#define CORE_H_


#include "STD_TYPES.h"


#define _RW_  volatile               //  Read / Write
#define _WO_  volatile               //  Write Only
#define _RO_  volatile const         //  Read Only




/**
  \brief  Structure type to access the System Timer (SysTick) Registers .
 */
typedef struct
{
    //System Timer (SysTick) Registers
    _RW_ u_int_32 STCTRL  ;               /*!< SysTick Control and Status Register  */
    _RW_ u_int_32 STRELOAD  ;             /*!< SysTick Reload Value Register        */
    _RW_ u_int_32 STCURRENT ;             /*!< SysTick Current Value Register       */

} SysTick_t ;





/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
    //Nested Vectored Interrupt Controller (NVIC) Registers

    _RW_ u_int_32 ISER[5U];               /*!<  Interrupt Set Enable Register       */
         u_int_32 RESERVED0[27U];
    _RW_ u_int_32 ICER[5U];               /*!<  Interrupt Clear Enable Register     */
         u_int_32 RSERVED1[27U];
    _RW_ u_int_32 ISPR[5U];               /*!<  Interrupt Set Pending Register      */
         u_int_32 RESERVED2[27U];
    _RW_ u_int_32 ICPR[5U];               /*!<  Interrupt Clear Pending Register    */
         u_int_32 RESERVED3[27U];
    _RW_ u_int_32 IABR[5U];               /*!<  Interrupt Active bit Register       */
         u_int_32 RESERVED4[59U];
    _RW_ u_int_32 IPR[35U];               /*!<   nterrupt Priority Register          */
         u_int_32 RESERVED5[669U];
    _WO_ u_int_32 STIR;                   /*!<  Software Trigger Interrupt Register */

}  NVIC_t ;


#define SCS_BASE          ( (volatile u_int_32*)0xE000E000 )                     /*!< System Control Space Base Address */

#define SysTick_BASE        (SCS_BASE +  0x0010UL)             /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)             /*!< NVIC Base Address */


#define SysTick             ((SysTick_t *)   SysTick_BASE  )   /*!< SysTick configuration struct */
#define NVIC                ((NVIC_t    *)   NVIC_BASE     )   /*!< NVIC configuration struct    */














#endif /* CORE_H_ */
