/*
 * TivaC_peripherals.h
 *
 *  Created on: Apr 26, 2020
 *      Author: Mohamed  Ebead
 */

#ifndef TIVAC_PERIPHERALS_H_
#define TIVAC_PERIPHERALS_H_


// _t stands for structure type


#include "STD_TYPES.h"
#include "core.h"

#define _RW_  volatile               //  Read / Write
#define _WO_  volatile               //  Write Only
#define _RO_  volatile const         //  Read Only

#define NB_PR_BITS_RESERVED   5     // reserved bits in The PRIn registers

/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum
{
/* -------------------  Cortex-M4 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,              /*!<   4  Memory Management, MPU mismatch, including Access Violation
                                                         and No Match                                                          */
  BusFault_IRQn                 = -11,              /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                         related Fault                                                         */
  UsageFault_IRQn               = -10,              /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* -------------------  TM4C123GH6PM Specific Interrupt Numbers  ------------------ */
  GPIOA_IRQn                    =   0,              /*!<   0  GPIOA                                                            */
  GPIOB_IRQn                    =   1,              /*!<   1  GPIOB                                                            */
  GPIOC_IRQn                    =   2,              /*!<   2  GPIOC                                                            */
  GPIOD_IRQn                    =   3,              /*!<   3  GPIOD                                                            */
  GPIOE_IRQn                    =   4,              /*!<   4  GPIOE                                                            */
  UART0_IRQn                    =   5,              /*!<   5  UART0                                                            */
  UART1_IRQn                    =   6,              /*!<   6  UART1                                                            */
  SSI0_IRQn                     =   7,              /*!<   7  SSI0                                                             */
  I2C0_IRQn                     =   8,              /*!<   8  I2C0                                                             */
  PWM0_FAULT_IRQn               =   9,              /*!<   9  PWM0_FAULT                                                       */
  PWM0_0_IRQn                   =  10,              /*!<  10  PWM0_0                                                           */
  PWM0_1_IRQn                   =  11,              /*!<  11  PWM0_1                                                           */
  PWM0_2_IRQn                   =  12,              /*!<  12  PWM0_2                                                           */
  QEI0_IRQn                     =  13,              /*!<  13  QEI0                                                             */
  ADC0SS0_IRQn                  =  14,              /*!<  14  ADC0SS0                                                          */
  ADC0SS1_IRQn                  =  15,              /*!<  15  ADC0SS1                                                          */
  ADC0SS2_IRQn                  =  16,              /*!<  16  ADC0SS2                                                          */
  ADC0SS3_IRQn                  =  17,              /*!<  17  ADC0SS3                                                          */
  WATCHDOG0_IRQn                =  18,              /*!<  18  WATCHDOG0                                                        */
  TIMER0A_IRQn                  =  19,              /*!<  19  TIMER0A                                                          */
  TIMER0B_IRQn                  =  20,              /*!<  20  TIMER0B                                                          */
  TIMER1A_IRQn                  =  21,              /*!<  21  TIMER1A                                                          */
  TIMER1B_IRQn                  =  22,              /*!<  22  TIMER1B                                                          */
  TIMER2A_IRQn                  =  23,              /*!<  23  TIMER2A                                                          */
  TIMER2B_IRQn                  =  24,              /*!<  24  TIMER2B                                                          */
  COMP0_IRQn                    =  25,              /*!<  25  COMP0                                                            */
  COMP1_IRQn                    =  26,              /*!<  26  COMP1                                                            */
  SYSCTL_IRQn                   =  28,              /*!<  28  SYSCTL                                                           */
  FLASH_CTRL_IRQn               =  29,              /*!<  29  FLASH_CTRL                                                       */
  GPIOF_IRQn                    =  30,              /*!<  30  GPIOF                                                            */
  UART2_IRQn                    =  33,              /*!<  33  UART2                                                            */
  SSI1_IRQn                     =  34,              /*!<  34  SSI1                                                             */
  TIMER3A_IRQn                  =  35,              /*!<  35  TIMER3A                                                          */
  TIMER3B_IRQn                  =  36,              /*!<  36  TIMER3B                                                          */
  I2C1_IRQn                     =  37,              /*!<  37  I2C1                                                             */
  QEI1_IRQn                     =  38,              /*!<  38  QEI1                                                             */
  CAN0_IRQn                     =  39,              /*!<  39  CAN0                                                             */
  CAN1_IRQn                     =  40,              /*!<  40  CAN1                                                             */
  HIB_IRQn                      =  43,              /*!<  43  HIB                                                              */
  USB0_IRQn                     =  44,              /*!<  44  USB0                                                             */
  PWM0_3_IRQn                   =  45,              /*!<  45  PWM0_3                                                           */
  UDMA_IRQn                     =  46,              /*!<  46  UDMA                                                             */
  UDMAERR_IRQn                  =  47,              /*!<  47  UDMAERR                                                          */
  ADC1SS0_IRQn                  =  48,              /*!<  48  ADC1SS0                                                          */
  ADC1SS1_IRQn                  =  49,              /*!<  49  ADC1SS1                                                          */
  ADC1SS2_IRQn                  =  50,              /*!<  50  ADC1SS2                                                          */
  ADC1SS3_IRQn                  =  51,              /*!<  51  ADC1SS3                                                          */
  SSI2_IRQn                     =  57,              /*!<  57  SSI2                                                             */
  SSI3_IRQn                     =  58,              /*!<  58  SSI3                                                             */
  UART3_IRQn                    =  59,              /*!<  59  UART3                                                            */
  UART4_IRQn                    =  60,              /*!<  60  UART4                                                            */
  UART5_IRQn                    =  61,              /*!<  61  UART5                                                            */
  UART6_IRQn                    =  62,              /*!<  62  UART6                                                            */
  UART7_IRQn                    =  63,              /*!<  63  UART7                                                            */
  I2C2_IRQn                     =  68,              /*!<  68  I2C2                                                             */
  I2C3_IRQn                     =  69,              /*!<  69  I2C3                                                             */
  TIMER4A_IRQn                  =  70,              /*!<  70  TIMER4A                                                          */
  TIMER4B_IRQn                  =  71,              /*!<  71  TIMER4B                                                          */
  TIMER5A_IRQn                  =  92,              /*!<  92  TIMER5A                                                          */
  TIMER5B_IRQn                  =  93,              /*!<  93  TIMER5B                                                          */
  WTIMER0A_IRQn                 =  94,              /*!<  94  WTIMER0A                                                         */
  WTIMER0B_IRQn                 =  95,              /*!<  95  WTIMER0B                                                         */
  WTIMER1A_IRQn                 =  96,              /*!<  96  WTIMER1A                                                         */
  WTIMER1B_IRQn                 =  97,              /*!<  97  WTIMER1B                                                         */
  WTIMER2A_IRQn                 =  98,              /*!<  98  WTIMER2A                                                         */
  WTIMER2B_IRQn                 =  99,              /*!<  99  WTIMER2B                                                         */
  WTIMER3A_IRQn                 = 100,              /*!< 100  WTIMER3A                                                         */
  WTIMER3B_IRQn                 = 101,              /*!< 101  WTIMER3B                                                         */
  WTIMER4A_IRQn                 = 102,              /*!< 102  WTIMER4A                                                         */
  WTIMER4B_IRQn                 = 103,              /*!< 103  WTIMER4B                                                         */
  WTIMER5A_IRQn                 = 104,              /*!< 104  WTIMER5A                                                         */
  WTIMER5B_IRQn                 = 105,              /*!< 105  WTIMER5B                                                         */
  SYSEXC_IRQn                   = 106,              /*!< 106  SYSEXC                                                           */
  PWM1_0_IRQn                   = 134,              /*!< 134  PWM1_0                                                           */
  PWM1_1_IRQn                   = 135,              /*!< 135  PWM1_1                                                           */
  PWM1_2_IRQn                   = 136,              /*!< 136  PWM1_2                                                           */
  PWM1_3_IRQn                   = 137,              /*!< 137  PWM1_3                                                           */
  PWM1_FAULT_IRQn               = 138               /*!< 138  PWM1_FAULT                                                       */
} IRQn_t;





/* ================================================================================ */
/* ================                    WATCHDOG0                   ================ */
/* ================================================================================ */


/**
  * @brief Register map for WATCHDOG0 peripheral (WATCHDOG0)
  */

typedef struct
{                                                   /*!< WATCHDOG0 Structure                                                   */
  _RW_ u_int_32  LOAD;                              /*!< Watchdog Load                                                         */
  _RW_ u_int_32  VALUE;                             /*!< Watchdog Value                                                        */
  _RW_ u_int_32  CTL;                               /*!< Watchdog Control                                                      */
  _WO_ u_int_32  ICR;                               /*!< Watchdog Interrupt Clear                                              */
  _RW_ u_int_32  RIS;                               /*!< Watchdog Raw Interrupt Status                                         */
  _RW_ u_int_32  MIS;                               /*!< Watchdog Masked Interrupt Status                                      */
  _RO_ u_int_32  RESERVED0[256];
  _RW_ u_int_32  TEST;                              /*!< Watchdog Test                                                         */
  _RO_ u_int_32  RESERVED1[505];
  _RW_ u_int_32  LOCK;                              /*!< Watchdog Lock
                                                         */
} WATCHDOG0_t;


/* ================================================================================ */
/* ================                      GPIOx                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for GPIOA peripheral (GPIOx)
  */

typedef struct
{                                                   /*!< GPIOA Structure                                                       */
  _RO_ u_int_32  RESERVED0[255];
  _RW_ u_int_32  DATA;                              /*!< GPIO Data                                                             */
  _RW_ u_int_32  DIR;                               /*!< GPIO Direction                                                        */
  _RW_ u_int_32  IS;                                /*!< GPIO Interrupt Sense                                                  */
  _RW_ u_int_32  IBE;                               /*!< GPIO Interrupt Both Edges                                             */
  _RW_ u_int_32  IEV;                               /*!< GPIO Interrupt Event                                                  */
  _RW_ u_int_32  IM;                                /*!< GPIO Interrupt Mask                                                   */
  _RW_ u_int_32  RIS;                               /*!< GPIO Raw Interrupt Status                                             */
  _RW_ u_int_32  MIS;                               /*!< GPIO Masked Interrupt Status                                          */
  _WO_ u_int_32  ICR;                               /*!< GPIO Interrupt Clear                                                  */
  _RW_ u_int_32  AFSEL;                             /*!< GPIO Alternate Function Select                                        */
  _RO_ u_int_32  RESERVED1[55];
  _RW_ u_int_32  DR2R;                              /*!< GPIO 2-mA Drive Select                                                */
  _RW_ u_int_32  DR4R;                              /*!< GPIO 4-mA Drive Select                                                */
  _RW_ u_int_32  DR8R;                              /*!< GPIO 8-mA Drive Select                                                */
  _RW_ u_int_32  ODR;                               /*!< GPIO Open Drain Select                                                */
  _RW_ u_int_32  PUR;                               /*!< GPIO Pull-Up Select                                                   */
  _RW_ u_int_32  PDR;                               /*!< GPIO Pull-Down Select                                                 */
  _RW_ u_int_32  SLR;                               /*!< GPIO Slew Rate Control Select                                         */
  _RW_ u_int_32  DEN;                               /*!< GPIO Digital Enable                                                   */
  _RW_ u_int_32  LOCK;                              /*!< GPIO Lock                                                             */
  _RW_ u_int_32  CR;                                /*!< GPIO Commit  Read Only (only least 8 bits are R/W for GPIO only )     */
  _RW_ u_int_32  AMSEL;                             /*!< GPIO Analog Mode Select                                               */
  _RW_ u_int_32  PCTL;                              /*!< GPIO Port Control                                                     */
  _RW_ u_int_32  ADCCTL;                            /*!< GPIO ADC Control                                                      */
  _RW_ u_int_32  DMACTL;                            /*!< GPIO DMA Control
                                                    */
} GPIO_RegDef_t;


/* ================================================================================ */
/* ================                      SSI0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for SSIx peripheral (SSIx)
  */

typedef struct
{                                             /*!< SSI0 Structure                  */
  _RW_ u_int_32  CR0;                         /*!< SSI Control 0                   */
  _RW_ u_int_32  CR1;                         /*!< SSI Control 1                   */
  _RW_ u_int_32  DR;                          /*!< SSI Data                        */
  _RW_ u_int_32  SR;                          /*!< SSI Status                      */
  _RW_ u_int_32  CPSR;                        /*!< SSI Clock Prescale              */
  _RW_ u_int_32  IM;                          /*!< SSI Interrupt Mask              */
  _RW_ u_int_32  RIS;                         /*!< SSI Raw Interrupt Status        */
  _RW_ u_int_32  MIS;                         /*!< SSI Masked Interrupt Status     */
  _WO_ u_int_32  ICR;                         /*!< SSI Interrupt Clear             */
  _RW_ u_int_32  DMACTL;                      /*!< SSI DMA Control                 */
  _RO_ u_int_32  RESERVED0[1000];
  _RW_ u_int_32  CC;                                /*!< SSI Clock Configuration
                                              */
} SSI_RegDef_t;


/* ================================================================================ */
/* ================                      UART0                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for UART0 peripheral (UART0)
  */

typedef struct
{                                                   /*!< UART0 Structure                        */
  _RW_ u_int_32  DR;                                /*!< UART Data                              */

  union
  {
    _RW_ u_int_32  ECR_UART_ALT;          /*!< UART Receive Status/Error Clear        */
    _RW_ u_int_32  RSR;                   /*!< UART Receive Status/Error Clear        */
  };
  _RO_ u_int_32  RESERVED0[4];
  _RW_ u_int_32  FR;                      /*!< UART Flag                              */
  _RO_ u_int_32  RESERVED1;
  _RW_ u_int_32  ILPR;                    /*!< UART IrDA Low-Power Register           */
  _RW_ u_int_32  IBRD;                    /*!< UART Integer Baud-Rate Divisor         */
  _RW_ u_int_32  FBRD;                    /*!< UART Fractional Baud-Rate Divisor      */
  _RW_ u_int_32  LCRH;                    /*!< UART Line Control                      */
  _RW_ u_int_32  CTL;                     /*!< UART Control                           */
  _RW_ u_int_32  IFLS;                    /*!< UART Interrupt FIFO Level Select       */
  _RW_ u_int_32  IM;                      /*!< UART Interrupt Mask                    */
  _RW_ u_int_32  RIS;                     /*!< UART Raw Interrupt Status              */
  _RW_ u_int_32  MIS;                     /*!< UART Masked Interrupt Status           */
  _WO_ u_int_32  ICR;                     /*!< UART Interrupt Clear                   */
  _RW_ u_int_32  DMACTL;                  /*!< UART DMA Control                       */
  _RO_ u_int_32  RESERVED2[22];
  _RW_ u_int_32  _9BITADDR;               /*!< UART 9-Bit Self Address                */
  _RW_ u_int_32  _9BITAMASK;              /*!< UART 9-Bit Self Address Mask           */
  _RO_ u_int_32  RESERVED3[965];
  _RW_ u_int_32  PP;                      /*!< UART Peripheral Properties             */
  _RO_ u_int_32  RESERVED4;
  _RW_ u_int_32  CC;                                /*!< UART Clock Configuration
                                          */
} UART0_t;


/* ================================================================================ */
/* ================                      I2C0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for I2C0 peripheral (I2C0)
  */

typedef struct {                      /*!< I2C0 Structure                           */
  _RW_ u_int_32  MSA;                 /*!< I2C Master Slave Address                 */

  union {
    _RW_ u_int_32  MCS_I2C0_ALT;      /*!< I2C Master Control/Status                */
    _RW_ u_int_32  MCS;               /*!< I2C Master Control/Status                */
  };
  _RW_ u_int_32  MDR;                 /*!< I2C Master Data                          */
  _RW_ u_int_32  MTPR;                /*!< I2C Master Timer Period                  */
  _RW_ u_int_32  MIMR;                /*!< I2C Master Interrupt Mask                */
  _RW_ u_int_32  MRIS;                /*!< I2C Master Raw Interrupt Status          */
  _RW_ u_int_32  MMIS;                /*!< I2C Master Masked Interrupt Status       */
  _WO_ u_int_32  MICR;                /*!< I2C Master Interrupt Clear               */
  _RW_ u_int_32  MCR;                 /*!< I2C Master Configuration                 */
  _RW_ u_int_32  MCLKOCNT;            /*!< I2C Master Clock Low Timeout Count       */
  _RO_ u_int_32  RESERVED0;
  _RW_ u_int_32  MBMON;               /*!< I2C Master Bus Monitor                   */
  _RO_ u_int_32  RESERVED1[2];
  _RW_ u_int_32  MCR2;                /*!< I2C Master Configuration 2               */
  _RO_ u_int_32  RESERVED2[497];
  _RW_ u_int_32  SOAR;                /*!< I2C Slave Own Address                    */

  union {
    _RW_ u_int_32  SCSR_I2C0_ALT;     /*!< I2C Slave Control/Status                 */
    _RW_ u_int_32  SCSR;              /*!< I2C Slave Control/Status                 */
  };
  _RW_ u_int_32  SDR;                 /*!< I2C Slave Data                           */
  _RW_ u_int_32  SIMR;                /*!< I2C Slave Interrupt Mask                 */
  _RW_ u_int_32  SRIS;                /*!< I2C Slave Raw Interrupt Status           */
  _RW_ u_int_32  SMIS;                /*!< I2C Slave Masked Interrupt Status        */
  _WO_ u_int_32  SICR;                /*!< I2C Slave Interrupt Clear                */
  _RW_ u_int_32  SOAR2;               /*!< I2C Slave Own Address 2                  */
  _RW_ u_int_32  SACKCTL;             /*!< I2C Slave ACK Control                    */
  _RO_ u_int_32  RESERVED3[487];
  _RW_ u_int_32  PP;                  /*!< I2C Peripheral Properties                */
  _RW_ u_int_32  PC;                  /*!< I2C Peripheral Configuration             */
} I2C0_t;


/* ================================================================================ */
/* ================                      PWM0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for PWM0 peripheral (PWM0)
  */

typedef struct {                         /*!< PWM0 Structure                                 */
  _RW_ u_int_32  CTL;                    /*!< PWM Master Control                             */
  _RW_ u_int_32  SYNC;                   /*!< PWM Time Base Sync                             */
  _RW_ u_int_32  ENABLE;                 /*!< PWM Output Enable                              */
  _RW_ u_int_32  INVERT;                 /*!< PWM Output Inversion                           */
  _RW_ u_int_32  FAULT;                  /*!< PWM Output Fault                               */
  _RW_ u_int_32  INTEN;                  /*!< PWM Interrupt Enable                           */
  _RW_ u_int_32  RIS;                    /*!< PWM Raw Interrupt Status                       */
  _RW_ u_int_32  ISC;                    /*!< PWM Interrupt Status and Clear                 */
  _RW_ u_int_32  STATUS;                 /*!< PWM Status                                     */
  _RW_ u_int_32  FAULTVAL;               /*!< PWM Fault Condition Value                      */
  _RW_ u_int_32  ENUPD;                  /*!< PWM Enable Update                              */
  _RO_ u_int_32  RESERVED0[5];
  _RW_ u_int_32  _0_CTL;                 /*!< PWM0 Control                                   */
  _RW_ u_int_32  _0_INTEN;               /*!< PWM0 Interrupt and Trigger Enable              */
  _RW_ u_int_32  _0_RIS;                 /*!< PWM0 Raw Interrupt Status                      */
  _RW_ u_int_32  _0_ISC;                 /*!< PWM0 Interrupt Status and Clear                */
  _RW_ u_int_32  _0_LOAD;                /*!< PWM0 Load                                      */
  _RW_ u_int_32  _0_COUNT;               /*!< PWM0 Counter                                   */
  _RW_ u_int_32  _0_CMPA;                /*!< PWM0 Compare A                                 */
  _RW_ u_int_32  _0_CMPB;                /*!< PWM0 Compare B                                 */
  _RW_ u_int_32  _0_GENA;                /*!< PWM0 Generator A Control                       */
  _RW_ u_int_32  _0_GENB;                /*!< PWM0 Generator B Control                       */
  _RW_ u_int_32  _0_DBCTL;               /*!< PWM0 Dead-Band Control                         */
  _RW_ u_int_32  _0_DBRISE;              /*!< PWM0 Dead-Band Rising-Edge Delay               */
  _RW_ u_int_32  _0_DBFALL;              /*!< PWM0 Dead-Band Falling-Edge-Delay              */
  _RW_ u_int_32  _0_FLTSRC0;             /*!< PWM0 Fault Source 0                            */
  _RW_ u_int_32  _0_FLTSRC1;             /*!< PWM0 Fault Source 1                            */
  _RW_ u_int_32  _0_MINFLTPER;           /*!< PWM0 Minimum Fault Period                      */
  _RW_ u_int_32  _1_CTL;                 /*!< PWM1 Control                                   */
  _RW_ u_int_32  _1_INTEN;               /*!< PWM1 Interrupt and Trigger Enable              */
  _RW_ u_int_32  _1_RIS;                 /*!< PWM1 Raw Interrupt Status                      */
  _RW_ u_int_32  _1_ISC;                 /*!< PWM1 Interrupt Status and Clear                */
  _RW_ u_int_32  _1_LOAD;                /*!< PWM1 Load                                      */
  _RW_ u_int_32  _1_COUNT;               /*!< PWM1 Counter                                   */
  _RW_ u_int_32  _1_CMPA;                /*!< PWM1 Compare A                                 */
  _RW_ u_int_32  _1_CMPB;                /*!< PWM1 Compare B                                 */
  _RW_ u_int_32  _1_GENA;                /*!< PWM1 Generator A Control                       */
  _RW_ u_int_32  _1_GENB;                /*!< PWM1 Generator B Control                       */
  _RW_ u_int_32  _1_DBCTL;               /*!< PWM1 Dead-Band Control                         */
  _RW_ u_int_32  _1_DBRISE;              /*!< PWM1 Dead-Band Rising-Edge Delay               */
  _RW_ u_int_32  _1_DBFALL;              /*!< PWM1 Dead-Band Falling-Edge-Delay              */
  _RW_ u_int_32  _1_FLTSRC0;             /*!< PWM1 Fault Source 0                            */
  _RW_ u_int_32  _1_FLTSRC1;             /*!< PWM1 Fault Source 1                            */
  _RW_ u_int_32  _1_MINFLTPER;           /*!< PWM1 Minimum Fault Period                      */
  _RW_ u_int_32  _2_CTL;                 /*!< PWM2 Control                                   */
  _RW_ u_int_32  _2_INTEN;               /*!< PWM2 Interrupt and Trigger Enable              */
  _RW_ u_int_32  _2_RIS;                 /*!< PWM2 Raw Interrupt Status                      */
  _RW_ u_int_32  _2_ISC;                 /*!< PWM2 Interrupt Status and Clear                */
  _RW_ u_int_32  _2_LOAD;                /*!< PWM2 Load                                      */
  _RW_ u_int_32  _2_COUNT;               /*!< PWM2 Counter                                   */
  _RW_ u_int_32  _2_CMPA;                /*!< PWM2 Compare A                                 */
  _RW_ u_int_32  _2_CMPB;                /*!< PWM2 Compare B                                 */
  _RW_ u_int_32  _2_GENA;                /*!< PWM2 Generator A Control                       */
  _RW_ u_int_32  _2_GENB;                /*!< PWM2 Generator B Control                       */
  _RW_ u_int_32  _2_DBCTL;               /*!< PWM2 Dead-Band Control                         */
  _RW_ u_int_32  _2_DBRISE;              /*!< PWM2 Dead-Band Rising-Edge Delay               */
  _RW_ u_int_32  _2_DBFALL;              /*!< PWM2 Dead-Band Falling-Edge-Delay              */
  _RW_ u_int_32  _2_FLTSRC0;             /*!< PWM2 Fault Source 0                            */
  _RW_ u_int_32  _2_FLTSRC1;             /*!< PWM2 Fault Source 1                            */
  _RW_ u_int_32  _2_MINFLTPER;           /*!< PWM2 Minimum Fault Period                      */
  _RW_ u_int_32  _3_CTL;                 /*!< PWM3 Control                                   */
  _RW_ u_int_32  _3_INTEN;               /*!< PWM3 Interrupt and Trigger Enable              */
  _RW_ u_int_32  _3_RIS;                 /*!< PWM3 Raw Interrupt Status                      */
  _RW_ u_int_32  _3_ISC;                 /*!< PWM3 Interrupt Status and Clear                */
  _RW_ u_int_32  _3_LOAD;                /*!< PWM3 Load                                      */
  _RW_ u_int_32  _3_COUNT;               /*!< PWM3 Counter                                   */
  _RW_ u_int_32  _3_CMPA;                /*!< PWM3 Compare A                                 */
  _RW_ u_int_32  _3_CMPB;                /*!< PWM3 Compare B                                 */
  _RW_ u_int_32  _3_GENA;                /*!< PWM3 Generator A Control                       */
  _RW_ u_int_32  _3_GENB;                /*!< PWM3 Generator B Control                       */
  _RW_ u_int_32  _3_DBCTL;               /*!< PWM3 Dead-Band Control                         */
  _RW_ u_int_32  _3_DBRISE;              /*!< PWM3 Dead-Band Rising-Edge Delay               */
  _RW_ u_int_32  _3_DBFALL;              /*!< PWM3 Dead-Band Falling-Edge-Delay              */
  _RW_ u_int_32  _3_FLTSRC0;             /*!< PWM3 Fault Source 0                            */
  _RW_ u_int_32  _3_FLTSRC1;             /*!< PWM3 Fault Source 1                            */
  _RW_ u_int_32  _3_MINFLTPER;           /*!< PWM3 Minimum Fault Period                      */
  _RO_ u_int_32  RESERVED1[432];
  _RW_ u_int_32  _0_FLTSEN;              /*!< PWM0 Fault Pin Logic Sense                     */
  _RO_ u_int_32  _0_FLTSTAT0;            /*!< PWM0 Fault Status 0                            */
  _RO_ u_int_32  _0_FLTSTAT1;            /*!< PWM0 Fault Status 1                            */
  _RO_ u_int_32  RESERVED2[29];
  _RW_ u_int_32  _1_FLTSEN;              /*!< PWM1 Fault Pin Logic Sense                     */
  _RO_ u_int_32  _1_FLTSTAT0;            /*!< PWM1 Fault Status 0                            */
  _RO_ u_int_32  _1_FLTSTAT1;            /*!< PWM1 Fault Status 1                            */
  _RO_ u_int_32  RESERVED3[30];
  _RO_ u_int_32  _2_FLTSTAT0;            /*!< PWM2 Fault Status 0                            */
  _RO_ u_int_32  _2_FLTSTAT1;            /*!< PWM2 Fault Status 1                            */
  _RO_ u_int_32  RESERVED4[30];
  _RO_ u_int_32  _3_FLTSTAT0;            /*!< PWM3 Fault Status 0                            */
  _RO_ u_int_32  _3_FLTSTAT1;            /*!< PWM3 Fault Status 1                            */
  _RO_ u_int_32  RESERVED5[397];
  _RW_ u_int_32  PP;                     /*!< PWM Peripheral Properties                      */
} PWM0_t;


/* ================================================================================ */
/* ================                      QEI0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for QEI0 peripheral (QEI0)
  */

typedef struct {                 /*!< QEI0 Structure                      */
  _RW_ u_int_32  CTL;            /*!< QEI Control                         */
  _RW_ u_int_32  STAT;           /*!< QEI Status                          */
  _RW_ u_int_32  POS;            /*!< QEI Position                        */
  _RW_ u_int_32  MAXPOS;         /*!< QEI Maximum Position                */
  _RW_ u_int_32  LOAD;           /*!< QEI Timer Load                      */
  _RW_ u_int_32  TIME;           /*!< QEI Timer                           */
  _RW_ u_int_32  COUNT;          /*!< QEI Velocity Counter                */
  _RW_ u_int_32  SPEED;          /*!< QEI Velocity                        */
  _RW_ u_int_32  INTEN;          /*!< QEI Interrupt Enable                */
  _RW_ u_int_32  RIS;            /*!< QEI Raw Interrupt Status            */
  _RW_ u_int_32  ISC;            /*!< QEI Interrupt Status and Clear      */
} QEI0_t;


/* ================================================================================ */
/* ================                     TIMERx                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for TIMER0 peripheral (TIMERx)
  */

typedef struct {                         /*!< TIMERx Structure                  */
  _RW_ u_int_32  CFG;                    /*!< GPTM Configuration                */
  _RW_ u_int_32  TAMR;                   /*!< GPTM Timer A Mode                 */
  _RW_ u_int_32  TBMR;                   /*!< GPTM Timer B Mode                 */
  _RW_ u_int_32  CTL;                    /*!< GPTM Control                      */
  _RW_ u_int_32  SYNC;                   /*!< GPTM Synchronize                  */
  _RO_  u_int_32  RESERVED0;
  _RW_ u_int_32  IMR;                    /*!< GPTM Interrupt Mask               */
  _RW_ u_int_32  RIS;                    /*!< GPTM Raw Interrupt Status         */
  _RW_ u_int_32  MIS;                    /*!< GPTM Masked Interrupt Status      */
  _WO_  u_int_32  ICR;                    /*!< GPTM Interrupt Clear              */
  _RW_ u_int_32  TAILR;                  /*!< GPTM Timer A Interval Load        */
  _RW_ u_int_32  TBILR;                  /*!< GPTM Timer B Interval Load        */
  _RW_ u_int_32  TAMATCHR;               /*!< GPTM Timer A Match                */
  _RW_ u_int_32  TBMATCHR;               /*!< GPTM Timer B Match                */
  _RW_ u_int_32  TAPR;                   /*!< GPTM Timer A Prescale             */
  _RW_ u_int_32  TBPR;                   /*!< GPTM Timer B Prescale             */
  _RW_ u_int_32  TAPMR;                  /*!< GPTM TimerA Prescale Match        */
  _RW_ u_int_32  TBPMR;                  /*!< GPTM TimerB Prescale Match        */
  _RW_ u_int_32  TAR;                    /*!< GPTM Timer A                      */
  _RW_ u_int_32  TBR;                    /*!< GPTM Timer B                      */
  _RW_ u_int_32  TAV;                    /*!< GPTM Timer A Value                */
  _RW_ u_int_32  TBV;                    /*!< GPTM Timer B Value                */
  _RW_ u_int_32  RTCPD;                  /*!< GPTM RTC Predivide                */
  _RW_ u_int_32  TAPS;                   /*!< GPTM Timer A Prescale Snapshot    */
  _RW_ u_int_32  TBPS;                   /*!< GPTM Timer B Prescale Snapshot    */
  _RW_ u_int_32  TAPV;                   /*!< GPTM Timer A Prescale Value       */
  _RW_ u_int_32  TBPV;                   /*!< GPTM Timer B Prescale Value       */
  _RO_  u_int_32  RESERVED1[981];
  _RW_ u_int_32  PP;                     /*!< GPTM Peripheral Properties        */
} Timer_RegDef_t;


/* ================================================================================ */
/* ================                     WTIMERx                    ================ */
/* ================================================================================ */


/**
  * @brief Register map for WTIMER0 peripheral (WTIMERx)
  */

typedef struct {                          /*!< WTIMERx Structure                     */
  _RW_ u_int_32  CFG;                     /*!< GPTM Configuration                    */
  _RW_ u_int_32  TAMR;                    /*!< GPTM Timer A Mode                     */
  _RW_ u_int_32  TBMR;                    /*!< GPTM Timer B Mode                     */
  _RW_ u_int_32  CTL;                     /*!< GPTM Control                          */
  _RW_ u_int_32  SYNC;                    /*!< GPTM Synchronize                      */
  _RO_ u_int_32  RESERVED0;
  _RW_ u_int_32  IMR;                     /*!< GPTM Interrupt Mask                   */
  _RW_ u_int_32  RIS;                     /*!< GPTM Raw Interrupt Status             */
  _RW_ u_int_32  MIS;                     /*!< GPTM Masked Interrupt Status          */
  _WO_ u_int_32  ICR;                     /*!< GPTM Interrupt Clear                  */
  _RW_ u_int_32  TAILR;                   /*!< GPTM Timer A Interval Load            */
  _RW_ u_int_32  TBILR;                   /*!< GPTM Timer B Interval Load            */
  _RW_ u_int_32  TAMATCHR;                /*!< GPTM Timer A Match                    */
  _RW_ u_int_32  TBMATCHR;                /*!< GPTM Timer B Match                    */
  _RW_ u_int_32  TAPR;                    /*!< GPTM Timer A Prescale                 */
  _RW_ u_int_32  TBPR;                    /*!< GPTM Timer B Prescale                 */
  _RW_ u_int_32  TAPMR;                   /*!< GPTM TimerA Prescale Match            */
  _RW_ u_int_32  TBPMR;                   /*!< GPTM TimerB Prescale Match            */
  _RW_ u_int_32  TAR;                     /*!< GPTM Timer A                          */
  _RW_ u_int_32  TBR;                     /*!< GPTM Timer B                          */
  _RW_ u_int_32  TAV;                     /*!< GPTM Timer A Value                    */
  _RW_ u_int_32  TBV;                     /*!< GPTM Timer B Value                    */
  _RW_ u_int_32  RTCPD;                   /*!< GPTM RTC Predivide                    */
  _RW_ u_int_32  TAPS;                    /*!< GPTM Timer A Prescale Snapshot        */
  _RW_ u_int_32  TBPS;                    /*!< GPTM Timer B Prescale Snapshot        */
  _RW_ u_int_32  TAPV;                    /*!< GPTM Timer A Prescale Value           */
  _RW_ u_int_32  TBPV;                    /*!< GPTM Timer B Prescale Value           */
  _RO_ u_int_32  RESERVED1[981];
  _RW_ u_int_32  PP;                      /*!< GPTM Peripheral Properties            */
} W_Timer_RegDef_t;


/* ================================================================================ */
/* ================                      ADC                       ================ */
/* ================================================================================ */


/**
  * @brief Register map for ADC0 peripheral (ADC0)
  */

typedef struct {                     /*!< ADC0 Structure                                           */
  _RW_ u_int_32  ACTSS;              /*!< ADC Active Sample Sequencer                              */
  _RW_ u_int_32  RIS;                /*!< ADC Raw Interrupt Status                                 */
  _RW_ u_int_32  IM;                 /*!< ADC Interrupt Mask                                       */
  _RW_ u_int_32  ISC;                /*!< ADC Interrupt Status and Clear                           */
  _RW_ u_int_32  OSTAT;              /*!< ADC Overflow Status                                      */
  _RW_ u_int_32  EMUX;               /*!< ADC Event Multiplexer Select                             */
  _RW_ u_int_32  USTAT;              /*!< ADC Underflow Status                                     */
  _RW_ u_int_32  TSSEL;              /*!< ADC Trigger Source Select                                */
  _RW_ u_int_32  SSPRI;              /*!< ADC Sample Sequencer Priority                            */
  _RW_ u_int_32  SPC;                /*!< ADC Sample Phase Control                                 */
  _RW_ u_int_32  PSSI;               /*!< ADC Processor Sample Sequence Initiate                   */
  _RO_  u_int_32  RESERVED0;
  _RW_ u_int_32  SAC;                /*!< ADC Sample Averaging Control                             */
  _RW_ u_int_32  DCISC;              /*!< ADC Digital Comparator Interrupt Status and Clear        */
  _RW_ u_int_32  CTL;                /*!< ADC Control                                              */
  _RO_  u_int_32  RESERVED1;
  _RW_ u_int_32  SSMUX0;             /*!< ADC Sample Sequence Input Multiplexer Select 0           */
  _RW_ u_int_32  SSCTL0;             /*!< ADC Sample Sequence Control 0                            */
  _RW_ u_int_32  SSFIFO0;            /*!< ADC Sample Sequence Result FIFO 0                        */
  _RW_ u_int_32  SSFSTAT0;           /*!< ADC Sample Sequence FIFO 0 Status                        */
  _RW_ u_int_32  SSOP0;              /*!< ADC Sample Sequence 0 Operation                          */
  _RW_ u_int_32  SSDC0;              /*!< ADC Sample Sequence 0 Digital Comparator Select          */
  _RO_  u_int_32  RESERVED2[2];
  _RW_ u_int_32  SSMUX1;             /*!< ADC Sample Sequence Input Multiplexer Select 1           */
  _RW_ u_int_32  SSCTL1;             /*!< ADC Sample Sequence Control 1                            */
  _RW_ u_int_32  SSFIFO1;            /*!< ADC Sample Sequence Result FIFO 1                        */
  _RW_ u_int_32  SSFSTAT1;           /*!< ADC Sample Sequence FIFO 1 Status                        */
  _RW_ u_int_32  SSOP1;              /*!< ADC Sample Sequence 1 Operation                          */
  _RW_ u_int_32  SSDC1;              /*!< ADC Sample Sequence 1 Digital Comparator Select          */
  _RO_  u_int_32  RESERVED3[2];
  _RW_ u_int_32  SSMUX2;             /*!< ADC Sample Sequence Input Multiplexer Select 2           */
  _RW_ u_int_32  SSCTL2;             /*!< ADC Sample Sequence Control 2                            */
  _RW_ u_int_32  SSFIFO2;            /*!< ADC Sample Sequence Result FIFO 2                        */
  _RW_ u_int_32  SSFSTAT2;           /*!< ADC Sample Sequence FIFO 2 Status                        */
  _RW_ u_int_32  SSOP2;              /*!< ADC Sample Sequence 2 Operation                          */
  _RW_ u_int_32  SSDC2;              /*!< ADC Sample Sequence 2 Digital Comparator Select          */
  _RO_  u_int_32  RESERVED4[2];
  _RW_ u_int_32  SSMUX3;             /*!< ADC Sample Sequence Input Multiplexer Select 3           */
  _RW_ u_int_32  SSCTL3;             /*!< ADC Sample Sequence Control 3                            */
  _RW_ u_int_32  SSFIFO3;            /*!< ADC Sample Sequence Result FIFO 3                        */
  _RW_ u_int_32  SSFSTAT3;           /*!< ADC Sample Sequence FIFO 3 Status                        */
  _RW_ u_int_32  SSOP3;              /*!< ADC Sample Sequence 3 Operation                          */
  _RW_ u_int_32  SSDC3;              /*!< ADC Sample Sequence 3 Digital Comparator Select          */
  _RO_  u_int_32  RESERVED5[786];
  _WO_  u_int_32  DCRIC;              /*!< ADC Digital Comparator Reset Initial Conditions          */
  _RO_  u_int_32  RESERVED6[63];
  _RW_ u_int_32  DCCTL0;             /*!< ADC Digital Comparator Control 0                         */
  _RW_ u_int_32  DCCTL1;             /*!< ADC Digital Comparator Control 1                         */
  _RW_ u_int_32  DCCTL2;             /*!< ADC Digital Comparator Control 2                         */
  _RW_ u_int_32  DCCTL3;             /*!< ADC Digital Comparator Control 3                         */
  _RW_ u_int_32  DCCTL4;             /*!< ADC Digital Comparator Control 4                         */
  _RW_ u_int_32  DCCTL5;             /*!< ADC Digital Comparator Control 5                         */
  _RW_ u_int_32  DCCTL6;             /*!< ADC Digital Comparator Control 6                         */
  _RW_ u_int_32  DCCTL7;             /*!< ADC Digital Comparator Control 7                         */
  _RO_ u_int_32  RESERVED7[8];
  _RW_ u_int_32  DCCMP0;             /*!< ADC Digital Comparator Range 0                           */
  _RW_ u_int_32  DCCMP1;             /*!< ADC Digital Comparator Range 1                           */
  _RW_ u_int_32  DCCMP2;             /*!< ADC Digital Comparator Range 2                           */
  _RW_ u_int_32  DCCMP3;             /*!< ADC Digital Comparator Range 3                           */
  _RW_ u_int_32  DCCMP4;             /*!< ADC Digital Comparator Range 4                           */
  _RW_ u_int_32  DCCMP5;             /*!< ADC Digital Comparator Range 5                           */
  _RW_ u_int_32  DCCMP6;             /*!< ADC Digital Comparator Range 6                           */
  _RW_ u_int_32  DCCMP7;             /*!< ADC Digital Comparator Range 7                           */
  _RO_  u_int_32  RESERVED8[88];
  _RW_ u_int_32  PP;                 /*!< ADC Peripheral Properties                                */
  _RW_ u_int_32  PC;                 /*!< ADC Peripheral Configuration                             */
  _RW_ u_int_32  CC;                 /*!< ADC Clock Configuration                                  */
} ADC_RegDef_t;


/* ================================================================================ */
/* ================                      COMP                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for Analog COMP peripheral (COMP)
  */

typedef struct {                    /*!< COMP Structure                               */
  _RW_ u_int_32  ACMIS;             /*!< Analog Comparator Masked Interrupt Status    */
  _RW_ u_int_32  ACRIS;             /*!< Analog Comparator Raw Interrupt Status       */
  _RW_ u_int_32  ACINTEN;           /*!< Analog Comparator Interrupt Enable           */
  _RO_  u_int_32  RESERVED0;
  _RW_ u_int_32  ACREFCTL;          /*!< Analog Comparator Reference Voltage Control  */
  _RO_  u_int_32  RESERVED1[3];
  _RW_ u_int_32  ACSTAT0;           /*!< Analog Comparator Status 0                   */
  _RW_ u_int_32  ACCTL0;            /*!< Analog Comparator Control 0                  */
  _RO_  u_int_32  RESERVED2[6];
  _RW_ u_int_32  ACSTAT1;           /*!< Analog Comparator Status 1                   */
  _RW_ u_int_32  ACCTL1;            /*!< Analog Comparator Control 1                  */
  _RO_  u_int_32  RESERVED3[990];
  _RW_ u_int_32  PP;                /*!< Analog Comparator Peripheral Properties      */
} AN_COMP_t;


/* ================================================================================ */
/* ================                      CAN                       ================ */
/* ================================================================================ */


/**
  * @brief Register map for CAN0 peripheral (CAN)
  */

typedef struct {                           /*!< CAN Structure                        */
  _RW_ u_int_32  CTL;                      /*!< CAN Control                           */
  _RW_ u_int_32  STS;                      /*!< CAN Status                            */
  _RW_ u_int_32  ERR;                      /*!< CAN Error Counter                     */
  _RW_ u_int_32  BIT;                      /*!< CAN Bit Timing                        */
  _RW_ u_int_32  INT;                      /*!< CAN Interrupt                         */
  _RW_ u_int_32  TST;                      /*!< CAN Test                              */
  _RW_ u_int_32  BRPE;                     /*!< CAN Baud Rate Prescaler Extension     */
  _RO_  u_int_32  RESERVED0;
  _RW_ u_int_32  IF1CRQ;                   /*!< CAN IF1 Command Request               */

  union {
    _RW_ u_int_32  IF1CMSK_CAN_ALT;       /*!< CAN IF1 Command Mask                  */
    _RW_ u_int_32  IF1CMSK;                /*!< CAN IF1 Command Mask                  */
  };
  _RW_ u_int_32  IF1MSK1;                  /*!< CAN IF1 Mask 1                        */
  _RW_ u_int_32  IF1MSK2;                  /*!< CAN IF1 Mask 2                        */
  _RW_ u_int_32  IF1ARB1;                  /*!< CAN IF1 Arbitration 1                 */
  _RW_ u_int_32  IF1ARB2;                  /*!< CAN IF1 Arbitration 2                 */
  _RW_ u_int_32  IF1MCTL;                  /*!< CAN IF1 Message Control               */
  _RW_ u_int_32  IF1DA1;                   /*!< CAN IF1 Data A1                       */
  _RW_ u_int_32  IF1DA2;                   /*!< CAN IF1 Data A2                       */
  _RW_ u_int_32  IF1DB1;                   /*!< CAN IF1 Data B1                       */
  _RW_ u_int_32  IF1DB2;                   /*!< CAN IF1 Data B2                       */
  _RO_  u_int_32  RESERVED1[13];
  _RW_ u_int_32  IF2CRQ;                   /*!< CAN IF2 Command Request               */

  union {
    _RW_ u_int_32  IF2CMSK_CAN_ALT;       /*!< CAN IF2 Command Mask                  */
    _RW_ u_int_32  IF2CMSK;                /*!< CAN IF2 Command Mask                  */
  };
  _RW_ u_int_32  IF2MSK1;                  /*!< CAN IF2 Mask 1                        */
  _RW_ u_int_32  IF2MSK2;                  /*!< CAN IF2 Mask 2                        */
  _RW_ u_int_32  IF2ARB1;                  /*!< CAN IF2 Arbitration 1                 */
  _RW_ u_int_32  IF2ARB2;                  /*!< CAN IF2 Arbitration 2                 */
  _RW_ u_int_32  IF2MCTL;                  /*!< CAN IF2 Message Control               */
  _RW_ u_int_32  IF2DA1;                   /*!< CAN IF2 Data A1                       */
  _RW_ u_int_32  IF2DA2;                   /*!< CAN IF2 Data A2                       */
  _RW_ u_int_32  IF2DB1;                   /*!< CAN IF2 Data B1                       */
  _RW_ u_int_32  IF2DB2;                   /*!< CAN IF2 Data B2                       */
  _RO_  u_int_32  RESERVED2[21];
  _RW_ u_int_32  TXRQ1;                    /*!< CAN Transmission Request 1            */
  _RW_ u_int_32  TXRQ2;                    /*!< CAN Transmission Request 2            */
  _RO_  u_int_32  RESERVED3[6];
  _RW_ u_int_32  NWDA1;                    /*!< CAN New Data 1                        */
  _RW_ u_int_32  NWDA2;                    /*!< CAN New Data 2                        */
  _RO_  u_int_32  RESERVED4[6];
  _RW_ u_int_32  MSG1INT;                  /*!< CAN Message 1 Interrupt Pending       */
  _RW_ u_int_32  MSG2INT;                  /*!< CAN Message 2 Interrupt Pending       */
  _RO_  u_int_32  RESERVED5[6];
  _RW_ u_int_32  MSG1VAL;                  /*!< CAN Message 1 Valid                   */
  _RW_ u_int_32  MSG2VAL;                  /*!< CAN Message 2 Valid                   */

} CAN_RegDef_t;


/* ================================================================================ */
/* ================                      USB0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for USB0 peripheral (USB0)
  */

typedef struct {                                    /*!< USB0 Structure                                                        */
  _RW_ u_int_8   FADDR;                             /*!< USB Device Functional Address                                         */
  _RW_ u_int_8   POWER;                             /*!< USB Power                                                             */
  _RW_ u_int_16  TXIS;                              /*!< USB Transmit Interrupt Status                                         */
  _RW_ u_int_16  RXIS;                              /*!< USB Receive Interrupt Status                                          */
  _RW_ u_int_16  TXIE;                              /*!< USB Transmit Interrupt Enable                                         */
  _RW_ u_int_16  RXIE;                              /*!< USB Receive Interrupt Enable                                          */

  union {
    _RW_ u_int_8   IS_USB0_ALT;                     /*!< USB General Interrupt Status                                          */
    _RW_ u_int_8   IS;                              /*!< USB General Interrupt Status                                          */
  };

  union {
    _RW_ u_int_8   IE_USB0_ALT;                     /*!< USB Interrupt Enable                                                  */
    _RW_ u_int_8   IE;                              /*!< USB Interrupt Enable                                                  */
  };
  _RW_ u_int_16  FRAME;                             /*!< USB Frame Value                                                       */
  _RW_ u_int_8   EPIDX;                             /*!< USB Endpoint Index                                                    */
  _RW_ u_int_8   TEST;                              /*!< USB Test Mode                                                         */
  _RO_  u_int_32  RESERVED0[4];
  _RW_ u_int_32  FIFO0;                             /*!< USB FIFO Endpoint 0                                                   */
  _RW_ u_int_32  FIFO1;                             /*!< USB FIFO Endpoint 1                                                   */
  _RW_ u_int_32  FIFO2;                             /*!< USB FIFO Endpoint 2                                                   */
  _RW_ u_int_32  FIFO3;                             /*!< USB FIFO Endpoint 3                                                   */
  _RW_ u_int_32  FIFO4;                             /*!< USB FIFO Endpoint 4                                                   */
  _RW_ u_int_32  FIFO5;                             /*!< USB FIFO Endpoint 5                                                   */
  _RW_ u_int_32  FIFO6;                             /*!< USB FIFO Endpoint 6                                                   */
  _RW_ u_int_32  FIFO7;                             /*!< USB FIFO Endpoint 7                                                   */
  _RO_  u_int_32  RESERVED1[8];
  _RW_ u_int_8   DEVCTL;                            /*!< USB Device Control                                                    */
  _RO_  u_int_8   RESERVED2[1];
  _RW_ u_int_8   TXFIFOSZ;                          /*!< USB Transmit Dynamic FIFO Sizing                                      */
  _RW_ u_int_8   RXFIFOSZ;                          /*!< USB Receive Dynamic FIFO Sizing                                       */
  _RW_ u_int_16  TXFIFOADD;                         /*!< USB Transmit FIFO Start Address                                       */
  _RW_ u_int_16  RXFIFOADD;                         /*!< USB Receive FIFO Start Address                                        */
  _RO_  u_int_32  RESERVED3[4];
  _RO_  u_int_16  RESERVED4;
  _RW_ u_int_8   CONTIM;                            /*!< USB Connect Timing                                                    */
  _RW_ u_int_8   VPLEN;                             /*!< USB OTG VBUS Pulse Timing                                             */
  _RO_  u_int_8   RESERVED5[1];
  _RW_ u_int_8   FSEOF;                             /*!< USB Full-Speed Last Transaction to End of Frame Timing                */
  _RW_ u_int_8   LSEOF;                             /*!< USB Low-Speed Last Transaction to End of Frame Timing                 */
  _RO_  u_int_8   RESERVED6[1];
  _RW_ u_int_8   TXFUNCADDR0;                       /*!< USB Transmit Functional Address Endpoint 0                            */
  _RO_  u_int_8   RESERVED7[1];
  _RW_ u_int_8   TXHUBADDR0;                        /*!< USB Transmit Hub Address Endpoint 0                                   */
  _RW_ u_int_8   TXHUBPORT0;                        /*!< USB Transmit Hub Port Endpoint 0                                      */
  _RO_  u_int_32  RESERVED8;
  _RW_ u_int_8   TXFUNCADDR1;                       /*!< USB Transmit Functional Address Endpoint 1                            */
  _RO_  u_int_8   RESERVED9[1];
  _RW_ u_int_8   TXHUBADDR1;                        /*!< USB Transmit Hub Address Endpoint 1                                   */
  _RW_ u_int_8   TXHUBPORT1;                        /*!< USB Transmit Hub Port Endpoint 1                                      */
  _RW_ u_int_8   RXFUNCADDR1;                       /*!< USB Receive Functional Address Endpoint 1                             */
  _RO_  u_int_8   RESERVED10[1];
  _RW_ u_int_8   RXHUBADDR1;                        /*!< USB Receive Hub Address Endpoint 1                                    */
  _RW_ u_int_8   RXHUBPORT1;                        /*!< USB Receive Hub Port Endpoint 1                                       */
  _RW_ u_int_8   TXFUNCADDR2;                       /*!< USB Transmit Functional Address Endpoint 2                            */
  _RO_  u_int_8   RESERVED11[1];
  _RW_ u_int_8   TXHUBADDR2;                        /*!< USB Transmit Hub Address Endpoint 2                                   */
  _RW_ u_int_8   TXHUBPORT2;                        /*!< USB Transmit Hub Port Endpoint 2                                      */
  _RW_ u_int_8   RXFUNCADDR2;                       /*!< USB Receive Functional Address Endpoint 2                             */
  _RO_  u_int_8   RESERVED12[1];
  _RW_ u_int_8   RXHUBADDR2;                        /*!< USB Receive Hub Address Endpoint 2                                    */
  _RW_ u_int_8   RXHUBPORT2;                        /*!< USB Receive Hub Port Endpoint 2                                       */
  _RW_ u_int_8   TXFUNCADDR3;                       /*!< USB Transmit Functional Address Endpoint 3                            */
  _RO_  u_int_8   RESERVED13[1];
  _RW_ u_int_8   TXHUBADDR3;                        /*!< USB Transmit Hub Address Endpoint 3                                   */
  _RW_ u_int_8   TXHUBPORT3;                        /*!< USB Transmit Hub Port Endpoint 3                                      */
  _RW_ u_int_8   RXFUNCADDR3;                       /*!< USB Receive Functional Address Endpoint 3                             */
  _RO_  u_int_8   RESERVED14[1];
  _RW_ u_int_8   RXHUBADDR3;                        /*!< USB Receive Hub Address Endpoint 3                                    */
  _RW_ u_int_8   RXHUBPORT3;                        /*!< USB Receive Hub Port Endpoint 3                                       */
  _RW_ u_int_8   TXFUNCADDR4;                       /*!< USB Transmit Functional Address Endpoint 4                            */
  _RO_  u_int_8   RESERVED15[1];
  _RW_ u_int_8   TXHUBADDR4;                        /*!< USB Transmit Hub Address Endpoint 4                                   */
  _RW_ u_int_8   TXHUBPORT4;                        /*!< USB Transmit Hub Port Endpoint 4                                      */
  _RW_ u_int_8   RXFUNCADDR4;                       /*!< USB Receive Functional Address Endpoint 4                             */
  _RO_  u_int_8   RESERVED16[1];
  _RW_ u_int_8   RXHUBADDR4;                        /*!< USB Receive Hub Address Endpoint 4                                    */
  _RW_ u_int_8   RXHUBPORT4;                        /*!< USB Receive Hub Port Endpoint 4                                       */
  _RW_ u_int_8   TXFUNCADDR5;                       /*!< USB Transmit Functional Address Endpoint 5                            */
  _RO_  u_int_8   RESERVED17[1];
  _RW_ u_int_8   TXHUBADDR5;                        /*!< USB Transmit Hub Address Endpoint 5                                   */
  _RW_ u_int_8   TXHUBPORT5;                        /*!< USB Transmit Hub Port Endpoint 5                                      */
  _RW_ u_int_8   RXFUNCADDR5;                       /*!< USB Receive Functional Address Endpoint 5                             */
  _RO_  u_int_8   RESERVED18[1];
  _RW_ u_int_8   RXHUBADDR5;                        /*!< USB Receive Hub Address Endpoint 5                                    */
  _RW_ u_int_8   RXHUBPORT5;                        /*!< USB Receive Hub Port Endpoint 5                                       */
  _RW_ u_int_8   TXFUNCADDR6;                       /*!< USB Transmit Functional Address Endpoint 6                            */
  _RO_  u_int_8   RESERVED19[1];
  _RW_ u_int_8   TXHUBADDR6;                        /*!< USB Transmit Hub Address Endpoint 6                                   */
  _RW_ u_int_8   TXHUBPORT6;                        /*!< USB Transmit Hub Port Endpoint 6                                      */
  _RW_ u_int_8   RXFUNCADDR6;                       /*!< USB Receive Functional Address Endpoint 6                             */
  _RO_  u_int_8   RESERVED20[1];
  _RW_ u_int_8   RXHUBADDR6;                        /*!< USB Receive Hub Address Endpoint 6                                    */
  _RW_ u_int_8   RXHUBPORT6;                        /*!< USB Receive Hub Port Endpoint 6                                       */
  _RW_ u_int_8   TXFUNCADDR7;                       /*!< USB Transmit Functional Address Endpoint 7                            */
  _RO_  u_int_8   RESERVED21[1];
  _RW_ u_int_8   TXHUBADDR7;                        /*!< USB Transmit Hub Address Endpoint 7                                   */
  _RW_ u_int_8   TXHUBPORT7;                        /*!< USB Transmit Hub Port Endpoint 7                                      */
  _RW_ u_int_8   RXFUNCADDR7;                       /*!< USB Receive Functional Address Endpoint 7                             */
  _RO_  u_int_8   RESERVED22[1];
  _RW_ u_int_8   RXHUBADDR7;                        /*!< USB Receive Hub Address Endpoint 7                                    */
  _RW_ u_int_8   RXHUBPORT7;                        /*!< USB Receive Hub Port Endpoint 7                                       */
  _RO_  u_int_32  RESERVED23[16];
  _RO_  u_int_16  RESERVED24;

  union
  {
    _WO_  u_int_8   CSRL0_USB0_ALT;                  /*!< USB Control and Status Endpoint 0 Low                                 */
    _WO_  u_int_8   CSRL0;                           /*!< USB Control and Status Endpoint 0 Low                                 */
  };
  _WO_  u_int_8   CSRH0;                             /*!< USB Control and Status Endpoint 0 High                                */
  _RO_  u_int_16  RESERVED25[3];
  _RW_ u_int_8   COUNT0;                            /*!< USB Receive Byte Count Endpoint 0                                     */
  _RO_  u_int_8   RESERVED26[1];
  _RW_ u_int_8   TYPE0;                             /*!< USB Type Endpoint 0                                                   */
  _RW_ u_int_8   NAKLMT;                            /*!< USB NAK Limit                                                         */
  _RO_  u_int_32  RESERVED27;
  _RW_ u_int_16  TXMAXP1;                           /*!< USB Maximum Transmit Data Endpoint 1                                  */

  union
  {
    _RW_ u_int_8   TXCSRL1_USB0_ALT;                /*!< USB Transmit Control and Status Endpoint 1 Low                        */
    _RW_ u_int_8   TXCSRL1;                         /*!< USB Transmit Control and Status Endpoint 1 Low                        */
  };
  _RW_ u_int_8   TXCSRH1;                           /*!< USB Transmit Control and Status Endpoint 1 High                       */
  _RW_ u_int_16  RXMAXP1;                           /*!< USB Maximum Receive Data Endpoint 1                                   */

  union
  {
    _RW_ u_int_8   RXCSRL1_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 1 Low                         */
    _RW_ u_int_8   RXCSRL1;                         /*!< USB Receive Control and Status Endpoint 1 Low                         */
  };

  union
  {
    _RW_ u_int_8   RXCSRH1_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 1 High                        */
    _RW_ u_int_8   RXCSRH1;                         /*!< USB Receive Control and Status Endpoint 1 High                        */
  };
  _RW_ u_int_16  RXCOUNT1;                          /*!< USB Receive Byte Count Endpoint 1                                     */
  _RW_ u_int_8   TXTYPE1;                           /*!< USB Host Transmit Configure Type Endpoint 1                           */

  union
  {
    _RW_ u_int_8   TXINTERVAL1_USB0_ALT;            /*!< USB Host Transmit Interval Endpoint 1                                 */
    _RW_ u_int_8   TXINTERVAL1;                     /*!< USB Host Transmit Interval Endpoint 1                                 */
  };
  _RW_ u_int_8   RXTYPE1;                           /*!< USB Host Configure Receive Type Endpoint 1                            */

  union
  {
    _RW_ u_int_8   RXINTERVAL1_USB0_ALT;            /*!< USB Host Receive Polling Interval Endpoint 1                          */
    _RW_ u_int_8   RXINTERVAL1;                     /*!< USB Host Receive Polling Interval Endpoint 1                          */
  };
  _RO_  u_int_16  RESERVED28;
  _RW_ u_int_16  TXMAXP2;                           /*!< USB Maximum Transmit Data Endpoint 2                                  */

  union
  {
    _RW_ u_int_8   TXCSRL2_USB0_ALT;                /*!< USB Transmit Control and Status Endpoint 2 Low                        */
    _RW_ u_int_8   TXCSRL2;                         /*!< USB Transmit Control and Status Endpoint 2 Low                        */
  };
  _RW_ u_int_8   TXCSRH2;                           /*!< USB Transmit Control and Status Endpoint 2 High                       */
  _RW_ u_int_16  RXMAXP2;                           /*!< USB Maximum Receive Data Endpoint 2                                   */

  union
  {
    _RW_ u_int_8   RXCSRL2_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 2 Low                         */
    _RW_ u_int_8   RXCSRL2;                         /*!< USB Receive Control and Status Endpoint 2 Low                         */
  };

  union
  {
    _RW_ u_int_8   RXCSRH2_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 2 High                        */
    _RW_ u_int_8   RXCSRH2;                         /*!< USB Receive Control and Status Endpoint 2 High                        */
  };
  _RW_ u_int_16  RXCOUNT2;                          /*!< USB Receive Byte Count Endpoint 2                                     */
  _RW_ u_int_8   TXTYPE2;                           /*!< USB Host Transmit Configure Type Endpoint 2                           */

  union
  {
    _RW_ u_int_8   TXINTERVAL2_USB0_ALT;            /*!< USB Host Transmit Interval Endpoint 2                                 */
    _RW_ u_int_8   TXINTERVAL2;                     /*!< USB Host Transmit Interval Endpoint 2                                 */
  };
  _RW_ u_int_8   RXTYPE2;                           /*!< USB Host Configure Receive Type Endpoint 2                            */

  union
  {
    _RW_ u_int_8   RXINTERVAL2_USB0_ALT;            /*!< USB Host Receive Polling Interval Endpoint 2                          */
    _RW_ u_int_8   RXINTERVAL2;                     /*!< USB Host Receive Polling Interval Endpoint 2                          */
  };
  _RO_  u_int_16  RESERVED29;
  _RW_  u_int_16  TXMAXP3;                           /*!< USB Maximum Transmit Data Endpoint 3                                  */

  union
  {
    _RW_ u_int_8   TXCSRL3_USB0_ALT;                /*!< USB Transmit Control and Status Endpoint 3 Low                        */
    _RW_ u_int_8   TXCSRL3;                         /*!< USB Transmit Control and Status Endpoint 3 Low                        */
  };
  _RW_ u_int_8   TXCSRH3;                           /*!< USB Transmit Control and Status Endpoint 3 High                       */
  _RW_ u_int_16  RXMAXP3;                           /*!< USB Maximum Receive Data Endpoint 3                                   */

  union
  {
    _RW_ u_int_8   RXCSRL3_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 3 Low                         */
    _RW_ u_int_8   RXCSRL3;                         /*!< USB Receive Control and Status Endpoint 3 Low                         */
  };

  union
  {
    _RW_ u_int_8   RXCSRH3_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 3 High                        */
    _RW_ u_int_8   RXCSRH3;                         /*!< USB Receive Control and Status Endpoint 3 High                        */
  };
  _RW_ u_int_16  RXCOUNT3;                          /*!< USB Receive Byte Count Endpoint 3                                     */
  _RW_ u_int_8   TXTYPE3;                           /*!< USB Host Transmit Configure Type Endpoint 3                           */

  union
  {
    _RW_ u_int_8   TXINTERVAL3_USB0_ALT;            /*!< USB Host Transmit Interval Endpoint 3                                 */
    _RW_ u_int_8   TXINTERVAL3;                     /*!< USB Host Transmit Interval Endpoint 3                                 */
  };
  _RW_ u_int_8   RXTYPE3;                           /*!< USB Host Configure Receive Type Endpoint 3                            */

  union
  {
    _RW_ u_int_8   RXINTERVAL3_USB0_ALT;            /*!< USB Host Receive Polling Interval Endpoint 3                          */
    _RW_ u_int_8   RXINTERVAL3;                     /*!< USB Host Receive Polling Interval Endpoint 3                          */
  };
  _RO_  u_int_16  RESERVED30;
  _RW_ u_int_16  TXMAXP4;                           /*!< USB Maximum Transmit Data Endpoint 4                                  */

  union
  {
    _RW_ u_int_8   TXCSRL4_USB0_ALT;                /*!< USB Transmit Control and Status Endpoint 4 Low                        */
    _RW_ u_int_8   TXCSRL4;                         /*!< USB Transmit Control and Status Endpoint 4 Low                        */
  };
  _RW_ u_int_8   TXCSRH4;                           /*!< USB Transmit Control and Status Endpoint 4 High                       */
  _RW_ u_int_16  RXMAXP4;                           /*!< USB Maximum Receive Data Endpoint 4                                   */

  union
  {
    _RW_ u_int_8   RXCSRL4_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 4 Low                         */
    _RW_ u_int_8   RXCSRL4;                         /*!< USB Receive Control and Status Endpoint 4 Low                         */
  };

  union
  {
    _RW_ u_int_8   RXCSRH4_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 4 High                        */
    _RW_ u_int_8   RXCSRH4;                         /*!< USB Receive Control and Status Endpoint 4 High                        */
  };
  _RW_ u_int_16  RXCOUNT4;                          /*!< USB Receive Byte Count Endpoint 4                                     */
  _RW_ u_int_8   TXTYPE4;                           /*!< USB Host Transmit Configure Type Endpoint 4                           */

  union
  {
    _RW_ u_int_8   TXINTERVAL4_USB0_ALT;            /*!< USB Host Transmit Interval Endpoint 4                                 */
    _RW_ u_int_8   TXINTERVAL4;                     /*!< USB Host Transmit Interval Endpoint 4                                 */
  };
  _RW_ u_int_8   RXTYPE4;                           /*!< USB Host Configure Receive Type Endpoint 4                            */

  union
  {
    _RW_ u_int_8   RXINTERVAL4_USB0_ALT;            /*!< USB Host Receive Polling Interval Endpoint 4                          */
    _RW_ u_int_8   RXINTERVAL4;                     /*!< USB Host Receive Polling Interval Endpoint 4                          */
  };
  _RO_  u_int_16  RESERVED31;
  _RW_  u_int_16  TXMAXP5;                           /*!< USB Maximum Transmit Data Endpoint 5                                  */

  union
  {
    _RW_ u_int_8   TXCSRL5_USB0_ALT;                /*!< USB Transmit Control and Status Endpoint 5 Low                        */
    _RW_ u_int_8   TXCSRL5;                         /*!< USB Transmit Control and Status Endpoint 5 Low                        */
  };
  _RW_ u_int_8   TXCSRH5;                           /*!< USB Transmit Control and Status Endpoint 5 High                       */
  _RW_ u_int_16  RXMAXP5;                           /*!< USB Maximum Receive Data Endpoint 5                                   */

  union
  {
    _RW_ u_int_8   RXCSRL5_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 5 Low                         */
    _RW_ u_int_8   RXCSRL5;                         /*!< USB Receive Control and Status Endpoint 5 Low                         */
  };

  union
  {
    _RW_ u_int_8   RXCSRH5_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 5 High                        */
    _RW_ u_int_8   RXCSRH5;                         /*!< USB Receive Control and Status Endpoint 5 High                        */
  };
  _RW_ u_int_16  RXCOUNT5;                          /*!< USB Receive Byte Count Endpoint 5                                     */
  _RW_ u_int_8   TXTYPE5;                           /*!< USB Host Transmit Configure Type Endpoint 5                           */

  union
  {
    _RW_ u_int_8   TXINTERVAL5_USB0_ALT;            /*!< USB Host Transmit Interval Endpoint 5                                 */
    _RW_ u_int_8   TXINTERVAL5;                     /*!< USB Host Transmit Interval Endpoint 5                                 */
  };
  _RW_ u_int_8   RXTYPE5;                           /*!< USB Host Configure Receive Type Endpoint 5                            */

  union
  {
    _RW_ u_int_8   RXINTERVAL5_USB0_ALT;            /*!< USB Host Receive Polling Interval Endpoint 5                          */
    _RW_ u_int_8   RXINTERVAL5;                     /*!< USB Host Receive Polling Interval Endpoint 5                          */
  };
  _RO_  u_int_16  RESERVED32;
  _RW_  u_int_16  TXMAXP6;                           /*!< USB Maximum Transmit Data Endpoint 6                                  */

  union
  {
    _RW_ u_int_8   TXCSRL6_USB0_ALT;                /*!< USB Transmit Control and Status Endpoint 6 Low                        */
    _RW_ u_int_8   TXCSRL6;                         /*!< USB Transmit Control and Status Endpoint 6 Low                        */
  };
  _RW_ u_int_8   TXCSRH6;                           /*!< USB Transmit Control and Status Endpoint 6 High                       */
  _RW_ u_int_16  RXMAXP6;                           /*!< USB Maximum Receive Data Endpoint 6                                   */

  union
  {
    _RW_ u_int_8   RXCSRL6_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 6 Low                         */
    _RW_ u_int_8   RXCSRL6;                         /*!< USB Receive Control and Status Endpoint 6 Low                         */
  };

  union
  {
    _RW_ u_int_8   RXCSRH6_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 6 High                        */
    _RW_ u_int_8   RXCSRH6;                         /*!< USB Receive Control and Status Endpoint 6 High                        */
  };
  _RW_ u_int_16  RXCOUNT6;                              /*!< USB Receive Byte Count Endpoint 6                                     */
  _RW_ u_int_8   TXTYPE6;                           /*!< USB Host Transmit Configure Type Endpoint 6                           */

  union
  {
    _RW_ u_int_8   TXINTERVAL6_USB0_ALT;            /*!< USB Host Transmit Interval Endpoint 6                                 */
    _RW_ u_int_8   TXINTERVAL6;                     /*!< USB Host Transmit Interval Endpoint 6                                 */
  };
  _RW_ u_int_8   RXTYPE6;                           /*!< USB Host Configure Receive Type Endpoint 6                            */

  union
  {
    _RW_ u_int_8   RXINTERVAL6_USB0_ALT;            /*!< USB Host Receive Polling Interval Endpoint 6                          */
    _RW_ u_int_8   RXINTERVAL6;                     /*!< USB Host Receive Polling Interval Endpoint 6                          */
  };
  _RO_  u_int_16  RESERVED33;
  _RW_  u_int_16  TXMAXP7;                               /*!< USB Maximum Transmit Data Endpoint 7                                  */

  union
  {
    _RW_ u_int_8   TXCSRL7_USB0_ALT;                /*!< USB Transmit Control and Status Endpoint 7 Low                        */
    _RW_ u_int_8   TXCSRL7;                         /*!< USB Transmit Control and Status Endpoint 7 Low                        */
  };
  _RW_ u_int_8   TXCSRH7;                           /*!< USB Transmit Control and Status Endpoint 7 High                       */
  _RW_ u_int_16  RXMAXP7;                               /*!< USB Maximum Receive Data Endpoint 7                                   */

  union
  {
    _RW_ u_int_8   RXCSRL7_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 7 Low                         */
    _RW_ u_int_8   RXCSRL7;                         /*!< USB Receive Control and Status Endpoint 7 Low                         */
  };

  union
  {
    _RW_ u_int_8   RXCSRH7_USB0_ALT;                /*!< USB Receive Control and Status Endpoint 7 High                        */
    _RW_ u_int_8   RXCSRH7;                         /*!< USB Receive Control and Status Endpoint 7 High                        */
  };
  _RW_ u_int_16  RXCOUNT7;                              /*!< USB Receive Byte Count Endpoint 7                                     */
  _RW_ u_int_8   TXTYPE7;                           /*!< USB Host Transmit Configure Type Endpoint 7                           */

  union
  {
    _RW_ u_int_8   TXINTERVAL7_USB0_ALT;            /*!< USB Host Transmit Interval Endpoint 7                                 */
    _RW_ u_int_8   TXINTERVAL7;                     /*!< USB Host Transmit Interval Endpoint 7                                 */
  };
  _RW_ u_int_8   RXTYPE7;                           /*!< USB Host Configure Receive Type Endpoint 7                            */

  union
  {
    _RW_ u_int_8   RXINTERVAL7_USB0_ALT;            /*!< USB Host Receive Polling Interval Endpoint 7                          */
    _RW_ u_int_8   RXINTERVAL7;                     /*!< USB Host Receive Polling Interval Endpoint 7                          */
  };
  _RO_  u_int_16  RESERVED34[195];
  _RW_ u_int_16  RQPKTCOUNT1;                       /*!< USB Request Packet Count in Block Transfer Endpoint 1                 */
  _RO_  u_int_16  RESERVED35;
  _RW_ u_int_16  RQPKTCOUNT2;                       /*!< USB Request Packet Count in Block Transfer Endpoint 2                 */
  _RO_  u_int_16  RESERVED36;
  _RW_ u_int_16  RQPKTCOUNT3;                       /*!< USB Request Packet Count in Block Transfer Endpoint 3                 */
  _RO_  u_int_16  RESERVED37;
  _RW_ u_int_16  RQPKTCOUNT4;                       /*!< USB Request Packet Count in Block Transfer Endpoint 4                 */
  _RO_  u_int_16  RESERVED38;
  _RW_ u_int_16  RQPKTCOUNT5;                       /*!< USB Request Packet Count in Block Transfer Endpoint 5                 */
  _RO_  u_int_16  RESERVED39;
  _RW_ u_int_16  RQPKTCOUNT6;                       /*!< USB Request Packet Count in Block Transfer Endpoint 6                 */
  _RO_  u_int_16  RESERVED40;
  _RW_ u_int_16  RQPKTCOUNT7;                       /*!< USB Request Packet Count in Block Transfer Endpoint 7                 */
  _RO_  u_int_16  RESERVED41[17];
  _RW_ u_int_16  RXDPKTBUFDIS;                      /*!< USB Receive Double Packet Buffer Disable                              */
  _RW_ u_int_16  TXDPKTBUFDIS;                      /*!< USB Transmit Double Packet Buffer Disable                             */
  _RO_  u_int_32  RESERVED42[47];
  _RW_ u_int_32  EPC;                               /*!< USB External Power Control                                            */
  _RW_ u_int_32  EPCRIS;                            /*!< USB External Power Control Raw Interrupt Status                       */
  _RW_ u_int_32  EPCIM;                             /*!< USB External Power Control Interrupt Mask                             */
  _RW_ u_int_32  EPCISC;                            /*!< USB External Power Control Interrupt Status and Clear                 */
  _RW_ u_int_32  DRRIS;                             /*!< USB Device RESUME Raw Interrupt Status                                */
  _RW_ u_int_32  DRIM;                              /*!< USB Device RESUME Interrupt Mask                                      */
  _WO_  u_int_32  DRISC;                             /*!< USB Device RESUME Interrupt Status and Clear                          */
  _RW_ u_int_32  GPCS;                              /*!< USB General-Purpose Control and Status                                */
  _RO_  u_int_32  RESERVED43[4];
  _RW_ u_int_32  VDC;                               /*!< USB VBUS Droop Control                                                */
  _RW_ u_int_32  VDCRIS;                            /*!< USB VBUS Droop Control Raw Interrupt Status                           */
  _RW_ u_int_32  VDCIM;                             /*!< USB VBUS Droop Control Interrupt Mask                                 */
  _RW_ u_int_32  VDCISC;                            /*!< USB VBUS Droop Control Interrupt Status and Clear                     */
  _RO_  u_int_32  RESERVED44;
  _RW_ u_int_32  IDVRIS;                            /*!< USB ID Valid Detect Raw Interrupt Status                              */
  _RW_ u_int_32  IDVIM;                             /*!< USB ID Valid Detect Interrupt Mask                                    */
  _RW_ u_int_32  IDVISC;                            /*!< USB ID Valid Detect Interrupt Status and Clear                        */
  _RW_ u_int_32  DMASEL;                            /*!< USB DMA Select                                                        */
  _RO_  u_int_32  RESERVED45[731];
  _RW_ u_int_32  PP;                                /*!< USB Peripheral Properties                                             */
} USB0_t;


/* ================================================================================ */
/* ================                     EEPROM                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for EEPROM peripheral (EEPROM)
  */

typedef struct {                      /*!< EEPROM Structure                                                      */
  _RW_ u_int_32  EESIZE;              /*!< EEPROM Size Information                                               */
  _RW_ u_int_32  EEBLOCK;             /*!< EEPROM Current Block                                                  */
  _RW_ u_int_32  EEOFFSET;            /*!< EEPROM Current Offset                                                 */
  _RO_  u_int_32  RESERVED0;
  _RW_ u_int_32  EERDWR;              /*!< EEPROM Read-Write                                                     */
  _RW_ u_int_32  EERDWRINC;           /*!< EEPROM Read-Write with Increment                                      */
  _RW_ u_int_32  EEDONE;              /*!< EEPROM Done Status                                                    */
  _RW_ u_int_32  EESUPP;              /*!< EEPROM Support Control and Status                                     */
  _RW_ u_int_32  EEUNLOCK;            /*!< EEPROM Unlock                                                         */
  _RO_  u_int_32  RESERVED1[3];
  _RW_ u_int_32  EEPROT;              /*!< EEPROM Protection                                                     */
  _RW_ u_int_32  EEPASS0;             /*!< EEPROM Password                                                       */
  _RW_ u_int_32  EEPASS1;             /*!< EEPROM Password                                                       */
  _RW_ u_int_32  EEPASS2;             /*!< EEPROM Password                                                       */
  _RW_ u_int_32  EEINT;               /*!< EEPROM Interrupt                                                      */
  _RO_  u_int_32  RESERVED2[3];
  _RW_ u_int_32  EEHIDE;              /*!< EEPROM Block Hide                                                     */
  _RO_  u_int_32  RESERVED3[11];
  _RW_ u_int_32  EEDBGME;             /*!< EEPROM Debug Mass Erase                                               */
  _RO_  u_int_32  RESERVED4[975];
  _RW_ u_int_32  PP;                  /*!< EEPROM Peripheral Properties                                          */
} EEPROM_t;


/* ================================================================================ */
/* ================                     SYSEXC                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for SYSEXC peripheral (SYSEXC)
  */

typedef struct {                          /*!< SYSEXC Structure                                                      */
  _RW_ u_int_32  RIS;                     /*!< System Exception Raw Interrupt Status                                 */
  _RW_ u_int_32  IM;                      /*!< System Exception Interrupt Mask                                       */
  _RW_ u_int_32  MIS;                     /*!< System Exception Masked Interrupt Status                              */
  _WO_  u_int_32  IC;                      /*!< System Exception Interrupt Clear                                      */
} SYSEXC_Type;


/* ================================================================================ */
/* ================                       HIB                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for HIB peripheral (HIB)
  */

typedef struct {                            /*!< HIB Structure                                                         */
  _RW_ u_int_32  RTCC;                      /*!< Hibernation RTC Counter                                               */
  _RW_ u_int_32  RTCM0;                     /*!< Hibernation RTC Match 0                                               */
  _RO_  u_int_32  RESERVED0;
  _RW_ u_int_32  RTCLD;                     /*!< Hibernation RTC Load                                                  */
  _RW_ u_int_32  CTL;                       /*!< Hibernation Control                                                   */
  _RW_ u_int_32  IM;                        /*!< Hibernation Interrupt Mask                                            */
  _RW_ u_int_32  RIS;                       /*!< Hibernation Raw Interrupt Status                                      */
  _RW_ u_int_32  MIS;                       /*!< Hibernation Masked Interrupt Status                                   */
  _RW_ u_int_32  IC;                        /*!< Hibernation Interrupt Clear                                           */
  _RW_ u_int_32  RTCT;                      /*!< Hibernation RTC Trim                                                  */
  _RW_ u_int_32  RTCSS;                     /*!< Hibernation RTC Sub Seconds                                           */
  _RO_  u_int_32  RESERVED1;
  _RW_ u_int_32  DATA;                      /*!< Hibernation Data                                                      */
} HIB_t;


/* ================================================================================ */
/* ================                   FLASH_CTRL                   ================ */
/* ================================================================================ */


/**
  * @brief Register map for FLASH_CTRL peripheral (FLASH_CTRL)
  */

typedef struct {                             /*!< FLASH_CTRL Structure                                                  */
  _RW_ u_int_32  FMA;                        /*!< Flash Memory Address                                                  */
  _RW_ u_int_32  FMD;                        /*!< Flash Memory Data                                                     */
  _RW_ u_int_32  FMC;                        /*!< Flash Memory Control                                                  */
  _RW_ u_int_32  FCRIS;                      /*!< Flash Controller Raw Interrupt Status                                 */
  _RW_ u_int_32  FCIM;                       /*!< Flash Controller Interrupt Mask                                       */
  _RW_ u_int_32  FCMISC;                     /*!< Flash Controller Masked Interrupt Status and Clear                    */
  _RO_  u_int_32  RESERVED0[2];
  _RW_ u_int_32  FMC2;                       /*!< Flash Memory Control 2                                                */
  _RO_  u_int_32  RESERVED1[3];
  _RW_ u_int_32  FWBVAL;                     /*!< Flash Write Buffer Valid                                              */
  _RO_  u_int_32  RESERVED2[51];
  _RW_ u_int_32  FWBN;                       /*!< Flash Write Buffer n                                                  */
  _RO_  u_int_32  RESERVED3[943];
  _RW_ u_int_32  FSIZE;                      /*!< Flash Size                                                            */
  _RW_ u_int_32  SSIZE;                      /*!< SRAM Size                                                             */
  _RO_  u_int_32  RESERVED4;

  union {
    _RW_ u_int_32  ROMSWMAP_FLASH_CTRL_ALT;  /*!< ROM Software Map                                                      */
    _RW_ u_int_32  ROMSWMAP;                 /*!< ROM Software Map                                                      */
  };
  _RO_  u_int_32  RESERVED5[72];
  _RW_ u_int_32  RMCTL;                      /*!< ROM Control                                                           */
  _RO_  u_int_32  RESERVED6[55];
  _RW_ u_int_32  BOOTCFG;                    /*!< Boot Configuration                                                    */
  _RO_  u_int_32  RESERVED7[3];
  _RW_ u_int_32  USERREG0;                   /*!< User Register 0                                                       */
  _RW_ u_int_32  USERREG1;                   /*!< User Register 1                                                       */
  _RW_ u_int_32  USERREG2;                   /*!< User Register 2                                                       */
  _RW_ u_int_32  USERREG3;                   /*!< User Register 3                                                       */
  _RO_  u_int_32  RESERVED8[4];
  _RW_ u_int_32  FMPRE0;                     /*!< Flash Memory Protection Read Enable 0                                 */
  _RW_ u_int_32  FMPRE1;                     /*!< Flash Memory Protection Read Enable 1                                 */
  _RW_ u_int_32  FMPRE2;                     /*!< Flash Memory Protection Read Enable 2                                 */
  _RW_ u_int_32  FMPRE3;                     /*!< Flash Memory Protection Read Enable 3                                 */
  _RO_  u_int_32  RESERVED9[124];
  _RW_ u_int_32  FMPPE0;                     /*!< Flash Memory Protection Program Enable 0                              */
  _RW_ u_int_32  FMPPE1;                     /*!< Flash Memory Protection Program Enable 1                              */
  _RW_ u_int_32  FMPPE2;                     /*!< Flash Memory Protection Program Enable 2                              */
  _RW_ u_int_32  FMPPE3;                     /*!< Flash Memory Protection Program Enable 3                              */
} FLASH_CTRL_t;


/* ================================================================================ */
/* ================                     SYSCTL                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for SYSCTRL peripheral (SYSCTRL)
  */

typedef struct
{                                    /*!< SYSCTL Structure                                                   */
  _RW_ u_int_32  DID0;               /*!< Device Identification 0                                            */
  _RW_ u_int_32  DID1;               /*!< Device Identification 1                                            */
  _RW_ u_int_32  DC0;                /*!< Device Capabilities 0                                              */
  _RO_ u_int_32  RESERVED0;
  _RW_ u_int_32  DC1;                /*!< Device Capabilities 1                                              */
  _RW_ u_int_32  DC2;                /*!< Device Capabilities 2                                              */
  _RW_ u_int_32  DC3;                /*!< Device Capabilities 3                                              */
  _RW_ u_int_32  DC4;                /*!< Device Capabilities 4                                              */
  _RW_ u_int_32  DC5;                /*!< Device Capabilities 5                                              */
  _RW_ u_int_32  DC6;                /*!< Device Capabilities 6                                              */
  _RW_ u_int_32  DC7;                /*!< Device Capabilities 7                                              */
  _RW_ u_int_32  DC8;                /*!< Device Capabilities 8                                              */
  _RW_ u_int_32  PBORCTL;            /*!< Brown-Out Reset Control                                            */
  _RO_ u_int_32  RESERVED1[3];
  _RW_ u_int_32  SRCR0;              /*!< Software Reset Control 0                                           */
  _RW_ u_int_32  SRCR1;              /*!< Software Reset Control 1                                           */
  _RW_ u_int_32  SRCR2;              /*!< Software Reset Control 2                                           */
  _RO_ u_int_32  RESERVED2;
  _RW_ u_int_32  RIS;                /*!< Raw Interrupt Status                                               */
  _RW_ u_int_32  IMC;                /*!< Interrupt Mask Control                                             */
  _RW_ u_int_32  MISC;               /*!< Masked Interrupt Status and Clear                                  */
  _RW_ u_int_32  RESC;               /*!< Reset Cause                                                        */
  _RW_ u_int_32  RCC;                /*!< Run-Mode Clock Configuration                                       */
  _RO_ u_int_32  RESERVED3[2];
  _RW_ u_int_32  GPIOHBCTL;          /*!< GPIO High-Performance Bus Control                                  */
  _RW_ u_int_32  RCC2;               /*!< Run-Mode Clock Configuration 2                                     */
  _RO_ u_int_32  RESERVED4[2];
  _RW_ u_int_32  MOSCCTL;            /*!< Main Oscillator Control                                            */
  _RO_ u_int_32  RESERVED5[32];
  _RW_ u_int_32  RCGC0;              /*!< Run Mode Clock Gating Control Register 0                           */
  _RW_ u_int_32  RCGC1;              /*!< Run Mode Clock Gating Control Register 1                           */
  _RW_ u_int_32  RCGC2;              /*!< Run Mode Clock Gating Control Register 2                           */
  _RO_ u_int_32  RESERVED6;
  _RW_ u_int_32  SCGC0;              /*!< Sleep Mode Clock Gating Control Register 0                         */
  _RW_ u_int_32  SCGC1;              /*!< Sleep Mode Clock Gating Control Register 1                         */
  _RW_ u_int_32  SCGC2;              /*!< Sleep Mode Clock Gating Control Register 2                         */
  _RO_ u_int_32  RESERVED7;
  _RW_ u_int_32  DCGC0;              /*!< Deep Sleep Mode Clock Gating Control Register 0                    */
  _RW_ u_int_32  DCGC1;              /*!< Deep-Sleep Mode Clock Gating Control Register 1                    */
  _RW_ u_int_32  DCGC2;              /*!< Deep Sleep Mode Clock Gating Control Register 2                    */
  _RO_ u_int_32  RESERVED8[6];
  _RW_ u_int_32  DSLPCLKCFG;         /*!< Deep Sleep Clock Configuration                                     */
  _RO_ u_int_32  RESERVED9;
  _RW_ u_int_32  SYSPROP;            /*!< System Properties                                                  */
  _RW_ u_int_32  PIOSCCAL;           /*!< Precision Internal Oscillator Calibration                          */
  _RW_ u_int_32  PIOSCSTAT;          /*!< Precision Internal Oscillator Statistics                           */
  _RO_ u_int_32  RESERVED10[2];
  _RW_ u_int_32  PLLFREQ0;           /*!< PLL Frequency 0                                                    */
  _RW_ u_int_32  PLLFREQ1;           /*!< PLL Frequency 1                                                    */
  _RW_ u_int_32  PLLSTAT;            /*!< PLL Status                                                         */
  _RO_ u_int_32  RESERVED11[7];
  _RW_ u_int_32  SLPPWRCFG;          /*!< Sleep Power Configuration                                          */
  _RW_ u_int_32  DSLPPWRCFG;         /*!< Deep-Sleep Power Configuration                                     */
  _RW_ u_int_32  DC9;                /*!< Device Capabilities 9                                              */
  _RO_ u_int_32  RESERVED12[3];
  _RW_ u_int_32  NVMSTAT;            /*!< Non-Volatile Memory Information                                    */
  _RO_ u_int_32  RESERVED13[4];
  _RW_ u_int_32  LDOSPCTL;           /*!< LDO Sleep Power Control                                            */
  _RO_ u_int_32  RESERVED14;
  _RW_ u_int_32  LDODPCTL;           /*!< LDO Deep-Sleep Power Control                                       */
  _RO_ u_int_32  RESERVED15[80];
  _RW_ u_int_32  PPWD;               /*!< Watchdog Timer Peripheral Present                                  */
  _RW_ u_int_32  PPTIMER;            /*!< 16/32-Bit General-Purpose Timer Peripheral Present                 */
  _RW_ u_int_32  PPGPIO;             /*!< General-Purpose Input/Output Peripheral Present                    */
  _RW_ u_int_32  PPDMA;              /*!< Micro Direct Memory Access Peripheral Present                      */
  _RO_ u_int_32  RESERVED16;
  _RW_ u_int_32  PPHIB;              /*!< Hibernation Peripheral Present                                     */
  _RW_ u_int_32  PPUART;             /*!< Universal Asynchronous Receiver/Transmitter Peripheral Present     */
  _RW_ u_int_32  PPSSI;              /*!< Synchronous Serial Interface Peripheral Present                    */
  _RW_ u_int_32  PPI2C;              /*!< Inter-Integrated Circuit Peripheral Present                        */
  _RO_ u_int_32  RESERVED17;
  _RW_ u_int_32  PPUSB;              /*!< Universal Serial Bus Peripheral Present                            */
  _RO_ u_int_32  RESERVED18[2];
  _RW_ u_int_32  PPCAN;              /*!< Controller Area Network Peripheral Present                         */
  _RW_ u_int_32  PPADC;              /*!< Analog-to-Digital Converter Peripheral Present                     */
  _RW_ u_int_32  PPACMP;             /*!< Analog Comparator Peripheral Present                               */
  _RW_ u_int_32  PPPWM;              /*!< Pulse Width Modulator Peripheral Present                           */
  _RW_ u_int_32  PPQEI;              /*!< Quadrature Encoder Interface Peripheral Present                    */
  _RO_ u_int_32  RESERVED19[4];
  _RW_ u_int_32  PPEEPROM;           /*!< EEPROM Peripheral Present                                          */
  _RW_ u_int_32  PPWTIMER;           /*!< 32/64-Bit Wide General-Purpose Timer Peripheral Present            */
  _RO_ u_int_32  RESERVED20[104];
  _RW_ u_int_32  SRWD;               /*!< Watchdog Timer Software Reset                                      */
  _RW_ u_int_32  SRTIMER;            /*!< 16/32-Bit General-Purpose Timer Software Reset                     */
  _RW_ u_int_32  SRGPIO;             /*!< General-Purpose Input/Output Software Reset                        */
  _RW_ u_int_32  SRDMA;              /*!< Micro Direct Memory Access Software Reset                          */
  _RO_ u_int_32  RESERVED21;
  _RW_ u_int_32  SRHIB;              /*!< Hibernation Software Reset                                         */
  _RW_ u_int_32  SRUART;             /*!< Universal Asynchronous Receiver/Transmitter Software Reset         */
  _RW_ u_int_32  SRSSI;              /*!< Synchronous Serial Interface Software Reset                        */
  _RW_ u_int_32  SRI2C;              /*!< Inter-Integrated Circuit Software Reset                            */
  _RO_ u_int_32  RESERVED22;
  _RW_ u_int_32  SRUSB;              /*!< Universal Serial Bus Software Reset                                */
  _RO_ u_int_32  RESERVED23[2];
  _RW_ u_int_32  SRCAN;              /*!< Controller Area Network Software Reset                             */
  _RW_ u_int_32  SRADC;              /*!< Analog-to-Digital Converter Software Reset                         */
  _RW_ u_int_32  SRACMP;             /*!< Analog Comparator Software Reset                                   */
  _RW_ u_int_32  SRPWM;              /*!< Pulse Width Modulator Software Reset                               */
  _RW_ u_int_32  SRQEI;              /*!< Quadrature Encoder Interface Software Reset                        */
  _RO_ u_int_32  RESERVED24[4];
  _RW_ u_int_32  SREEPROM;           /*!< EEPROM Software Reset                                              */
  _RW_ u_int_32  SRWTIMER;           /*!< 32/64-Bit Wide General-Purpose Timer Software Reset                */
  _RO_ u_int_32  RESERVED25[40];
  _RW_ u_int_32  RCGCWD;             /*!< Watchdog Timer Run Mode Clock Gating Control                       */
  _RW_ u_int_32  RCGCTIMER;          /*!< 16/32-Bit General-Purpose Timer Run Mode Clock Gating Control      */
  _RW_ u_int_32  RCGCGPIO;           /*!< General-Purpose Input/Output Run Mode Clock Gating Control         */
  _RW_ u_int_32  RCGCDMA;            /*!< Micro Direct Memory Access Run Mode Clock Gating Control           */
  _RO_ u_int_32  RESERVED26;
  _RW_ u_int_32  RCGCHIB;            /*!< Hibernation Run Mode Clock Gating Control                          */
  _RW_ u_int_32  RCGCUART;           /*!< Universal Asynchronous Receiver/Transmitter Run Mode Clock Gating
                                          Control                                                            */
  _RW_ u_int_32  RCGCSSI;            /*!< Synchronous Serial Interface Run Mode Clock Gating Control         */
  _RW_ u_int_32  RCGCI2C;            /*!< Inter-Integrated Circuit Run Mode Clock Gating Control             */
  _RO_ u_int_32  RESERVED27;
  _RW_ u_int_32  RCGCUSB;            /*!< Universal Serial Bus Run Mode Clock Gating Control                 */
  _RO_ u_int_32  RESERVED28[2];
  _RW_ u_int_32  RCGCCAN;            /*!< Controller Area Network Run Mode Clock Gating Control              */
  _RW_ u_int_32  RCGCADC;            /*!< Analog-to-Digital Converter Run Mode Clock Gating Control          */
  _RW_ u_int_32  RCGCACMP;           /*!< Analog Comparator Run Mode Clock Gating Control                    */
  _RW_ u_int_32  RCGCPWM;            /*!< Pulse Width Modulator Run Mode Clock Gating Control                */
  _RW_ u_int_32  RCGCQEI;            /*!< Quadrature Encoder Interface Run Mode Clock Gating Control         */
  _RO_ u_int_32  RESERVED29[4];
  _RW_ u_int_32  RCGCEEPROM;         /*!< EEPROM Run Mode Clock Gating Control                               */
  _RW_ u_int_32  RCGCWTIMER;         /*!< 32/64-Bit Wide General-Purpose Timer Run Mode Clock Gating Control */
  _RO_ u_int_32  RESERVED30[40];
  _RW_ u_int_32  SCGCWD;             /*!< Watchdog Timer Sleep Mode Clock Gating Control                     */
  _RW_ u_int_32  SCGCTIMER;          /*!< 16/32-Bit General-Purpose Timer Sleep Mode Clock Gating Control    */
  _RW_ u_int_32  SCGCGPIO;           /*!< General-Purpose Input/Output Sleep Mode Clock Gating Control       */
  _RW_ u_int_32  SCGCDMA;            /*!< Micro Direct Memory Access Sleep Mode Clock Gating Control         */
  _RO_ u_int_32  RESERVED31;
  _RW_ u_int_32  SCGCHIB;            /*!< Hibernation Sleep Mode Clock Gating Control                        */
  _RW_ u_int_32  SCGCUART;           /*!< Universal Asynchronous Receiver/Transmitter Sleep Mode Clock
                                          Gating Control                                                     */
  _RW_ u_int_32  SCGCSSI;            /*!< Synchronous Serial Interface Sleep Mode Clock Gating Control       */
  _RW_ u_int_32  SCGCI2C;            /*!< Inter-Integrated Circuit Sleep Mode Clock Gating Control           */
  _RO_ u_int_32  RESERVED32;
  _RW_ u_int_32  SCGCUSB;            /*!< Universal Serial Bus Sleep Mode Clock Gating Control               */
  _RO_ u_int_32  RESERVED33[2];
  _RW_ u_int_32  SCGCCAN;            /*!< Controller Area Network Sleep Mode Clock Gating Control            */
  _RW_ u_int_32  SCGCADC;            /*!< Analog-to-Digital Converter Sleep Mode Clock Gating Control        */
  _RW_ u_int_32  SCGCACMP;           /*!< Analog Comparator Sleep Mode Clock Gating Control                  */
  _RW_ u_int_32  SCGCPWM;            /*!< Pulse Width Modulator Sleep Mode Clock Gating Control              */
  _RW_ u_int_32  SCGCQEI;            /*!< Quadrature Encoder Interface Sleep Mode Clock Gating Control       */
  _RO_ u_int_32  RESERVED34[4];
  _RW_ u_int_32  SCGCEEPROM;         /*!< EEPROM Sleep Mode Clock Gating Control                             */
  _RW_ u_int_32  SCGCWTIMER;         /*!< 32/64-Bit Wide General-Purpose Timer Sleep Mode Clock Gating
                                          Control                                                            */
  _RO_ u_int_32  RESERVED35[40];
  _RW_ u_int_32  DCGCWD;             /*!< Watchdog Timer Deep-Sleep Mode Clock Gating Control                */
  _RW_ u_int_32  DCGCTIMER;          /*!< 16/32-Bit General-Purpose Timer Deep-Sleep Mode Clock Gating
                                          Control                                                            */
  _RW_ u_int_32  DCGCGPIO;           /*!< General-Purpose Input/Output Deep-Sleep Mode Clock Gating Control  */
  _RW_ u_int_32  DCGCDMA;            /*!< Micro Direct Memory Access Deep-Sleep Mode Clock Gating Control    */
  _RO_ u_int_32  RESERVED36;
  _RW_ u_int_32  DCGCHIB;            /*!< Hibernation Deep-Sleep Mode Clock Gating Control                   */
  _RW_ u_int_32  DCGCUART;           /*!< Universal Asynchronous Receiver/Transmitter Deep-Sleep Mode
                                          Clock Gating Control                                               */
  _RW_ u_int_32  DCGCSSI;            /*!< Synchronous Serial Interface Deep-Sleep Mode Clock Gating Control  */
  _RW_ u_int_32  DCGCI2C;            /*!< Inter-Integrated Circuit Deep-Sleep Mode Clock Gating Control      */
  _RO_ u_int_32  RESERVED37;
  _RW_ u_int_32  DCGCUSB;            /*!< Universal Serial Bus Deep-Sleep Mode Clock Gating Control          */
  _RO_ u_int_32  RESERVED38[2];
  _RW_ u_int_32  DCGCCAN;            /*!< Controller Area Network Deep-Sleep Mode Clock Gating Control       */
  _RW_ u_int_32  DCGCADC;            /*!< Analog-to-Digital Converter Deep-Sleep Mode Clock Gating Control   */
  _RW_ u_int_32  DCGCACMP;           /*!< Analog Comparator Deep-Sleep Mode Clock Gating Control             */
  _RW_ u_int_32  DCGCPWM;            /*!< Pulse Width Modulator Deep-Sleep Mode Clock Gating Control         */
  _RW_ u_int_32  DCGCQEI;            /*!< Quadrature Encoder Interface Deep-Sleep Mode Clock Gating Control  */
  _RO_ u_int_32  RESERVED39[4];
  _RW_ u_int_32  DCGCEEPROM;         /*!< EEPROM Deep-Sleep Mode Clock Gating Control                        */
  _RW_ u_int_32  DCGCWTIMER;         /*!< 32/64-Bit Wide General-Purpose Timer Deep-Sleep Mode Clock Gating
                                          Control                                                            */
  _RO_ u_int_32  RESERVED40[104];
  _RW_ u_int_32  PRWD;               /*!< Watchdog Timer Peripheral Ready                                    */
  _RW_ u_int_32  PRTIMER;            /*!< 16/32-Bit General-Purpose Timer Peripheral Ready                   */
  _RW_ u_int_32  PRGPIO;             /*!< General-Purpose Input/Output Peripheral Ready                      */
  _RW_ u_int_32  PRDMA;              /*!< Micro Direct Memory Access Peripheral Ready                        */
  _RO_ u_int_32  RESERVED41;
  _RW_ u_int_32  PRHIB;              /*!< Hibernation Peripheral Ready                                       */
  _RW_ u_int_32  PRUART;             /*!< Universal Asynchronous Receiver/Transmitter Peripheral Ready       */
  _RW_ u_int_32  PRSSI;              /*!< Synchronous Serial Interface Peripheral Ready                      */
  _RW_ u_int_32  PRI2C;              /*!< Inter-Integrated Circuit Peripheral Ready                          */
  _RO_ u_int_32  RESERVED42;
  _RW_ u_int_32  PRUSB;              /*!< Universal Serial Bus Peripheral Ready                              */
  _RO_ u_int_32  RESERVED43[2];
  _RW_ u_int_32  PRCAN;              /*!< Controller Area Network Peripheral Ready                           */
  _RW_ u_int_32  PRADC;              /*!< Analog-to-Digital Converter Peripheral Ready                       */
  _RW_ u_int_32  PRACMP;             /*!< Analog Comparator Peripheral Ready                                 */
  _RW_ u_int_32  PRPWM;              /*!< Pulse Width Modulator Peripheral Ready                             */
  _RW_ u_int_32  PRQEI;              /*!< Quadrature Encoder Interface Peripheral Ready                      */
  _RO_ u_int_32  RESERVED44[4];
  _RW_ u_int_32  PREEPROM;           /*!< EEPROM Peripheral Ready                                            */
  _RW_ u_int_32  PRWTIMER;           /*!< 32/64-Bit Wide General-Purpose Timer Peripheral Ready              */
} SYS_CTRL_t;


/* ================================================================================ */
/* ================                      UDMA                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for UDMA peripheral (UDMA)
  */

typedef struct
{                                        /*!< UDMA Structure                                    */
  _RW_ u_int_32  STAT;                   /*!< DMA Status                                        */
  _WO_ u_int_32  CFG;                    /*!< DMA Configuration                                 */
  _RW_ u_int_32  CTLBASE;                /*!< DMA Channel Control Base Pointer                  */
  _RW_ u_int_32  ALTBASE;                /*!< DMA Alternate Channel Control Base Pointer        */
  _RW_ u_int_32  WAITSTAT;               /*!< DMA Channel Wait-on-Request Status                */
  _WO_ u_int_32  SWREQ;                  /*!< DMA Channel Software Request                      */
  _RW_ u_int_32  USEBURSTSET;            /*!< DMA Channel Useburst Set                          */
  _WO_ u_int_32  USEBURSTCLR;            /*!< DMA Channel Useburst Clear                        */
  _RW_ u_int_32  REQMASKSET;             /*!< DMA Channel Request Mask Set                      */
  _WO_ u_int_32  REQMASKCLR;             /*!< DMA Channel Request Mask Clear                    */
  _RW_ u_int_32  ENASET;                 /*!< DMA Channel Enable Set                            */
  _WO_ u_int_32  ENACLR;                 /*!< DMA Channel Enable Clear                          */
  _RW_ u_int_32  ALTSET;                 /*!< DMA Channel Primary Alternate Set                 */
  _WO_  u_int_32  ALTCLR;                 /*!< DMA Channel Primary Alternate Clear               */
  _RW_ u_int_32  PRIOSET;                /*!< DMA Channel Priority Set                          */
  _WO_  u_int_32  PRIOCLR;                /*!< DMA Channel Priority Clear                        */
  _RO_  u_int_32  RESERVED0[3];
  _RW_ u_int_32  ERRCLR;                 /*!< DMA Bus Error Clear                               */
  _RO_  u_int_32  RESERVED1[300];
  _RW_ u_int_32  CHASGN;                 /*!< DMA Channel Assignment                            */
  _RW_ u_int_32  CHIS;                   /*!< DMA Channel Interrupt Status                      */
  _RO_  u_int_32  RESERVED2[2];
  _RW_ u_int_32  CHMAP0;                 /*!< DMA Channel Map Select 0                          */
  _RW_ u_int_32  CHMAP1;                 /*!< DMA Channel Map Select 1                          */
  _RW_ u_int_32  CHMAP2;                 /*!< DMA Channel Map Select 2                          */
  _RW_ u_int_32  CHMAP3;                 /*!< DMA Channel Map Select 3                          */
} UDMA_t;


/* --------------------  End of section using anonymous unions  ------------------- */


/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define WATCHDOG0_BASE                  0x40000000UL
#define WATCHDOG1_BASE                  0x40001000UL
#define GPIOA_BASE                      0x40004000UL
#define GPIOB_BASE                      0x40005000UL
#define GPIOC_BASE                      0x40006000UL
#define GPIOD_BASE                      0x40007000UL
#define SSI0_BASE                       0x40008000UL
#define SSI1_BASE                       0x40009000UL
#define SSI2_BASE                       0x4000A000UL
#define SSI3_BASE                       0x4000B000UL
#define UART0_BASE                      0x4000C000UL
#define UART1_BASE                      0x4000D000UL
#define UART2_BASE                      0x4000E000UL
#define UART3_BASE                      0x4000F000UL
#define UART4_BASE                      0x40010000UL
#define UART5_BASE                      0x40011000UL
#define UART6_BASE                      0x40012000UL
#define UART7_BASE                      0x40013000UL
#define I2C0_BASE                       0x40020000UL
#define I2C1_BASE                       0x40021000UL
#define I2C2_BASE                       0x40022000UL
#define I2C3_BASE                       0x40023000UL
#define GPIOE_BASE                      0x40024000UL
#define GPIOF_BASE                      0x40025000UL
#define PWM0_BASE                       0x40028000UL
#define PWM1_BASE                       0x40029000UL
#define QEI0_BASE                       0x4002C000UL
#define QEI1_BASE                       0x4002D000UL
#define TIMER0_BASE                     0x40030000UL
#define TIMER1_BASE                     0x40031000UL
#define TIMER2_BASE                     0x40032000UL
#define TIMER3_BASE                     0x40033000UL
#define TIMER4_BASE                     0x40034000UL
#define TIMER5_BASE                     0x40035000UL
#define WTIMER0_BASE                    0x40036000UL
#define WTIMER1_BASE                    0x40037000UL
#define ADC0_BASE                       0x40038000UL
#define ADC1_BASE                       0x40039000UL
#define AN_COMP_BASE                    0x4003C000UL
#define CAN0_BASE                       0x40040000UL
#define CAN1_BASE                       0x40041000UL
#define WTIMER2_BASE                    0x4004C000UL
#define WTIMER3_BASE                    0x4004D000UL
#define WTIMER4_BASE                    0x4004E000UL
#define WTIMER5_BASE                    0x4004F000UL
#define USB0_BASE                       0x40050000UL
#define GPIOA_AHB_BASE                  0x40058000UL
#define GPIOB_AHB_BASE                  0x40059000UL
#define GPIOC_AHB_BASE                  0x4005A000UL
#define GPIOD_AHB_BASE                  0x4005B000UL
#define GPIOE_AHB_BASE                  0x4005C000UL
#define GPIOF_AHB_BASE                  0x4005D000UL
#define EEPROM_BASE                     0x400AF000UL
#define SYSEXC_BASE                     0x400F9000UL
#define HIB_BASE                        0x400FC000UL
#define FLASH_CTRL_BASE                 0x400FD000UL
#define SYS_CTRL_BASE                   0x400FE000UL
#define UDMA_BASE                       0x400FF000UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define WATCHDOG0            ((WATCHDOG0_t            *) WATCHDOG0_BASE)
#define WATCHDOG1            ((WATCHDOG0_t            *) WATCHDOG1_BASE)
#define GPIOA                ((GPIO_RegDef_t          *) GPIOA_BASE)
#define GPIOB                ((GPIO_RegDef_t          *) GPIOB_BASE)
#define GPIOC                ((GPIO_RegDef_t          *) GPIOC_BASE)
#define GPIOD                ((GPIO_RegDef_t          *) GPIOD_BASE)
#define SSI0                 ((SSI_RegDef_t           *) SSI0_BASE)
#define SSI1                 ((SSI_RegDef_t           *) SSI1_BASE)
#define SSI2                 ((SSI_RegDef_t           *) SSI2_BASE)
#define SSI3                 ((SSI_RegDef_t           *) SSI3_BASE)
#define UART0                ((UART0_t                *) UART0_BASE)
#define UART1                ((UART0_t                *) UART1_BASE)
#define UART2                ((UART0_t                *) UART2_BASE)
#define UART3                ((UART0_t                *) UART3_BASE)
#define UART4                ((UART0_t                *) UART4_BASE)
#define UART5                ((UART0_t                *) UART5_BASE)
#define UART6                ((UART0_t                *) UART6_BASE)
#define UART7                ((UART0_t                *) UART7_BASE)
#define I2C0                 ((I2C0_t                 *) I2C0_BASE)
#define I2C1                 ((I2C0_t                 *) I2C1_BASE)
#define I2C2                 ((I2C0_t                 *) I2C2_BASE)
#define I2C3                 ((I2C0_t                 *) I2C3_BASE)
#define GPIOE                ((GPIO_RegDef_t          *) GPIOE_BASE)
#define GPIOF                ((GPIO_RegDef_t          *) GPIOF_BASE)
#define PWM0                 ((PWM0_t                 *) PWM0_BASE)
#define PWM1                 ((PWM0_t                 *) PWM1_BASE)
#define QEI0                 ((QEI0_t                 *) QEI0_BASE)
#define QEI1                 ((QEI0_t                 *) QEI1_BASE)
#define TIMER0               ((Timer_RegDef_t         *) TIMER0_BASE)
#define TIMER1               ((Timer_RegDef_t         *) TIMER1_BASE)
#define TIMER2               ((Timer_RegDef_t         *) TIMER2_BASE)
#define TIMER3               ((Timer_RegDef_t         *) TIMER3_BASE)
#define TIMER4               ((Timer_RegDef_t         *) TIMER4_BASE)
#define TIMER5               ((Timer_RegDef_t         *) TIMER5_BASE)
#define WTIMER0              ((W_Timer_RegDef_t       *) WTIMER0_BASE)
#define WTIMER1              ((W_Timer_RegDef_t       *) WTIMER1_BASE)
#define ADC0                 ((ADC_RegDef_t           *) ADC0_BASE)
#define ADC1                 ((ADC_RegDef_t           *) ADC1_BASE)
#define AN_COMP              ((AN_COMP_t              *) AN_COMP_BASE)
#define CAN0                 ((CAN_RegDef_t           *) CAN0_BASE)
#define CAN1                 ((CAN_RegDef_t           *) CAN1_BASE)
#define WTIMER2              ((W_Timer_RegDef_t       *) WTIMER2_BASE)
#define WTIMER3              ((W_Timer_RegDef_t       *) WTIMER3_BASE)
#define WTIMER4              ((W_Timer_RegDef_t       *) WTIMER4_BASE)
#define WTIMER5              ((W_Timer_RegDef_t       *) WTIMER5_BASE)
#define USB0                 ((USB0_t                 *) USB0_BASE)
#define GPIOA_AHB            ((GPIO_RegDef_t          *) GPIOA_AHB_BASE)
#define GPIOB_AHB            ((GPIO_RegDef_t          *) GPIOB_AHB_BASE)
#define GPIOC_AHB            ((GPIO_RegDef_t          *) GPIOC_AHB_BASE)
#define GPIOD_AHB            ((GPIO_RegDef_t          *) GPIOD_AHB_BASE)
#define GPIOE_AHB            ((GPIO_RegDef_t          *) GPIOE_AHB_BASE)
#define GPIOF_AHB            ((GPIO_RegDef_t          *) GPIOF_AHB_BASE)
#define EEPROM               ((EEPROM_t               *) EEPROM_BASE)
#define SYSEXC               ((SYSEXC_t               *) SYSEXC_BASE)
#define HIB                  ((HIB_t                  *) HIB_BASE)
#define FLASH_CTRL           ((FLASH_CTRL_t           *) FLASH_CTRL_BASE)
#define SYS_CTRL             ((SYS_CTRL_t             *) SYS_CTRL_BASE)
#define UDMA                 ((UDMA_t                 *) UDMA_BASE)



/* ================================================================================ */
/* ================              CLOCK EN&DIS MACROS               ================ */
/* ================================================================================ */

/*---------------------------------  RUN MODE --------------------------------------*/

// CLOCK ENABLE MACROS FOR WATCH DOG TIMER MODULES
#define WATCHDOG0_PCLK_EN()     ( SYS_CTRL->RCGCWD |= ( 1<<0 ) )
#define WATCHDOG1_PCLK_EN()     ( SYS_CTRL->RCGCWD |= ( 1<<1 ) )

// CLOCK DISABLE MACROS FOR WATCH DOG TIMER MODULES
#define WATCHDOG0_PCLK_DIS()    ( SYS_CTRL->RCGCWD &= ~( 1<<0 ) )
#define WATCHDOG1_PCLK_DIS()    ( SYS_CTRL->RCGCWD &= ~( 1<<1 ) )

// CLOCK ENABLE MACROS FOR TIMERS (16/32 bit Timers)
#define TIMER0_PCLK_EN()        ( SYS_CTRL->RCGCTIMER |= ( 1<<0 ) )
#define TIMER1_PCLK_EN()        ( SYS_CTRL->RCGCTIMER |= ( 1<<1 ) )
#define TIMER2_PCLK_EN()        ( SYS_CTRL->RCGCTIMER |= ( 1<<2 ) )
#define TIMER3_PCLK_EN()        ( SYS_CTRL->RCGCTIMER |= ( 1<<3 ) )
#define TIMER4_PCLK_EN()        ( SYS_CTRL->RCGCTIMER |= ( 1<<4 ) )
#define TIMER5_PCLK_EN()        ( SYS_CTRL->RCGCTIMER |= ( 1<<5 ) )

// CLOCK DISABLE MACROS FOR TIMERS
#define TIMER0_PCLK_DIS()       ( SYS_CTRL->RCGCTIMER &= ~( 1<<0 ) )
#define TIMER1_PCLK_DIS()       ( SYS_CTRL->RCGCTIMER &= ~( 1<<1 ) )
#define TIMER2_PCLK_DIS()       ( SYS_CTRL->RCGCTIMER &= ~( 1<<2 ) )
#define TIMER3_PCLK_DIS()       ( SYS_CTRL->RCGCTIMER &= ~( 1<<3 ) )
#define TIMER4_PCLK_DIS()       ( SYS_CTRL->RCGCTIMER &= ~( 1<<4 ) )
#define TIMER5_PCLK_DIS()       ( SYS_CTRL->RCGCTIMER &= ~( 1<<5 ) )



// CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS
#define GPIOA_PCLK_EN()         ( SYS_CTRL->RCGCGPIO |= ( 1<<0 ) )
#define GPIOB_PCLK_EN()         ( SYS_CTRL->RCGCGPIO |= ( 1<<1 ) )
#define GPIOC_PCLK_EN()         ( SYS_CTRL->RCGCGPIO |= ( 1<<2 ) )
#define GPIOD_PCLK_EN()         ( SYS_CTRL->RCGCGPIO |= ( 1<<3 ) )
#define GPIOE_PCLK_EN()         ( SYS_CTRL->RCGCGPIO |= ( 1<<4 ) )
#define GPIOF_PCLK_EN()         ( SYS_CTRL->RCGCGPIO |= ( 1<<5 ) )

// CLOCK DISABLE MACROS FOR GPIOx PERIPHERALS
#define GPIOA_PCLK_DIS()        ( SYS_CTRL->RCGCGPIO &= ~( 1<<0 ) )
#define GPIOB_PCLK_DIS()        ( SYS_CTRL->RCGCGPIO &= ~( 1<<1 ) )
#define GPIOC_PCLK_DIS()        ( SYS_CTRL->RCGCGPIO &= ~( 1<<2 ) )
#define GPIOD_PCLK_DIS()        ( SYS_CTRL->RCGCGPIO &= ~( 1<<3 ) )
#define GPIOE_PCLK_DIS()        ( SYS_CTRL->RCGCGPIO &= ~( 1<<4 ) )
#define GPIOF_PCLK_DIS()        ( SYS_CTRL->RCGCGPIO &= ~( 1<<5 ) )

// CLOCK ENABLE MACROS FOR MICRO DIRECT MEMORY MODULE
#define UDMA_PCLK_EN()          ( SYS_CTRL->RCGCDMA |= ( 1<<0 ) )

// CLOCK DISABLE MACROS FOR MICRO DIRECT MEMORY MODULE
#define UDMA_PCLK_DIS()         ( SYS_CTRL->RCGCDMA &= ~( 1<<0 ) )

// CLOCK ENABLE MACROS FOR HIBERNATION MODULE
#define HIB_PCLK_EN()           ( SYS_CTRL->RCGCHIB |= ( 1<<0 ) )

// CLOCK DISABLE MACROS FOR HIBERNATION MODULE
#define HIB_PCLK_DIS()           ( SYS_CTRL->RCGCHIB &= ~( 1<<0 ) )

// CLOCK ENABLE MACROS FOR UART MODULES
#define UART0_PCLK_EN()         ( SYS_CTRL->RCGCUART |= ( 1<<0 ) )
#define UART1_PCLK_EN()         ( SYS_CTRL->RCGCUART |= ( 1<<1 ) )
#define UART2_PCLK_EN()         ( SYS_CTRL->RCGCUART |= ( 1<<2 ) )
#define UART3_PCLK_EN()         ( SYS_CTRL->RCGCUART |= ( 1<<3 ) )
#define UART4_PCLK_EN()         ( SYS_CTRL->RCGCUART |= ( 1<<4 ) )
#define UART5_PCLK_EN()         ( SYS_CTRL->RCGCUART |= ( 1<<5 ) )
#define UART6_PCLK_EN()         ( SYS_CTRL->RCGCUART |= ( 1<<6 ) )

// CLOCK DISABLE MACROS FOR UART MODULES
#define UART0_PCLK_DIS()         ( SYS_CTRL->RCGCUART &= ~( 1<<0 ) )
#define UART1_PCLK_DIS()         ( SYS_CTRL->RCGCUART &= ~( 1<<1 ) )
#define UART2_PCLK_DIS()         ( SYS_CTRL->RCGCUART &= ~( 1<<2 ) )
#define UART3_PCLK_DIS()         ( SYS_CTRL->RCGCUART &= ~( 1<<3 ) )
#define UART4_PCLK_DIS()         ( SYS_CTRL->RCGCUART &= ~( 1<<4 ) )
#define UART5_PCLK_DIS()         ( SYS_CTRL->RCGCUART &= ~( 1<<5 ) )
#define UART6_PCLK_DIS()         ( SYS_CTRL->RCGCUART &= ~( 1<<6 ) )


// CLOCK ENABLE MACROS FOR SYNCHRONOUS SERIAL INTERFACE MODULES
#define SSI0_PCLK_EN()         ( SYS_CTRL->RCGCSSI |= ( 1<<0 ) )
#define SSI1_PCLK_EN()         ( SYS_CTRL->RCGCSSI |= ( 1<<1 ) )
#define SSI2_PCLK_EN()         ( SYS_CTRL->RCGCSSI |= ( 1<<2 ) )
#define SSI3_PCLK_EN()         ( SYS_CTRL->RCGCSSI |= ( 1<<3 ) )

// CLOCK DISABLE MACROS FOR SYNCHRONOUS SERIAL INTERFACE MODULES
#define SSI0_PCLK_DIS()         ( SYS_CTRL->RCGCSSI &= ~( 1<<0 ) )
#define SSI1_PCLK_DIS()         ( SYS_CTRL->RCGCSSI &= ~( 1<<1 ) )
#define SSI2_PCLK_DIS()         ( SYS_CTRL->RCGCSSI &= ~( 1<<2 ) )
#define SSI3_PCLK_DIS()         ( SYS_CTRL->RCGCSSI &= ~( 1<<3 ) )

// CLOCK ENABLE MACROS FOR INTER INTEGRATED CIRCUIT MODULES
#define I2C0_PCLK_EN()         ( SYS_CTRL->RCGCI2C |= ( 1<<0 ) )
#define I2C1_PCLK_EN()         ( SYS_CTRL->RCGCI2C |= ( 1<<1 ) )
#define I2C2_PCLK_EN()         ( SYS_CTRL->RCGCI2C |= ( 1<<2 ) )
#define I2C3_PCLK_EN()         ( SYS_CTRL->RCGCI2C |= ( 1<<3 ) )

// CLOCK DISABLE MACROS FOR INTER INTEGRATED CIRCUIT MODULES
#define I2C0_PCLK_DIS()         ( SYS_CTRL->RCGCI2C &= ~( 1<<0 ) )
#define I2C1_PCLK_DIS()         ( SYS_CTRL->RCGCI2C &= ~( 1<<1 ) )
#define I2C2_PCLK_DIS()         ( SYS_CTRL->RCGCI2C &= ~( 1<<2 ) )
#define I2C3_PCLK_DIS()         ( SYS_CTRL->RCGCI2C &= ~( 1<<3 ) )

// CLOCK ENABLE MACROS FOR UNIVERSAL SERIAL BUS MODULE
#define USB0_PCLK_EN()          ( SYS_CTRL->RCGCUSB |= ( 1<<0 ) )

// CLOCK DISABLE MACROS FOR UNIVERSAL SERIAL BUS MODULE
#define USB0_PCLK_DIS()         ( SYS_CTRL->RCGCUSB &= ~( 1<<0 ) )

// CLOCK ENABLE MACROS FOR CONTROLLER AREA NETWORK MODULES
#define CAN0_PCLK_EN()          ( SYS_CTRL->RCGCCAN |= ( 1<<0 ) )
#define CAN1_PCLK_EN()          ( SYS_CTRL->RCGCCAN |= ( 1<<1 ) )

// CLOCK DISABLE MACROS FOR CONTROLLER AREA NETWORK MODULES
#define CAN0_PCLK_DIS()         ( SYS_CTRL->RCGCCAN &= ~( 1<<0 ) )
#define CAN1_PCLK_DIS()         ( SYS_CTRL->RCGCCAN &= ~( 1<<1 ) )

// CLOCK ENABLE MACROS FOR ANALOG TO DIGITAL MODULES
#define ADC0_PCLK_EN()          ( SYS_CTRL->RCGCADC |= ( 1<<0 ) )
#define ADC1_PCLK_EN()          ( SYS_CTRL->RCGCADC |= ( 1<<1 ) )

// CLOCK DISABLE MACROS FOR ANALOG TO DIGITAL MODULES
#define ADC0_PCLK_DIS()          ( SYS_CTRL->RCGCADC &= ~( 1<<0 ) )
#define ADC1_PCLK_DIS()          ( SYS_CTRL->RCGCADC &= ~( 1<<1 ) )

// CLOCK ENABLE MACROS FOR ANALOG COMPARATOR MODULE
#define AN_COMP_PCLK_EN()          ( SYS_CTRL->RCGCACMP |= ( 1<<0 ) )

// CLOCK DISABLE MACROS FOR ANALOG COMPARATOR MODULE
#define AN_COMP_PCLK_DIS()          ( SYS_CTRL->RCGCACMP &= ~( 1<<0 ) )

// CLOCK ENABLE MACROS FOR PULSE WIDTH MODULATOR MODULES
#define PWM0_PCLK_EN()          ( SYS_CTRL->RCGCPWM |= ( 1<<0 ) )
#define PWM1_PCLK_EN()          ( SYS_CTRL->RCGCPWM |= ( 1<<1 ) )

// CLOCK DISABLE MACROS FOR PULSE WIDTH MODULATOR MODULES
#define PWM0_PCLK_DIS()          ( SYS_CTRL->RCGCPWM &= ~( 1<<0 ) )
#define PWM1_PCLK_DIS()          ( SYS_CTRL->RCGCPWM &= ~( 1<<1 ) )

// CLOCK ENABLE MACROS FOR QUADRATURE ENCODER INTERFACE
#define QEI0_PCLK_EN()          ( SYS_CTRL->RCGCQEI |= ( 1<<0 ) )
#define QEI1_PCLK_EN()          ( SYS_CTRL->RCGCQEI |= ( 1<<1 ) )

// CLOCK DISABLE MACROS FOR QUADRATURE ENCODER INTERFACE
#define QEI0_PCLK_DIS()          ( SYS_CTRL->RCGCQEI &= ~( 1<<0 ) )
#define QEI1_PCLK_DIS()          ( SYS_CTRL->RCGCQEI &= ~( 1<<1 ) )

// CLOCK ENABLE MACROS FOR EEPROM
#define EEPROM_COMP_PCLK_EN()          ( SYS_CTRL->RCGCEEPROM |= ( 1<<0 ) )

// CLOCK DISABLE MACROS FOR EEPROM
#define EEPROM_COMP_PCLK_DIS()          ( SYS_CTRL->RCGCEEPROM &= ~( 1<<0 ) )

// CLOCK ENABLE MACROS FOR WIDE TIMERS ( 32/64 bit Timers)
#define WTIMER0_PCLK_EN()        ( SYS_CTRL->RCGCWTIMER |= ( 1<<0 ) )
#define WTIMER1_PCLK_EN()        ( SYS_CTRL->RCGCWTIMER |= ( 1<<1 ) )
#define WTIMER2_PCLK_EN()        ( SYS_CTRL->RCGCWTIMER |= ( 1<<2 ) )
#define WTIMER3_PCLK_EN()        ( SYS_CTRL->RCGCWTIMER |= ( 1<<3 ) )
#define WTIMER4_PCLK_EN()        ( SYS_CTRL->RCGCWTIMER |= ( 1<<4 ) )
#define WTIMER5_PCLK_EN()        ( SYS_CTRL->RCGCWTIMER |= ( 1<<5 ) )

// CLOCK DISABLE MACROS FOR WIDE TIMERS ( 32/64 bit Timers)
#define WTIMER0_PCLK_DIS()       ( SYS_CTRL->RCGCWTIMER &= ~( 1<<0 ) )
#define WTIMER1_PCLK_DIS()       ( SYS_CTRL->RCGCWTIMER &= ~( 1<<1 ) )
#define WTIMER2_PCLK_DIS()       ( SYS_CTRL->RCGCWTIMER &= ~( 1<<2 ) )
#define WTIMER3_PCLK_DIS()       ( SYS_CTRL->RCGCWTIMER &= ~( 1<<3 ) )
#define WTIMER4_PCLK_DIS()       ( SYS_CTRL->RCGCWTIMER &= ~( 1<<4 ) )
#define WTIMER5_PCLK_DIS()       ( SYS_CTRL->RCGCWTIMER &= ~( 1<<5 ) )



/************************************************************************************/

/*-------------------------------- SLEEP MODE --------------------------------------*/

/************************************************************************************/

/*--------------------------- Deep SLEEP MODE --------------------------------------*/

/************************************************************************************/


/*--------------------- BIT POSITION DEFINITION OF SSI -----------------------------*/


#define SSI_CR0_SPO      6
#define SSI_CR0_SPH      7

#define SSI_CR1_LBM      0
#define SSI_CR1_SSE      1
#define SSI_CR1_MS       2
#define SSI_CR1_EOT      4


#define SSI_SR_TFE       0
#define SSI_SR_TNF       1
#define SSI_SR_RNE       2
#define SSI_SR_RFF       3
#define SSI_SR_BSY       4

#define SSI_IM_RORIM     0
#define SSI_IM_RTIM      1
#define SSI_IM_RXIM      2
#define SSI_IM_TXIM      3

#define SSI_IS_RORIS     0
#define SSI_IS_RTIS      1
#define SSI_IS_RXIS      2
#define SSI_IS_TXIS      3

#define SSI_MIS_RORMIS   0
#define SSI_MIS_RTMIS    1
#define SSI_MIS_RXMIS    2
#define SSI_MIS_TXMIS    3



#define SSI_ICR_RORIC    0
#define SSI_ICR_RTIC     1






/************************************************************************************/
/*--------------------- BIT POSITION DEFINITION OF Timer registers -----------------------------*/

// CTL REGISTER
#define TIMER_CTL_TAEN         0
#define TIMER_CTL_TASTALL      1
#define TIMER_CTL_TAEVENT      2      // 2bits :2,3
#define TIMER_CTL_RTCEN        4
#define TIMER_CTL_TAOTE        5
#define TIMER_CTL_TAPWML       6
#define TIMER_CTL_TBEN         8
#define TIMER_CTL_TBSTALL      9
#define TIMER_CTL_TBEVENT      10      //  10 , 11
#define TIMER_CTL_TBOTE        13
#define TIMER_CTL_TBPWML       14



// CFG REGISTER
#define TIMER_CFG_GPTMCFG      0   // 0,1,2 BITS

// TAMR REGISTER
#define TIMER_TAMR_TAMR        0  // 0,1
#define TIMER_TAMR_TACMR       2
#define TIMER_TAMR_TAAMS       3
#define TIMER_TAMR_TACDIR      4
#define TIMER_TAMR_TAMIE       5
#define TIMER_TAMR_TAWOT       6
#define TIMER_TAMR_TASNAPS     7
#define TIMER_TAMR_TALID       8
#define TIMER_TAMR_TAPWMIE     9
#define TIMER_TAMR_TAMRSU     10
#define TIMER_TAMR_TAPLO      11



// TBMR REGISTER
#define TIMER_TBMR_TBMR        0  // 0,1
#define TIMER_TBMR_TBCMR       2
#define TIMER_TBMR_TBAMS       3
#define TIMER_TBMR_TBCDIR      4
#define TIMER_TBMR_TBMIE       5
#define TIMER_TBMR_TBWOT       6
#define TIMER_TBMR_TBSNAPS     7
#define TIMER_TBMR_TBLID       8
#define TIMER_TBMR_TBPWMIE     9
#define TIMER_TBMR_TBMRSU     10
#define TIMER_TBMR_TBPLO      11

/************************************************************************************/
/*--------------------- BIT POSITION DEFINITION OF Timer registers -----------------------------*/

//  ADC Trigger Source Select (ADCTSSEL)
#define AD_TSSEL_PS0   4
#define AD_TSSEL_PS1   12
#define AD_TSSEL_PS2   20
#define AD_TSSEL_PS3   28

/************************************************************************************/
/*--------------------- BIT POSITION DEFINITION OF CAN registers -----------------------------*/

#define CAN_CTL_INIT   0
#define CAN_CTL_IE     1
#define CAN_CTL_SIE    2
#define CAN_CTL_EIE    3
#define CAN_CTL_DAR    5
#define CAN_CTL_CCE    6
#define CAN_CTL_TEST   7


#define CAN_BIT_PRB     0
#define CAN_BIT_SJW     6
#define CAN_BIT_TSEG1   8
#define CAN_BIT_TSEG2   12


#define CAN_IF1ARB2_MSGVAL     15
#define CAN_IF2ARB2_MSGVAL     15

#define CAN_CANBRPE_BRPE    0






/************************************************************************************/























// Some Generic Macros

#define ENABLE           1
#define DISABLE          0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_SET_PIN     SET
#define GPIO_RESET_PIN   RESET


#include "GPIO_driver_interface.h"
#include "SSI_driver_interface.h"
#include "Timer_Driver_interface.h"
#include "drivers/inc/ADC_Driver_interface.h"
#include "drivers/inc/CAN_driver_interface.h"










#endif /* TIVAC_PERIPHERALS_H_ */
