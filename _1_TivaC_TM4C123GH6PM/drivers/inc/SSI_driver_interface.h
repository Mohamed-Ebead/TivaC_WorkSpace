/*
 * SSI_driver_interface.h
 *
 *  Created on: May 4, 2020
 *      Author: Mohamed  Ebead
 */

#ifndef DRIVERS_INC_SSI_DRIVER_INTERFACE_H_
#define DRIVERS_INC_SSI_DRIVER_INTERFACE_H_


#include "STD_TYPES.h"
//#include "TivaC_peripherals.h"


// SSI configuration structure

typedef struct
{
    u_int_8 SSI_DeviceMode  ;       // @SSI_DeviceMode    Master or Slave
    u_int_8 SSI_BusConfig   ;       // @SSI_BusConfig     Full duplex / Half Duplex / Simplex
    u_int_8 SSI_SClkRate  ;       // @SSI_SclkSpeed     Bus Speed
    u_int_8 SSI_DFF         ;       // @SSI_DFF           DATA frame format
    u_int_8 SSI_DSS         ;       // @SSI_DSS           DATA size select
    u_int_8 SSI_CPOL        ;       // @CPOL              clock polarity
    u_int_8 SSI_CPHA        ;       // @CPHA              clock phase
    u_int_8 SSI_SSM         ;       // @SSI_SSM           Software Slave Management

}SSI_Config_t;


// SSI Handler structure

typedef struct
{
    SSI_RegDef_t *pSSIx ;
    SSI_Config_t *pSSI_Config ;

}SSI_Handler_t;


// don't ever change these macro values
// don't ever change these macro values
// don't ever change these macro values
// don't ever change these macro values
// don't ever change these macro values
// don't ever change these macro values
// don't ever change these macro values

/*
 * @SSI_DeviceMode
 */
#define SSI_DEVICE_MODE_MASTER    0
#define SSI_DEVICE_MODE_SLAVE     1



/*
 * @SSI_BusConfig
 */
#define SSI_BUS_CONFIG_FD                1
#define SSI_BUS_CONFIG_HD                2
#define SSI_BUS_CONFIG_SIMPLEX_RXONLY    3




/*
 * @SSI_SclkRate
 */
/*
SSI Serial Clock Rate
This bit field is used to generate the transmit and receive bit rate of the
SSI. The bit rate is:
BR( bit rate )=SysClk/(CPSDVSR * (1 + SCR))
where CPSDVSR is an even value from 2-254 programmed in the
SSICPSR register, and SCR is a value from 0-255.
*/

// we will keep SCR as 0 and configure CPSDVSR prescaler
// don't ever change these macro values
#define SSI_SCLK_RATE_DIV2              2
#define SSI_SCLK_RATE_DIV4              4
#define SSI_SCLK_RATE_DIV8              8
#define SSI_SCLK_RATE_DIV16             16
#define SSI_SCLK_RATE_DIV32             32
#define SSI_SCLK_RATE_DIV64             64
#define SSI_SCLK_RATE_DIV128            128
#define SSI_SCLK_RATE_DIV254            254

/*
 * @SSI_DFF
 */
#define SSI_FF_FREESCALE_SPI       0
#define SSI_FF_TEXAS_INSTRUMENTS   1
#define SSI_FF_MICROWAVE           2


/*
 * @SSI_DSS  data size select
 */
// don't ever change these macro values
#define SSI_DSS_4BITS       0x3
#define SSI_DSS_5BITS       0x4
#define SSI_DSS_6BITS       0x5
#define SSI_DSS_7BITS       0x6
#define SSI_DSS_8BITS       0x7
#define SSI_DSS_9BITS       0x8
#define SSI_DSS_10BITS      0x9
#define SSI_DSS_11BITS      0xA
#define SSI_DSS_12BITS      0xB
#define SSI_DSS_13BITS      0xC
#define SSI_DSS_14BITS      0xD
#define SSI_DSS_15BITS      0xE
#define SSI_DSS_16BITS      0xF

/*
 * @CPOL
 */
#define SSI_CPOL_LOW 0     //0 A steady state Low value is placed on the SSInClk pin
#define SSI_CPOL_HIGH 1    // 1 A steady state High value is placed on the SSInClk pin when data is not being transferred.


/*
 * @CPHA
 */
#define SSI_CPHA_LOW  0    //0 Data is captured on the first clock edge transition.
#define SSI_CPHA_HIGH 1    //1 Data is captured on the second clock edge transition.


/*
 * @SSI_SSM
 */
#define SSI_SSM_EN     1
#define SSI_SSM_DI     0










/*--------------------------------------------------------------*/
/**------------------------  SSI APIs  ------------------------**/
/*--------------------------------------------------------------*/



// Peripheral Clock Setup
void SSI_ClockCtrl (SSI_RegDef_t *pSSIx , u_int_8 EnOrDis) ;

// Initialize and DeInitialize
void SSI_Init   (SSI_Handler_t *pSSIxHandler )   ;
void SSI_DeInit (SSI_RegDef_t *pGPIOx )   ;

// Data Send and Recive
void SPI_SendData_Blocking (SSI_RegDef_t *pSSIx , u_int_8 *pTXBuffer , u_int_32 Length ) ;
void SPI_RecieveData_Blocking (SSI_RegDef_t *pSSIx , u_int_8 *pTXBuffer , u_int_32 Length ) ;

void SPI_SendData_NonBlocking (SSI_Handler_t *pSSIxHandler , u_int_8 *pTXBuffer , u_int_32 Length ) ;
void SPI_RecieveData_NonBlocking (SSI_Handler_t *pSSIxHandler , u_int_8 *pRXBuffer , u_int_32 Length ) ;


// IRQ configuration and ISR handling
void SSI_IRQInterruptConfig (u_int_8 IRQNumber , u_int_8 EnOrDis) ;
void SSI_IRQPriorityConfig (u_int_8 IRQNumber , u_int_8 IRQPriority) ;
void SSIx_IRQHandling (SSI_Handler_t *pSSIxHandler )   ;



/*--------------------------------  SSI PINS CONFIG  -------------------------------------*/

#define SSI0_GPIO    GPIOA
#define SSI1_GPIO    GPIOF
#define SSI2_GPIO    GPIOB
#define SSI3_GPIO    GPIOD


// SSI0
#define SCK0    GPIOA_PIN_2
#define CS0     GPIOA_PIN_3
#define MISO0   GPIOA_PIN_4
#define MOSI0   GPIOA_PIN_5

//SSI1
#define SCK1    GPIOF_PIN_2
#define CS1     GPIOF_PIN_3
#define MISO1   GPIOF_PIN_0
#define MOSI1   GPIOF_PIN_1

//SSI2
#define SCK2    GPIOB_PIN_4
#define CS2     GPIOB_PIN_5
#define MISO2   GPIOB_PIN_6
#define MOSI2   GPIOB_PIN_7

//SSI3
#define SCK3    GPIOD_PIN_0
#define CS3     GPIOD_PIN_1
#define MISO3   GPIOD_PIN_2
#define MOSI3   GPIOD_PIN_3

// SSI0
#define  GPIOA_PIN_2     GPIO_PIN_NO_2
#define  GPIOA_PIN_3     GPIO_PIN_NO_3
#define  GPIOA_PIN_4     GPIO_PIN_NO_4
#define  GPIOA_PIN_5     GPIO_PIN_NO_5

//SSI1
#define  GPIOF_PIN_2     GPIO_PIN_NO_2
#define  GPIOF_PIN_3     GPIO_PIN_NO_3
#define  GPIOF_PIN_0     GPIO_PIN_NO_0
#define  GPIOF_PIN_1     GPIO_PIN_NO_1

//SSI2
#define  GPIOB_PIN_4     GPIO_PIN_NO_4
#define  GPIOB_PIN_5     GPIO_PIN_NO_5
#define  GPIOB_PIN_6     GPIO_PIN_NO_6
#define  GPIOB_PIN_7     GPIO_PIN_NO_7

//SSI3
#define  GPIOD_PIN_0     GPIO_PIN_NO_0
#define  GPIOD_PIN_1     GPIO_PIN_NO_1
#define  GPIOD_PIN_2     GPIO_PIN_NO_2
#define  GPIOD_PIN_3     GPIO_PIN_NO_3















#endif /* DRIVERS_INC_SSI_DRIVER_INTERFACE_H_ */
