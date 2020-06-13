/*
 * CAN_driver_interface.h
 *
 *  Created on: May 9, 2020
 *      Author: Mohamed  Ebead
 */

#ifndef DRIVERS_INC_CAN_DRIVER_INTERFACE_H_
#define DRIVERS_INC_CAN_DRIVER_INTERFACE_H_


typedef struct
{
   u_int_8 MsgObject ;    // @_Msg_Obj_Needed_Or_Not needed or not needed
   u_int_8 BRP       ;    // [1 ... 64]   Defines the length of the time quantum tq.
   u_int_8 Prob      ;    // [1 ... 8]   Compensates for the physical delay times
   u_int_8 Phase1    ;    // [1 ... 8]  May be lengthened temporarily by synchronization
   u_int_8 Phase2    ;    // [1 ... 8]  May be shortened temporarily by synchronization
   u_int_8 SJW       ;    // [1 ... 4]  May not be longer than either Phase Buffer Segment
   u_int_8 BRPE       ;   // [0x00 ... 0x0F] Extend the BRP bit in the CANBIT register to values up to 1023



}CAN_Config_t ;


typedef struct
{

    CAN_RegDef_t *CAN ;
    CAN_Config_t *CAN_Config ;

}CAN_Handler_t;




// @_Msg_Obj_Needed_Or_Not
#define CAN_MSG_OBJ_NOT_NEEDED   0
#define CAN_MSG_OBJ_NEEDED       1












/*-------------------------------------------------------------------------*/
/*--------------------        CAN Related APIs       ----------------------*/
/*-------------------------------------------------------------------------*/

void CAN_ClkCtrl (CAN_RegDef_t *pCANx , u_int_8 ENOrDIS) ;
void CAN_Init (CAN_Handler_t *pCANx_Handler) ;


















#endif /* DRIVERS_INC_CAN_DRIVER_INTERFACE_H_ */
