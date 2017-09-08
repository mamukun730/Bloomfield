/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No 
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all 
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM 
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES 
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS 
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of 
* this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer 
*
* Copyright (C) 2013 Renesas Electronics Corporation. All rights reserved.    
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : locking.h
* Description  : This implements a locking mechanism that can be used by all code. The locking is done atomically so
*                common resources can be accessed safely.
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 07.03.2012 1.00     First Release
*         : 20.09.2012 1.10     Added ability for user to implement their own locking if  
*                               BSP_CFG_USER_LOCKING_ENABLED != 0.
*         : 19.11.2012 1.20     Updated code to use 'BSP_' and 'BSP_CFG_' prefix for macros.
*         : 16.01.2013 1.30     Added const qualifiers to lock functions.
***********************************************************************************************************************/

#ifndef LOCKING_H
#define LOCKING_H

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
/* Lock types. */
#include "mcu_locks.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" void xchg(int32_t *data1, int32_t *data2);
#endif

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global functions (to be accessed by other files)
***********************************************************************************************************************/
bool R_BSP_SoftwareLock(BSP_CFG_USER_LOCKING_TYPE * const plock);
bool R_BSP_SoftwareUnlock(BSP_CFG_USER_LOCKING_TYPE * const plock);
bool R_BSP_HardwareLock(mcu_lock_t const hw_index);
bool R_BSP_HardwareUnlock(mcu_lock_t const hw_index);

#if BSP_CFG_USER_LOCKING_ENABLED != 0
/* Is user is using their own lock functions then these are the prototypes. */
bool BSP_CFG_USER_LOCKING_SW_LOCK_FUNCTION(BSP_CFG_USER_LOCKING_TYPE * const plock);
bool BSP_CFG_USER_LOCKING_SW_UNLOCK_FUNCTION(BSP_CFG_USER_LOCKING_TYPE * const plock);
bool BSP_CFG_USER_LOCKING_HW_LOCK_FUNCTION(mcu_lock_t const hw_index);
bool BSP_CFG_USER_LOCKING_HW_UNLOCK_FUNCTION(mcu_lock_t const hw_index);
#endif

#endif /* LOCKING_H */

