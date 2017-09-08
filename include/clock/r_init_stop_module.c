/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2014 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/*******************************************************************************
* System Name  : RX63N initialization example
* File Name    : r_init_stop_module.c
* Version      : Ver 1.10
* Device       : R5F563NBDDFC(RX63N Group)
* Abstract     : Stopping the peripherals which start operations after a reset
* Tool-Chain   : High-performance Embedded Workshop (Version 4.09.01.007)
*              : C/C++ Compiler Package for RX Family (V.1.02 Release 01)
* OS           : not use
* H/W Platform : Renesas Starter Kit for RX63N
* Description  : Configure the setting to enter module-stop state.
* Limitation   : none
*******************************************************************************/
/*******************************************************************************
* History      : DD.MM.YYYY Version  Description
*              : 01.04.2013 1.00     First Release
*              : 06.01.2014 1.10     Change File Header
*******************************************************************************/

/******************************************************************************
Includes <System Includes> , "Project Includes"
******************************************************************************/
#include "..\..\iodefine.h"
#include "r_init_stop_module.h"

/*******************************************************************************
* Outline      : Stopping the peripherals which start operations after a reset
* Header       : r_init_stop_module.h
* Function Name: R_INIT_StopModule
* Description  : Configure the setting to enter module-stop state.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void R_INIT_StopModule(void)
{

    /* ---- Enable write protection ---- */
    /* PRCR - Protect Register 
    b15:b8   PRKEY    - PRC Key Code - A5h
                        (The write value should be A5h to permission writing PRCi bit)
    b7:b4    Reserved - The write value should be 0.
    b3       PRC3     - Protect Bit 3 - Write disabled
    b2       Reserved - The write value should be 0.
    b1       PRC1     - Protect Bit 1 - Write enabled
    b0       PRC0     - Protect Bit 0 - Write disabled */
    SYSTEM.PRCR.WORD = 0xA502;

    /* ---- Set transition to module-stop state ---- */
#if MSTP_STATE_DMACDTC == MODULE_STOP_ENABLE 
    MSTP(DTC) = 1;            /* DMAC/DTC trans to module-stop state */
#endif 

#if MSTP_STATE_EXDMAC == MODULE_STOP_ENABLE 
    MSTP(EXDMAC) = 1;         /* EXDMAC trans to module-stop state */
#endif 

#if MSTP_STATE_RAM0 == MODULE_STOP_ENABLE 
    MSTP(RAM0) = 1;           /* RAM0 trans to module-stop state */
#endif 

#if MSTP_STATE_RAM1 == MODULE_STOP_ENABLE 
    MSTP(RAM1) = 1;           /* RAM1 trans to module-stop state */
#endif 

    /* ---- Disable write protection ---- */
    /* PRCR - Protect Register 
    b15:b8   PRKEY    - PRC Key Code - A5h
                         (The write value should be A5h to permission writing PRCi bit)
    b1       PRC1     - Protect Bit 1 - Write disabled */
    SYSTEM.PRCR.WORD = 0xA500;

}

/* End of File */

