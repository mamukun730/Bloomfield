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
* File Name    : r_init_stop_module.h
* Version      : Ver 1.00
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

/* Guards against multiple inclusion */
#ifndef R_INIT_STOP_MODULE_H 
#define R_INIT_STOP_MODULE_H 

/******************************************************************************
Macro definitions
******************************************************************************/
#define MODULE_STOP_ENABLE    (1)
#define MODULE_STOP_DISABLE   (0)

#define MSTP_STATE_DMACDTC (MODULE_STOP_DISABLE)  /* DMAC/DTC trans to module-stop state  */
#define MSTP_STATE_EXDMAC  (MODULE_STOP_DISABLE)  /* EXDMAC trans to module-stop state  */
#define MSTP_STATE_RAM0    (MODULE_STOP_DISABLE)  /* RAM0 trans to module-stop state */
#define MSTP_STATE_RAM1    (MODULE_STOP_DISABLE)  /* RAM1 trans to module-stop state */

/*******************************************************************************
Exported global variables and functions (to be accessed by other files)
*******************************************************************************/
void R_INIT_StopModule(void);

#endif  /* R_INIT_STOP_MODULE_H */

/* End of File */

