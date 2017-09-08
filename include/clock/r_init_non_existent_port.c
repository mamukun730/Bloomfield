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
* File Name    : r_init_non_existent_port.c
* Version      : Ver 1.10
* Device       : R5F563NBDDFC(RX63N Group)
* Abstract     : Example of non-existent port initialization for RX63N
* Tool-Chain   : High-performance Embedded Workshop (Version 4.09.01.007)
*              : C/C++ Compiler Package for RX Family (V.1.02 Release 01)
* OS           : not use
* H/W Platform : Renesas Starter Kit for RX63N
* Description  : Initialization of non-existent ports
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
#include "r_init_non_existent_port.h"

/*******************************************************************************
* Outline      : Initialization of non-existent ports
* Header       : r_init_non_existent_port.h
* Function Name: R_INIT_NonExistentPort
* Description  : Initialize non-existent ports.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void R_INIT_NonExistentPort(void)
{
    /* PDR - Port Direction Register
    0:Input(Function as an input pin.)
    1:Output(Function as an output pin.) */

#if DEF_P0PDR != 0x00 
    /* ---- Set the PORT0.PDR register ---- */
    PORT0.PDR.BYTE = PORT0.PDR.BYTE | DEF_P0PDR;
#endif 

#if DEF_P1PDR != 0x00 
    /* ---- Set the PORT1.PDR register ---- */
    PORT1.PDR.BYTE = PORT1.PDR.BYTE | DEF_P1PDR;
#endif 

#if DEF_P2PDR != 0x00 
    /* ---- Set the PORT2.PDR register ---- */
    PORT2.PDR.BYTE = PORT2.PDR.BYTE | DEF_P2PDR;
#endif 

#if DEF_P3PDR != 0x00 
    /* ---- Set the PORT3.PDR register ---- */
    PORT3.PDR.BYTE = PORT3.PDR.BYTE | DEF_P3PDR;
#endif 

#if DEF_P4PDR != 0x00 
    /* ---- Set the PORT4.PDR register ---- */
    PORT4.PDR.BYTE = PORT4.PDR.BYTE | DEF_P4PDR;
#endif 

#if DEF_P5PDR != 0x00 
    /* ---- Set the PORT5.PDR register ---- */
    PORT5.PDR.BYTE = PORT5.PDR.BYTE | DEF_P5PDR;
#endif 

#if DEF_P6PDR != 0x00 
    /* ---- Set the PORT6.PDR register ---- */
    PORT6.PDR.BYTE = PORT6.PDR.BYTE | DEF_P6PDR;
#endif 

#if DEF_P7PDR != 0x00 
    /* ---- Set the PORT7.PDR register ---- */
    PORT7.PDR.BYTE = PORT7.PDR.BYTE | DEF_P7PDR;
#endif 

#if DEF_P8PDR != 0x00 
    /* ---- Set the PORT8.PDR register ---- */
    PORT8.PDR.BYTE = PORT8.PDR.BYTE | DEF_P8PDR;
#endif 

#if DEF_P9PDR != 0x00 
    /* ---- Set the PORT9.PDR register ---- */
    PORT9.PDR.BYTE = PORT9.PDR.BYTE | DEF_P9PDR;
#endif 

#if DEF_PAPDR != 0x00 
    /* ---- Set the PORTA.PDR register ---- */
    PORTA.PDR.BYTE = PORTA.PDR.BYTE | DEF_PAPDR;
#endif 

#if DEF_PBPDR != 0x00 
    /* ---- Set the PORTB.PDR register ---- */
    PORTB.PDR.BYTE = PORTB.PDR.BYTE | DEF_PBPDR;
#endif 

#if DEF_PCPDR != 0x00 
    /* ---- Set the PORTC.PDR register ---- */
    PORTC.PDR.BYTE = PORTC.PDR.BYTE | DEF_PCPDR;
#endif 

#if DEF_PDPDR != 0x00 
    /* ---- Set the PORTD.PDR register ---- */
    PORTD.PDR.BYTE = PORTD.PDR.BYTE | DEF_PDPDR;
#endif 

#if DEF_PEPDR != 0x00 
    /* ---- Set the PORTE.PDR register ---- */
    PORTE.PDR.BYTE = PORTE.PDR.BYTE | DEF_PEPDR;
#endif 

#if DEF_PFPDR != 0x00 
    /* ---- Set the PORTF.PDR register ---- */
    PORTF.PDR.BYTE = PORTF.PDR.BYTE | DEF_PFPDR;
#endif 

#if DEF_PGPDR != 0x00 
    /* ---- Set the PORTG.PDR register ---- */
    PORTG.PDR.BYTE = PORTG.PDR.BYTE | DEF_PGPDR;
#endif 

#if DEF_PJPDR != 0x00 
    /* ---- Set the PORTJ.PDR register ---- */
    PORTJ.PDR.BYTE = PORTJ.PDR.BYTE | DEF_PJPDR;
#endif 

}

/* End of File */

