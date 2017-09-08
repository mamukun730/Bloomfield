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
* File Name    : r_init_clock.h
* Version      : Ver 1.10
* Device       : R5F563NBDDFC(RX63N Group)
* Abstract     : Program example of RX63N initialization
* Tool-Chain   : High-performance Embedded Workshop (Version 4.09.01.007)
*              : C/C++ Compiler Package for RX Family (V.1.02 Release 01)
* OS           : not use
* H/W Platform : Renesas Starter Kit for RX63N
* Description  : Initialize the clock.
* Limitation   : none
*******************************************************************************/
/*******************************************************************************
* History      : DD.MM.YYYY Version  Description
*              : 01.04.2013 1.00     First Release
*              : 02.09.2013 1.01     Drive ability for Low CL add
*              : 06.01.2014 1.10     Change File Header
*******************************************************************************/

/* Guards against multiple inclusion */
#ifndef R_INIT_CLOCK_H 
#define R_INIT_CLOCK_H 

/******************************************************************************
Macro definitions
******************************************************************************/
/* ---- Please set the main clock and the sub-clock ---- */
#define MAIN_CLOCK_Hz  (12000000L)       /* This sample code uses 12MHz */
#define SUB_CLOCK_Hz   (32768L)          /* This sample code uses 32.768kHz */

/* ---- Please set wait processing for the clock oscillation stabilization ---- */
#define WAIT_TIME_FOR_MAIN_OSCILLATION (11026000L)
#define WAIT_TIME_FOR_SUB_OSCILLATION  (2600000000L)
//#define WAIT_TIME_FOR_SUB_OSCILLATION  (3300000000L) /* 64Pin Package */
#define WAIT_TIME_FOR_PLL_OSCILLATION  (1865000L)
#define WAIT_TIME_FOR_HOCO_OSCILLATION (2000000L)

/* ---- Please choose the sub-clock pattern ---- */
#define PATTERN_A       (1)             /* Sub-clock pattern A */
#define PATTERN_B       (2)             /* Sub-clock pattern B */
#define PATTERN_C       (3)             /* Sub-clock pattern C */
#define PATTERN_D       (4)             /* Sub-clock pattern D */
#define PATTERN_E       (5)             /* Sub-clock pattern E */
#define PATTERN_48      (PATTERN_A)     /* Sub-clock pattern 48Pin Package */

/* Select the sub-clock pattern to use. (Pattern A to E) */
#define SELECT_SUB      (PATTERN_A)     /* This sample code uses pattern A. */

//#define LOW_CL                        /* Drive ability for Low CL Used */

/*******************************************************************************
Exported global variables and functions (to be accessed by other files)
*******************************************************************************/
void R_INIT_Clock(void);
void CGC_oscillation_main(void);
void CGC_oscillation_PLL(void);
void CGC_oscillation_HOCO(void);
void CGC_no_use_subclk(void);
void CGC_disable_subclk_RTC_use_mainclk(void);
void CGC_subclk_as_sysclk(void);
void CGC_subclk_as_RTC(void);
void CGC_subclk_as_sysclk_RTC(void);

#endif  /* R_INIT_CLOCK_H */

/* End of File */

