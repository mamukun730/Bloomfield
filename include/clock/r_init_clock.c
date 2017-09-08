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
* File Name    : r_init_clock.c
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

/******************************************************************************
Includes <System Includes> , "Project Includes"
******************************************************************************/
#include <stdint.h>
#include "..\..\iodefine.h"
#include "r_init_clock.h"

/******************************************************************************
Macro definitions
******************************************************************************/
#define MAIN_CLOCK_CYCLE (1000000000L/MAIN_CLOCK_Hz)
#define SUB_CLOCK_CYCLE  (1000000000L/SUB_CLOCK_Hz)

/* CMT0 count use LOCO when RTC count source is the main clock */
#define FOR_CMT0_LOCO    (222608)

#if (SELECT_SUB == PATTERN_A) || (SELECT_SUB == PATTERN_B) 
    /* Count source of RTC use main clock */
    #define FOR_CMT0_TIME    (581)
    /* Time for one count of CMT0 is approximately 581 ns
       when the count source is HOCO divided by 32 (max. of HOCO = 55 MHz) */
#elif (SELECT_SUB == PATTERN_C) || (SELECT_SUB == PATTERN_D) || (SELECT_SUB == PATTERN_E) 
    /* Count source of RTC use sub-clock */
    #define FOR_CMT0_TIME    (222608)
    /* Time for one count of CMT0 is approximately 222608 ns
       when the count source is LOCO divided by 32 (max. of LOCO = 143.75 kHz) */
#else 
#endif 

/*******************************************************************************
Private variables and functions
*******************************************************************************/
static void disable_subclk(void);
static void oscillation_subclk(void);
static void no_use_subclk_as_sysclk(void);
static void resetting_wtcr_mainclk(void);
static void resetting_wtcr_subclk(void);
static void enable_RTC(void);
static void disable_RTC_mainclk(void);
static void disable_RTC_subclk(void);
static void cmt0_wait(uint32_t);

/*******************************************************************************
* Outline      : Clock initialization
* Header       : r_init_clock.h
* Function Name: R_INIT_Clock
* Description  : Initialize the clock.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void R_INIT_Clock(void)
{

    /* ---- Enable write protection ---- */
    /* PRCR - Protect Register 
    b15:b8   PRKEY    - PRC Key Code - A5h 
                        (The write value should be A5h to permission writing PRCi bit)
    b7:b4    Reserved - The write value should be 0.
    b3       PRC3     - Protect Bit 3 - Write disabled
    b2       Reserved - The write value should be 0.
    b1       PRC1     - Protect Bit 1 - Write enabled
    b0       PRC0     - Protect Bit 0 - Write enabled */
    SYSTEM.PRCR.WORD = 0xA503;

    /* ---- Operate the main clock oscillator ---- */
    CGC_oscillation_main();

    /* ---- Set the sub-clock  ---- */
#if SELECT_SUB == PATTERN_A 
    CGC_no_use_subclk();                     /* the sub-clock pattern A */
#elif SELECT_SUB == PATTERN_B 
    CGC_disable_subclk_RTC_use_mainclk();    /* the sub-clock pattern B */
#elif SELECT_SUB == PATTERN_C 
    CGC_subclk_as_sysclk();                  /* the sub-clock pattern C */
#elif SELECT_SUB == PATTERN_D 
    CGC_subclk_as_RTC();                     /* the sub-clock pattern D */
#elif SELECT_SUB == PATTERN_E 
    CGC_subclk_as_sysclk_RTC();              /* the sub-clock pattern E */
#else 
#endif 

    /* ---- Operate the PLL ---- */
    CGC_oscillation_PLL();

    /* ---- Set the internal clock division ratio ---- */
    /* SCKCR - System Clock Control Register
    b31:b28  FCK      - FlashIF Clock(FCLK) Select - divide-by-4
    b27:b24  ICK      - System Clock (ICLK) Select - divide-by-2
    b23      PSTOP1   - BCLK Pin Output Control    - disabled. (Fixed high)
    b22      PSTOP0   - SDCLK Pin Output Control   - disabled. (Fixed high)
    b21:b20  Reserved - The write value should be 0.
    b19:b16  BCK      - External Bus Clock (BCLK) Select - divide-by-4
    b15:b12  PCLKA    - Peripheral Module Clock A(PCLKA) Select - divide-by-2
    b10:b8   PCLKB    - Peripheral Module Clock B(PCLKB) Select - divide-by-4
    b7:b4    Reserved - The write value should be 0001b.
    b3:b0    Reserved - The write value should be 0001b. */
    SYSTEM.SCKCR.LONG = 0x21C21211;

    while (0x21C21211 != SYSTEM.SCKCR.LONG) 
    {
         /* Confirm that the written value can be read correctly. */
    }

    /* SCKCR2 - System Clock Control Register 2
    b15:b8   Reserved - The write value should be 0.
    b7:b4    UCK      - USB Clock (UCLK) Select     - USB No Use
    b3:b0    IECLK    - IEBUS Clock (IECLK) Select  - divide-by-4 */
    SYSTEM.SCKCR2.WORD = 0x0012;

    while (0x0012 != SYSTEM.SCKCR2.WORD) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Set the BCLK pin output ---- */
    /* BCKCR - External Bus Clock Control Register
    b7:b1    Reserved - The write value should be 0.
    b0       BCLKDIV  - BCLK Pin Output Select - no division */
    SYSTEM.BCKCR.BYTE = 0x00;

    while (0x00 != SYSTEM.BCKCR.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Set the internal clock source ---- */
    /* SCKCR3 - System Clock Control Register 3
    b15:b11  Reserved - The write value should be 0.
    b10:b8   CKSEL    - Clock Source Select - PLL circuit is selected.
    b7:b0    Reserved - The write value should be 0. */
    SYSTEM.SCKCR3.WORD = 0x0400;

    while (0x0400 != SYSTEM.SCKCR3.WORD) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Turn off the HOCO power supply ---- */
    /* HOCOCR - High-Speed On-Chip Oscillator Control Register
    b7:b1    Reserved - The write value should be 0.
    b0       HCSTP    - HOCO Stop - the HOCO is stopped.*/
    SYSTEM.HOCOCR.BYTE = 0x01;

    /* HOCOPCR - High-Speed On-Chip Oscillator Power Supply Control Register
    b7:b1    Reserved - The write value should be 0.
    b0       HOCOPCNT - High-Speed On-Chip Oscillator Power Supply Control
                      - HOCO power supply is turned off. */
    SYSTEM.HOCOPCR.BYTE = 0x01;

    /* ---- Disable write protection ---- */
    /* PRCR - Protect Register 
    b15:b8   PRKEY    - PRC Key Code - A5h 
                        (The write value should be A5h to permission writing PRCi bit)
    b1       PRC1     - Protect Bit 1 - Write disabled
    b0       PRC0     - Protect Bit 0 - Write disabled */
    SYSTEM.PRCR.WORD = 0xA500;

}

/*******************************************************************************
* Outline      : Configure main clock oscillation
* Header       : r_init_clock.h
* Function Name: CGC_oscillation_main
* Description  : Set the wait control register (MOSCWTCR), 
*                then enable main clock oscillation.
*                Wait for the main clock oscillation stabilization
*                wait time by a software.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void CGC_oscillation_main(void)
{

    /* ---- Set wait time until the main clock oscillator stabilizes ---- */
    /* MOSCWTCR - Main Clock Oscillator Wait Control Register
    b7:b5    Reserved - The write value should be 0.
    b4:b0    MSTS     - Main Clock Oscillator Waiting Time
                      - Wait time is 65536 cycles (approx. 5.46 ms). */
    SYSTEM.MOSCWTCR.BYTE = 0x0C;

    /* ---- Operate the main clock oscillator ---- */
    /* MOSCCR   - Main Clock Oscillator Control Register
    b7:b1    Reserved - The write value should be 0.
    b0       MOSTP    - Main Clock Oscillator Stop - Main clock oscillator is operating. */
    SYSTEM.MOSCCR.BYTE = 0x00;

    while (0x00 != SYSTEM.MOSCCR.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Wait processing for the clock oscillation stabilization ---- */
#if (SELECT_SUB == PATTERN_A) || (SELECT_SUB == PATTERN_B) 
    cmt0_wait((WAIT_TIME_FOR_MAIN_OSCILLATION/FOR_CMT0_LOCO)+1);        /* LOCO is cmt0 count source. */
                                                                        /* oscillation stabilize (11.026 ms). */
#elif (SELECT_SUB == PATTERN_C) || (SELECT_SUB == PATTERN_D) || (SELECT_SUB == PATTERN_E) 
    cmt0_wait((WAIT_TIME_FOR_MAIN_OSCILLATION/FOR_CMT0_TIME)+1);        /* LOCO is cmt0 count source. */
                                                                        /* oscillation stabilize (11.026 ms). */
#else 
#endif 

}

/*******************************************************************************
* Outline      : Configure PLL clock oscillation
* Header       : r_init_clock.h
* Function Name: CGC_oscillation_PLL
* Description  : Set the PLL input frequency division ratio and 
*                frequency multiplication factor, set the PLLWTCR, 
*                then enable PLL clock oscillation.
*                Wait for the PLL clock oscillation stabilization time.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void CGC_oscillation_PLL(void)
{

    /* ---- Set the PLL division ratio and multiplication factor ---- */
    /* PLLCR - PLL Control Register
    b15:b14  Reserved - The write value should be 0.
    b13:b8   STC      - Frequency Multiplication Factor Select 
                      - Frequency multiplication factor is multiply-by-16.
    b7:b2    Reserved - The write value should be 0.
    b1:b0    PLIDIV   - PLL Input Frequency Division Ratio Select 
                      - PLL input division ratio is no division. */
    SYSTEM.PLLCR.WORD = 0x0F00;

    /* ---- Set wait time until the PLL clock oscillator stabilizes ---- */
    /* PLLWTCR - PLL Wait Control Register
    b7:b5    Reserved - The write value should be 0.
    b4:b0    PSTS     - PLL Waiting Time
                      - Wait time is 131072 cycles (approx. 681.6 us). */
    SYSTEM.PLLWTCR.BYTE = 0x0A;

    /* ---- Operate the PLL clock oscillator ---- */
    /* PLLCR2 - PLL Control Register 2 
    b7:b1    Reserved - The write value should be 0.
    b0       PLLEN    - PLL Stop Control - PLL is operating. */
    SYSTEM.PLLCR2.BYTE = 0x00;

    /* ---- Wait processing for the clock oscillation stabilization ---- */
    cmt0_wait((WAIT_TIME_FOR_PLL_OSCILLATION/FOR_CMT0_TIME)+1); /* oscillation stabilize (1.865 ms). */

}

/*******************************************************************************
* Outline      : Configure the HOCO clock oscillation
* Header       : r_init_clock.h
* Function Name: CGC_oscillation_HOCO
* Description  : Enable the HOCO and
*                Wait for HOCO stabilization wait time by a software.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void CGC_oscillation_HOCO(void)
{

    /* ---- Operate the HOCO clock ---- */
    /* HOCOCR - High-Speed On-Chip Oscillator Control Register
    b0       HCSTP    - the HOCO is operating. */
    SYSTEM.HOCOCR.BYTE = 0x00;

    /* ---- Wait processing for the clock oscillation stabilization ---- */
    cmt0_wait((WAIT_TIME_FOR_HOCO_OSCILLATION/FOR_CMT0_LOCO)+1);       /* LOCO is cmt0 count source. */
                                                                       /* oscillation stabilize (2.0 ms). */
}

/*******************************************************************************
* Outline      : Sub-clock pattern A
* Header       : r_init_clock.h
* Function Name: CGC_no_use_subclk
* Description  : Configure settings when the sub-clock is not used 
                 as the system clock or RTC count source.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void CGC_no_use_subclk(void)
{

    uint8_t i;
    volatile uint8_t dummy;

    /* ---- Operate the HOCO ---- */
    CGC_oscillation_HOCO();

    /* ---- Set clock source ---- */
    /* SCKCR3 - System Clock Control Register 3 
    b10:b8   CKSEL    - Clock Source Select - the HOCO is selected. */
    SYSTEM.SCKCR3.WORD = 0x0100;

    while (0x0100 != SYSTEM.SCKCR3.WORD) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Set RTC count source ---- */
    /* RCR4 - RTC Control Register 4 
    b7:b1    Reserved - The write value should be 0.
    b0       RCKSEL   - Count Source Select - RTC count source use the main clock */
    RTC.RCR4.BIT.RCKSEL = 1;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR4.BIT.RCKSEL;
    }

    /* ---- Wait for six the main-clock cycles  ---- */
    cmt0_wait((MAIN_CLOCK_CYCLE*6/FOR_CMT0_TIME)+1);

    /* ---- Setting of disable the sub-clock ---- */
    disable_subclk();

    /* ---- Initialization Procedure when the RTC is not to be Used
            (The main clock use RTC count source) ---- */
    disable_RTC_mainclk();

}

/*******************************************************************************
* Outline      : Sub-clock pattern B
* Header       : r_init_clock.h
* Function Name: CGC_disable_subclk_RTC_use_mainclk
* Description  : Configure setting When the sub-clock is not used 
                 and the RTC operates using the main clock.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void CGC_disable_subclk_RTC_use_mainclk(void)
{

    uint8_t i;
    volatile uint8_t dummy;

    /* ---- Operate the HOCO ---- */
    CGC_oscillation_HOCO();

    /* ---- Set clock source ---- */
    /* SCKCR3 - System Clock Control Register 3 
    b10:b8   CKSEL    - Clock Source Select - the HOCO is selected. */
    SYSTEM.SCKCR3.WORD = 0x0100;

    while (0x0100 != SYSTEM.SCKCR3.WORD) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Set RTC count source ---- */
    /* RCR4 - RTC Control Register 4 
    b0       RCKSEL   - Count Source Select - RTC count source use the main clock */
    RTC.RCR4.BIT.RCKSEL = 1;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR4.BIT.RCKSEL;
    }

    /* ---- Wait for six the main-clock cycles  ---- */
    cmt0_wait((MAIN_CLOCK_CYCLE*6/FOR_CMT0_TIME)+1);

    /* ---- Setting of not use the sub-clock  ---- */
    disable_subclk();

    /* ---- When using the RTC ---- */
    enable_RTC();

    /* ---- Resetting the wait control register by the main clock ---- */
    resetting_wtcr_mainclk();

}

/*******************************************************************************
* Outline      : Sub-clock pattern C
*                (When the sub-clock is used only as the system clock.)
* Header       : r_init_clock.h
* Function Name: CGC_subclk_as_sysclk
* Description  : Configure setting when the sub-clock is used as the system clock
                 and not used as theRTC count source.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void CGC_subclk_as_sysclk(void)
{

    uint8_t i;
    volatile uint8_t dummy;

    /* ---- setting of RTC count source ---- */
    /* RCR4 - RTC Control Register 4 
    b0       RCKSEL   - Count Source Select - RTC count source use the sub-clock */
    RTC.RCR4.BIT.RCKSEL = 0;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR4.BIT.RCKSEL;
    }

    /* ---- Setting of the sub-clock oscillation  ---- */
    oscillation_subclk();

    /* ---- Initialization Procedure when the Realtime Clock is not to be Used---- */
    disable_RTC_subclk();

}

/*******************************************************************************
* Outline      : Sub-clock pattern D
* Header       : r_init_clock.h
* Function Name: CGC_subclk_as_RTC
* Description  : Configure setting when the sub-clock is used 
                 as the RTC count source and not used as
the system clock.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void CGC_subclk_as_RTC(void)
{
    uint8_t i;
    volatile uint8_t dummy;

    /* ---- setting of RTC count source ---- */
    /* RCR4 - RTC Control Register 4 
    b0       RCKSEL   - Count Source Select - RTC count source use the sub-clock */
    RTC.RCR4.BIT.RCKSEL = 0;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR4.BIT.RCKSEL;
    }

    /* ---- Setting of the sub-clock oscillation ---- */
    oscillation_subclk();

    /* ---- When using the RTC ---- */
    enable_RTC();

    /* ---- Setting of the sub-clock do not use the system clock--- */
    no_use_subclk_as_sysclk();

}

/*******************************************************************************
* Outline      : Sub-clock pattern E
* Header       : r_init_clock.h
* Function Name: CGC_subclk_as_sysclk_RTC
* Description  : Configure setting when the sub-clock is used 
                 as both the system clock and RTC count source.
* Arguments    : none
* Return Value : none
*******************************************************************************/
void CGC_subclk_as_sysclk_RTC(void)
{

    uint8_t i;
    volatile uint8_t dummy;

    /* ---- setting of RTC count source ---- */
    /* RCR4 - RTC Control Register 4 
    b0       RCKSEL   - Count Source Select - RTC count source use the sub-clock */
    RTC.RCR4.BIT.RCKSEL = 0;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR4.BIT.RCKSEL;
    }

    /* ---- Setting of the sub-clock oscillation ---- */
    oscillation_subclk();

    /* ---- When using the RTC ---- */
    enable_RTC();

    /* ---- Resetting the wait control register by the sub-clock---- */
    resetting_wtcr_subclk();

}

/*******************************************************************************
* Outline      : Setting when not using the sub-clock
* Header       : none
* Function Name: disable_subclk
* Description  : Configure the setting when the sub-clock is not used
*                as the system clock or RTC count source.
* Arguments    : none
* Return Value : none
*******************************************************************************/
static void disable_subclk(void)
{

    uint8_t i;
    volatile uint8_t dummy;

    /* ---- Stop the sub-clock oscillator ---- */
    /* SOSCCR - Sub-Clock Oscillator Control Register
    b7:b1    Reserved - The write value should be 0.
    b0       SOSTP    - Sub-clock oscillator Stop - Sub-clock oscillator is stopped. */
    SYSTEM.SOSCCR.BYTE = 0x01;

    while (0x01 != SYSTEM.SOSCCR.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }
    /* RCR3 - RTC Control Register 3
    b7:b4    Reserved - The write value should be 0.
    b3:b1    RTCDV    - Sub-clock Oscillator Drive Ability Control. 
    b0       RTCEN    - Sub-clock oscillator is stopped. */
    RTC.RCR3.BIT.RTCEN = 0;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR3.BIT.RTCEN;
    }

    while (0 != RTC.RCR3.BIT.RTCEN) 
    {
        /* Confirm that the written value can be read correctly. */
    }

}

/*******************************************************************************
* Outline      : Configure sub-clock oscillation
* Header       : none
* Function Name: oscillation_subclk
* Description  : Configure sub-clock oscillation.
* Arguments    : none
* Return Value : none
*******************************************************************************/
static void oscillation_subclk(void)
{

    uint8_t i;
    volatile uint8_t dummy;

    /* ---- Stop the Sub-clock oscillator ---- */
    /* SOSCCR - Sub-Clock Oscillator Control Register
    b0       SOSTP    - Sub-clock oscillator Stop - Sub-clock oscillator is stopped. */
    SYSTEM.SOSCCR.BYTE = 0x01;

    while (0x01 != SYSTEM.SOSCCR.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* RCR3 - RTC Control Register 3
    b0       RTCEN    - Sub-clock oscillator is stopped. */
    RTC.RCR3.BIT.RTCEN = 0;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR3.BIT.RTCEN;
    }

    while (0 != RTC.RCR3.BIT.RTCEN) 
    {
        /* Confirm that the written */
    }

    /* ---- Wait for five sub-clock cycles ---- */
    cmt0_wait((SUB_CLOCK_CYCLE*5/FOR_CMT0_TIME)+1);        /* Wait time is 5 sub-clock cycles (approx. 152 us). */

#ifndef LOW_CL
    /* ---- Setting of the sub-clock drive strength ---- */
    /* RCR3 - RTC Control Register 3
    b3:b1    RTCDV    - Sub-clock Oscillator Drive Ability Control - Drive ability for standard CL. 
    b0       RTCEN    - Sub-clock oscillator is stopped. */
    RTC.RCR3.BYTE = 0x0C;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR3.BYTE;
    }

    while (0x0C != RTC.RCR3.BYTE) 
    {
        /* Confirm that the written */
    }

#else 
    /* ---- Setting of the sub-clock drive strength ---- */
    /* RCR3 - RTC Control Register 3
    b3:b1    RTCDV    - Sub-clock Oscillator Drive Ability Control - Drive ability for standard CL. 
    b0       RTCEN    - Sub-clock oscillator is stopped. */
    RTC.RCR3.BYTE = 0x02;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR3.BYTE;
    }

    while (0x02 != RTC.RCR3.BYTE) 
    {
        /* Confirm that the written */
    }

#endif 

    /* ---- Set wait time until the sub-clock oscillator stabilizes ---- */
    /* SOSCWTCR - Sub-Clock Oscillator Wait Control Register
    b7:b5    Reserved - The write value should be 0.
    b4:b0    SSTS - Sub-Clock Oscillator Waiting Time - Waiting time is 2 cycles. */
    SYSTEM.SOSCWTCR.BIT.SSTS = 0;            /* Wait time is 2 sub-clock cycles (approx. 61 us). */

    /* ---- Operate the Sub-clock oscillator ---- */
    /* SOSCCR - Sub-Clock Oscillator Control Register
    b0       SOSTP    - Sub-clock oscillator Stop - Sub-clock oscillator is running. */
    SYSTEM.SOSCCR.BYTE = 0x00;

    while (0x00 != SYSTEM.SOSCCR.BYTE) 
    {
        /* Confirm that the written */
    }

    /* ---- Wait processing for the clock oscillation stabilization ---- */
    cmt0_wait(WAIT_TIME_FOR_SUB_OSCILLATION/FOR_CMT0_TIME+1);      /* oscillation stabilize (2.6s). */

}

/*******************************************************************************
* Outline      : Processing when not using the sub-clock as the system clock
* Header       : none
* Function Name: no_use_subclk_as_sysclk
* Description  : Set the SOSTP bit to 1 (sub-clock stops)
*                when the sub-clock is used only as the RTC count source.
* Arguments    : none
* Return Value : none
*******************************************************************************/
static void no_use_subclk_as_sysclk(void)
{

    /* ---- Stop the Sub-clock oscillator ---- */
    /* SOSCCR - Sub-Clock Oscillator Control Register
    b0       SOSTP    - Sub-clock oscillator Stop - Sub-clock oscillator is stopped. */
    SYSTEM.SOSCCR.BYTE = 0x01;

    while (0x01 != SYSTEM.SOSCCR.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }

}

/*******************************************************************************
* Outline      : Set the wait control register
*                (the main clock is used as the RTC count source.)
* Header       : none
* Function Name: resetting_wtcr_mainclk
* Description  : Set the wait control register when exiting 
*                from the software standby mode.
* Arguments    : none
* Return Value : none
*******************************************************************************/
static void resetting_wtcr_mainclk(void)
{

    /* ---- Stop the main clock oscillator ---- */
    /* MOFCR    - Main Clock Oscillator Forced Oscillation Control Register
    b7:b1    Reserved - The write value should be 0.
    b0       MOFXIN   - Main Clock Oscillator Forced Oscillation
                      - The main clock oscillator is forcedly oscillated. */
    SYSTEM.MOFCR.BIT.MOFXIN = 1;

    while (1 != SYSTEM.MOFCR.BIT.MOFXIN) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Operate the main clock oscillator ---- */
    /* MOSCCR   - Main Clock Oscillator Control Register
    b0       MOSTP    - Main Clock Oscillator Stop - Main clock oscillator is stopped. */
    SYSTEM.MOSCCR.BYTE = 0x01;

    while (0x01 != SYSTEM.MOSCCR.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Wait for five main clock cycles  ---- */
    cmt0_wait((MAIN_CLOCK_CYCLE*5/FOR_CMT0_TIME)+1);        /* Wait time is five main clock cycles (approx. 416 ns). */

    /* ---- Reset wait control register ---- */
    /* MOSCWTCR - Main Clock Oscillator Wait Control Register
    b4:b0    MSTS     - Main Clock Oscillator Waiting Time - Wait time is two cycles (approx. 166 ns). */
    SYSTEM.MOSCWTCR.BYTE = 0x00;

    /* ---- Operate the main clock oscillator ---- */
    /* MOSCCR   - Main Clock Oscillator Control Register
    b0       MOSTP    - Main Clock Oscillator Stop - Main clock oscillator is running. */
    SYSTEM.MOSCCR.BYTE = 0x00;

    while (0x00 != SYSTEM.MOSCCR.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Wait for two main clock cycled ---- */
    cmt0_wait((MAIN_CLOCK_CYCLE*2/FOR_CMT0_TIME)+1);          /* Wait time is two main clock cycles (approx. 166 ns). */

}

/*******************************************************************************
* Outline      : Set the wait control register
*                (the sub-clock is used as the RTC count source.)
* Header       : none
* Function Name: resetting_wtcr_subclk
* Description  : Set the wait control register when exiting 
*                from the software standby mode.
* Arguments    : none
* Return Value : none
*******************************************************************************/
static void resetting_wtcr_subclk(void)
{

    /* ---- Stop the Sub-clock oscillator ---- */
    /* SOSCCR - Sub-Clock Oscillator Control Register
    b0       SOSTP    - Sub-clock oscillator Stop - Sub-clock oscillator is stopped. */
    SYSTEM.SOSCCR.BYTE = 0x01;

    while (0x01 != SYSTEM.SOSCCR.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Wait for five sub-clock cycles ---- */
    cmt0_wait((SUB_CLOCK_CYCLE*5/FOR_CMT0_TIME)+1);       /* Wait time is five sub-clock cycles (approx. 152 us).*/

    /* ---- Set wait time until the sub-clock oscillator stabilizes ---- */
    /* SOSCWTCR - Sub-Clock Oscillator Wait Control Register
    b4:b0    SSTS     - Sub-Clock Oscillator Waiting Time - Waiting time is 2 cycles. */
    SYSTEM.SOSCWTCR.BYTE = 0x00;            /* Wait time is two sub-clock cycles (approx. 61 us).*/

    /* ---- Operate the sub-clock oscillator ---- */
    /* SOSCCR - Sub-Clock Oscillator Control Register
    b0       SOSTP    - Sub-clock oscillator Stop - Sub-clock oscillator is running. */
    SYSTEM.SOSCCR.BYTE = 0x00;              /* Sub-clock oscillator is operating. */

    while (0x00 != SYSTEM.SOSCCR.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Wait processing for the clock oscillation stabilization ---- */
    cmt0_wait((SUB_CLOCK_CYCLE*2/FOR_CMT0_TIME)+1 );    /* Wait time is two sub-clock cycles (approx. 61 us).*/

}

/*******************************************************************************
* Outline      : Initialization when using the RTC
* Header       : none
* Function Name: enable_RTC
* Description  : Configure the initialization when using the RTC.
* Arguments    : none
* Return Value : none
*******************************************************************************/
static void enable_RTC(void)
{

#if (SELECT_SUB == PATTERN_D) || (SELECT_SUB == PATTERN_E) 

    uint8_t i;
    volatile uint8_t dummy;

    /* ---- Set RCR3 register ---- */
    /* RCR3 - RTC Control Register 3
    b3:b1    RTCDV    - Sub-clock Oscillator Drive Ability Control
    b0       RTCEN    - Sub-clock oscillator is running. */
    RTC.RCR3.BIT.RTCEN = 1;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
         dummy = RTC.RCR3.BIT.RTCEN;
    }

    while (1 != RTC.RCR3.BIT.RTCEN) 
    {
         /* Confirm that the written value can be read correctly. */
    }

#endif 
    /* ---- Stop prescaler and counter ---- */
    /* RCR2 - RTC Control Register 2
    b7       Reserved - The write value should be 0.
    b6       HR24     - Hours Mode Select.
    b5       AADJP    - Automatic Adjustment Period Select
    b4       AADJE    - Automatic Adjustment Enable
    b3       RTCOE    - RTCOUT Output Enable
    b2       ADJ30    - 30-Second Adjustment
    b1       RESET    - RTC Software Reset
    b0       START    - start - Prescaler is stopped. */
    RTC.RCR2.BYTE &= 0x7E;

    while (0 != RTC.RCR2.BIT.START) 
    {
        /* Confirm that the written value can be read correctly. */
    }

#if (SELECT_SUB == PATTERN_B) 
     /* ---- Set the frequency register H/L ---- */
     /* Set the RFRH register */
     RTC.RFRH.WORD = (uint16_t)(((MAIN_CLOCK_Hz/128)-1) >> 16);
     /* Set the RFRL register */
     RTC.RFRL.WORD = (uint16_t)(((MAIN_CLOCK_Hz/128)-1) & 0x0000FFFF);
#endif 

    /* ---- RTC Software Reset ---- */
    /* RCR2 - RTC Control Register 2
    b1       RESET    - RTC Software Reset
                      - The prescaler and target registers are reset by RTC software reset.*/
    RTC.RCR2.BIT.RESET = 1;

    while (0 != RTC.RCR2.BIT.RESET) 
    {
        /* Confirm that the written value can be read correctly. */
    }

#if (SELECT_SUB == PATTERN_D) || (SELECT_SUB == PATTERN_E) 
#ifndef LOW_CL 
    /* ---- Set RCR3 register ---- */
    /* RCR3 - RTC Control Register 3
    b3:b1    RTCDV    - Sub-clock Oscillator Drive Ability Control - Drive ability for standard CL. 
    b0       RTCEN    - Sub-clock oscillator is running. */
    RTC.RCR3.BYTE = 0x0D;

    for (i = 0; i < 3; i++) 
    {
         dummy = RTC.RCR3.BYTE;
    }

    while (0x0D != RTC.RCR3.BYTE) 
    {
         /* Confirm that the written value can be read correctly. */
    }

#else 
    /* ---- Set RCR3 register ---- */
    /* RCR3 - RTC Control Register 3
    b3:b1    RTCDV    - Sub-clock Oscillator Drive Ability Control - Drive ability for standard CL. 
    b0       RTCEN    - Sub-clock oscillator is running. */
    RTC.RCR3.BYTE = 0x03;

    for (i = 0; i < 3; i++) 
    {
         dummy = RTC.RCR3.BYTE;
    }

    while (0x03 != RTC.RCR3.BYTE) 
    {
         /* Confirm that the written value can be read correctly. */
    }

#endif 
#endif 

}

/*******************************************************************************
* Outline      : Initialization when not using the RTC
* Header       : none
* Function Name: disable_RTC_mainclk
* Description  : Configure the initialization when not using the RTC.
* Arguments    : none
* Return Value : none
*******************************************************************************/
static void disable_RTC_mainclk(void)
{

    /* ---- Stop prescaler and counter ---- */
    /* RCR2 - RTC Control Register 2
    b0       START    - start - Prescaler is stopped. */
    RTC.RCR2.BYTE &= 0x7E;

    while (0 != RTC.RCR2.BIT.START) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- RTC Software Reset ---- */
    /* RCR2 - RTC Control Register 2
    b1       RESET    - RTC Software Reset
                      - The prescaler and target registers are reset by RTC software reset.*/
    RTC.RCR2.BIT.RESET = 1;

    while (0 != RTC.RCR2.BIT.RESET) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Disabled interrupt request ---- */
    /* RCR1 - RTC Control Register 1
    b7:b4    PES      - Periodic Interrupt Select.
    b3       Reserved - The write value should be 0.
    b2       PIE      - Periodic Interrupt Enable - A periodic interrupt request is disabled.
    b1       CIE      - Carry Interrupt Enable - A carry interrupt request is disabled.
    b0       AIE      - Alarm Interrupt Enable - An alarm interrupt request is disabled.*/
    RTC.RCR1.BYTE = 0x00;

    while (0x00 != RTC.RCR1.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- IR flag of RTC is Clear ---- */
    IR(RTC, ALM) = 0;
    IR(RTC, CUP) = 0;
    IR(RTC, PRD) = 0;

}

/*******************************************************************************
* Outline      : Initialization when not using the RTC
* Header       : none
* Function Name: disable_RTC_subclk
* Description  : Configure the initialization when not using the RTC.
* Arguments    : none
* Return Value : none
*******************************************************************************/
static void disable_RTC_subclk(void)
{

    uint8_t i;
    volatile uint8_t dummy;

    /* RCR3 - RTC Control Register 3
    b3:b1    RTCDV    - Sub-clock Oscillator Drive Ability Control.
    b0       RTCEN    - Sub-clock oscillator is operating.*/
    RTC.RCR3.BIT.RTCEN = 1;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR3.BIT.RTCEN;
    }

    while (1 != RTC.RCR3.BIT.RTCEN) 
    {
        /* Confirm that the written */
    }

    /* ---- Stop prescaler and counter ---- */
    /* RCR2 - RTC Control Register 2
    b0       START    - start - Prescaler is stopped. */
    RTC.RCR2.BYTE &= 0x7E;

    while (0 != RTC.RCR2.BIT.START) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- RTC Software Reset ---- */
    /* RCR2 - RTC Control Register 2
    b1       RESET    - RTC Software Reset
                      - The prescaler and target registers are reset by RTC software reset.*/
    RTC.RCR2.BIT.RESET = 1;

    while (0 != RTC.RCR2.BIT.RESET) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- Disabled interrupt request ---- */
    /* RCR1 - RTC Control Register 1
    b7:b4    PES      - Periodic Interrupt Select.
    b2       PIE      - Periodic Interrupt Enable - A periodic interrupt request is disabled.
    b1       CIE      - Carry Interrupt Enable - A carry interrupt request is disabled.
    b0       AIE      - Alarm Interrupt Enable - An alarm interrupt request is disabled.*/
    RTC.RCR1.BYTE = 0x00;

    while (0x00 != RTC.RCR1.BYTE) 
    {
        /* Confirm that the written value can be read correctly. */
    }

    /* ---- IR flag of RTC is Clear ---- */
    IR(RTC, ALM) = 0;
    IR(RTC, CUP) = 0;
    IR(RTC, PRD) = 0;

#ifndef LOW_CL 
    /* RCR3 - RTC Control Register 3
    b3:b1    RTCDV    - Sub-clock Oscillator Drive Ability Control - Drive ability for standard CL. 
    b0       RTCEN    - Sub-clock oscillator is stopped.*/
    RTC.RCR3.BYTE = 0x0C;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR3.BYTE;
    }

    while (0x0C != RTC.RCR3.BYTE) 
    {
        /* Confirm that the written */
    }

#else 
    /* RCR3 - RTC Control Register 3
    b3:b1    RTCDV    - Sub-clock Oscillator Drive Ability Control - Drive ability for standard CL. 
    b0       RTCEN    - Sub-clock oscillator is stopped.*/
    RTC.RCR3.BYTE = 0x02;

    /* dummy read three times */
    for (i = 0; i < 3; i++) 
    {
        dummy = RTC.RCR3.BYTE;
    }

    while (0x02 != RTC.RCR3.BYTE) 
    {
        /* Confirm that the written */
    }

#endif 
}

/*******************************************************************************
* Outline      : Software wait with CMT0
* Header       : none
* Function Name: cmt0_wait
* Description  : This function is to wait for the oscillation stabilization wait time.
* Arguments    : cnt : Wait for the time specified by an argument multiplied 
*                      by FOR_CMT0_TIME (us).
* Return Value : none
* Note         : This processing assumes that no other interrupts occur during the operation.
*******************************************************************************/
static void cmt0_wait(uint32_t cnt)
{

    uint32_t cmt_int_cnt;
    unsigned long backup_MSTP;
    unsigned short backup_PRCR;

    backup_MSTP = SYSTEM.MSTPCRA.LONG;
    backup_PRCR = SYSTEM.PRCR.WORD | 0xA500;

    /* ---- Enable write protection ---- */
    /* PRCR - Protect Register 
    b15:b8   PRKEY    - PRC Key Code - A5h 
                        (The write value should be A5h to permission writing PRCi bit)
    b7:b4    Reserved - The write value should be 0.
    b3       PRC3     - Protect Bit 3 - Write disabled
    b2       Reserved - The write value should be 0.
    b1       PRC1     - Protect Bit 1 - Write enabled
    b0       PRC0     - Protect Bit 0 - Write enabled */
    SYSTEM.PRCR.WORD = 0xA502;

    cmt_int_cnt = 0;

    /* ---- Exit the module stop state ---- */
    MSTP(CMT0) = 0;                         /* CMT0 and CMT1 exit the module stop state. */

    /* ---- Stop CMT0 counting ---- */
    /* CMSTR0 - Compare Match Timer Start Register 0
    b15:b2   Reserved - The write value should be 0.
    b1       STR1     - Count Start 1 - CMT1.CMCNT count is stopped.
    b0       STR0     - Count Start 0 - CMT0.CMCNT count is stopped. */
    CMT.CMSTR0.BIT.STR0 = 0;                /* CMT0 count is stopped. */

    /* ---- Set the CMT0 count source and Enable the CMT0 compare match interrupt ---- */
    /* CMCR - Compare Match Timer Control Register
    b15:b8   Reserved - The write value should be 0.
    b7       Reserved - The write value should be 1.
    b6       CMIE     - Compare Match Interrupt Enable - Compare match interrupt enable.
    b5:b2    Reserved - The write value should be 0.
    b1:b0    CKS      - Clock Select - PCLK/32 */
    CMT0.CMCR.WORD = 0x00C1;

    /* ---- Clear the CMT0 count ---- */
    /* CMCNT - Compare Match Timer Counter */
    CMT0.CMCNT = 0x0000;

    /* ---- Wait time set is other than 0? ---- */
    if (0 != cnt) 
    {
        /* ---- Decrement the wait time  & wait counter ---- */
        cnt = cnt - 1;
        cmt_int_cnt = ((cnt & 0xFFFF0000) >> 16) + 1;
        cnt = cnt & 0x0000FFFF;

    }

    /* ---- Set the CMT0 wait time ---- */
    /* CMCOR - Compare Match Timer Constant Register */
    CMT0.CMCOR = (unsigned short)cnt;

    /* ---- Clear the CMT0 interrupt request ---- */
    while (1 == IR(CMT0,CMI0)) 
    {
        IR(CMT0,CMI0) = 0;                  /* CMT0.CMI0 interrupt request is cleared. */
    }

    /* ---- Start CMT0 counting ---- */
    /* CMSTR0 - Compare Match Timer Start Register 0
    b0       STR0     - Count Start 0 - CMT0.CMCNT count is started. */
    CMT.CMSTR0.BIT.STR0 = 1;

    for ( ; cmt_int_cnt > 0; cmt_int_cnt--) 
    {

        /* ---- CMI0 interrupt request generated? ---- */
        while (0 == IR(CMT0,CMI0)) 
        {
            
        }

        /* CMCOR - Compare Match Timer Constant Register */
        CMT0.CMCOR = 0xFFFF;

        /* Clear IR flag */
        IR(CMT0,CMI0) = 0;

        /* ---- Set the CMT0 count source and Enable the CMT0 compare match interrupt ---- */
        /* CMCR - Compare Match Timer Control Register
        b6       CMIE     - Compare Match Interrupt Enable - Compare match interrupt enable.
        b1:b0    CKS      - Clock Select - PCLK/32 */
        CMT0.CMCR.WORD = 0x00C1;

    }

    /* ---- Stop CMT0 counting ---- */
    /* CMSTR0 - Compare Match Timer Start Register 0
    b0      STR0      - Count Start 0 - CMT0.CMCNT count is stopped. */
    CMT.CMSTR0.BIT.STR0 = 0;

    /* ---- Initialize CMT0 ---- */
    /* CMCR - Compare Match Timer Control Register
    b6       CMIE     - Compare Match Interrupt Enable - Compare match interrupt disable.
    b1:b0    CKS      - Clock Select - PCLK/8 */
    CMT0.CMCR.WORD = 0x0080;

    /* CMCNT - Compare Match Timer Counter */
    CMT0.CMCNT = 0x0000;

    /* CMCOR - Compare Match Timer Constant Register */
    CMT0.CMCOR = 0x0000;

    /* Clear IR flag */
    IR(CMT0,CMI0) = 0;

    /* ----Transfer the module stop state ---- */
    MSTP(CMT0) = 1;

    SYSTEM.MSTPCRA.LONG = backup_MSTP;

    /* ---- Disable write protection ---- */
    /* PRCR - Protect Register 
    b15:b8   PRKEY    - PRC Key Code - A5h 
                        (The write value should be A5h to permission writing PRCi bit)
    b1       PRC1     - Protect Bit 1 - Write disabled
    b0       PRC0     - Protect Bit 0 - Write disabled */
    SYSTEM.PRCR.WORD = backup_PRCR;

}

/* End of File */

