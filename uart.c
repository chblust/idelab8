/*
 * File:        uart.c
 * Purpose:     Provide UART routines for serial IO
 *
 * Notes:        
 *
 */


#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)

#include "uart.h"  // you need to create uart.h

void uart0_init(void)
{
    //define variables for baud rate and baud rate fine adjust
    uint16_t ubd, brfa;

    //Enable clock for UART using a System Clock Gating Control Register x
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
    // Enable Clock on PORTx
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    

    //Configure the port control register to alternative 3 (which is UART mode for K64)
    PORTB_PCR16 = PORT_PCR_MUX(3);
    PORTB_PCR17 = PORT_PCR_MUX(3);
    // Configure the UART for establishing serial communication using System Options Register
    SIM_SOPT5 |= (SIM_SOPT5_UART0TXSRC_MASK | SIM_SOPT5_UART0RXSRC_MASK);

    //Disable transmitter and receiver until proper settings are chosen for the UART module
    UART0_C2 &= ~UART_C2_RE_MASK;
    UART0_C2 &= ~UART_C2_TE_MASK;

    //Select default transmission/reception settings for serial communication of UART by clearing the control register 1
    UART0_C1 = 0;

    //UART Baud rate is calculated by: baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
    //13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH 
    //BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
    //BRFA is defined by the lower 4 bits of control register, UART0_C4 

    //calculate baud rate settings: ubd = UART module clock/16* baud rate
    ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));  

    //clear SBR bits of BDH
    UART0_BDH = 0;

    //distribute this ubd in BDH and BDL
    UART0_BDH |= (ubd & 0x1F00) >> 8;
    UART0_BDL |= (ubd & 0xFF);

    //BRFD = (1/32)*BRFA 
    //make the baud rate closer to the desired value by using BRFA
    brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

    //write the value of brfa in UART0_C4
    UART0_C4 |= brfa;
        
    //Enable transmitter and receiver of UART
    UART0_C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK);

}

uint8_t uart0_getchar(void)
{
// Wait until there is space for more data in the receiver buffer
    while(!(UART0_S1 & UART_S1_RDRF_MASK));

// Return the 8-bit data from the receiver 
    return UART0_D;
}

void uart0_putchar(char ch)
{
// Wait until transmission of previous bit is complete 
    while(!(UART0_S1 & UART_S1_TDRE_MASK));
    
//  Send the character 
    UART0_D = ch;
}

void uart0_put(char *ptr_str)
{
    while(*ptr_str)
        uart0_putchar(*ptr_str++);
}

void uart3_init(void)
{
    //define variables for baud rate and baud rate fine adjust
    uint16_t ubd, brfa;

    //Enable clock for UART using a System Clock Gating Control Register x
    SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
    // Enable Clock on PORTx
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    

    //Configure the port control register to alternative 3 (which is UART mode for K64)
    PORTB_PCR10 = PORT_PCR_MUX(3);
    PORTB_PCR11 = PORT_PCR_MUX(3);
    // Configure the UART for establishing serial communication using System Options Register
    //SIM_SOPT5 |= (SIM_SOPT5_UART3TXSRC_MASK | SIM_SOPT5_UART3RXSRC_MASK);

    //Disable transmitter and receiver until proper settings are chosen for the UART module
    UART3_C2 &= ~UART_C2_RE_MASK;
    UART3_C2 &= ~UART_C2_TE_MASK;

    //Select default transmission/reception settings for serial communication of UART by clearing the control register 1
    UART3_C1 = 0;

    //UART Baud rate is calculated by: baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
    //13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH 
    //BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
    //BRFA is defined by the lower 4 bits of control register, UART0_C4 

    //calculate baud rate settings: ubd = UART module clock/16* baud rate
    ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));  

    //clear SBR bits of BDH
    UART3_BDH = 0;

    //distribute this ubd in BDH and BDL
    UART3_BDH |= (ubd & 0x1F00) >> 8;
    UART3_BDL |= (ubd & 0xFF);

    //BRFD = (1/32)*BRFA 
    //make the baud rate closer to the desired value by using BRFA
    brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

    //write the value of brfa in UART0_C4
    UART3_C4 |= brfa;
        
    //Enable transmitter and receiver of UART
    UART3_C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK);

}

uint8_t uart3_getchar(void)
{
// Wait until there is space for more data in the receiver buffer
    while(!(UART3_S1 & UART_S1_RDRF_MASK));

// Return the 8-bit data from the receiver 
    return UART3_D;
}

void uart3_putchar(char ch)
{
// Wait until transmission of previous bit is complete 
    while(!(UART3_S1 & UART_S1_TDRE_MASK));
    
//  Send the character 
    UART3_D = ch;
}

void uart3_put(char *ptr_str)
{
    while(*ptr_str)
        uart3_putchar(*ptr_str++);
}

