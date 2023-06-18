/*
 * File:   newmain.c
 * Author: nemet
 *
 * Created on June 9, 2023, 1:17 PM
 */

// PIC18F26K83 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = HS     // External Oscillator Selection (HS (crystal oscillator) above 8 MHz; PFM set to high power)
#pragma config RSTOSC = EXTOSC// Reset Oscillator Selection (EXTOSC with 4x PLL, with EXTOSC operating per FEXTOSC bits)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = OFF     // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be set and cleared repeatedly)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = OFF     // Multi-vector enable bit (Interrupt contoller does not use vector table to prioritze interrupts)
#pragma config IVT1WAY = OFF    // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set repeatedly)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write-protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// CONFIG5H

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include <xc.h>


#define _XTAL_FREQ 20000000
char own_add = 10;
 char own_add;
 char lst_add;
 char rcl_add;
 char can_add;
 char can_dat;
 char can_idat;
 char can_iedat;
 char can_ilen;
 char can_itxadd;
 char can_irxadd;
 char call_add;
 char t_cnt;			// table counter
 char tbl_add;			// table address
 char tbl_dat;			// table data

void main(void) {
    
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC0 = 0;
    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB2 = 0;
    TRISCbits.TRISC4 = 0;
    RB2PPS = 0b110011;   //RB2->ECAN:CANTX1;  
    RC4PPS = 0b110100;   //Rc4->ECAN:CANTX0;
    CANRXPPS = 0x0B;   //RB3->ECAN:CANRX;  
    ANSELBbits.ANSELB2 = 0;
    ANSELBbits.ANSELB3 = 0;
    ANSELCbits.ANSELC4 = 0;
 

    __delay_ms(2000);
    CANCON = 0x80;
    while (0x80 != (CANSTAT & 0xE0)); // wait until ECAN is in config mode
    
    ECANCON = 0x00;

    CIOCON = 0x01;

    RXM0EIDH = 0x00;
    RXM0EIDL = 0x00;
    RXM0SIDH = 0x00;
    RXM0SIDL = 0x00;
    RXM1EIDH = 0x00;
    RXM1EIDL = 0x00;
    RXM1SIDH = 0x00;
    RXM1SIDL = 0x00;
 
    /**
    Initialize Receive Filters
    */   
    RXF0EIDH = 0x00;
    RXF0EIDL = 0x00;
    RXF0SIDH = 0x00;
    RXF0SIDL = 0x00;
    RXF1EIDH = 0x00;
    RXF1EIDL = 0x00;
    RXF1SIDH = 0x00;
    RXF1SIDL = 0x00;
    RXF2EIDH = 0x00;
    RXF2EIDL = 0x00;
    RXF2SIDH = 0x00;
    RXF2SIDL = 0x00;
    RXF3EIDH = 0x00;
    RXF3EIDL = 0x00;
    RXF3SIDH = 0x00;
    RXF3SIDL = 0x00;
    RXF4EIDH = 0x00;
    RXF4EIDL = 0x00;
    RXF4SIDH = 0x00;
    RXF4SIDL = 0x00;
    RXF5EIDH = 0x00;
    RXF5EIDL = 0x00;
    RXF5SIDH = 0x00;
    RXF5SIDL = 0x00;


    BRGCON1 = 0x3F;
    BRGCON2 = 0xF1;
    BRGCON3 = 0x05;
    CIOCONbits.CLKSEL = 0;
    ECANCONbits.MDSEL = 0b00;
    CANCON = 0x00;
    while (0x00 != (CANSTAT & 0xE0)); // wait until ECAN is in Normal mode

    CANCONbits.REQOP = 0;
    while (0x00 != (CANSTAT & 0xE0)); // wait until ECAN is in Normal mode
    CANCON = 0b00001000;
    
    if (TXB0CONbits.TXREQ != 1) 
    {
        TXB0EIDH = 0x00;
    TXB0EIDL = 0x00;
    
    //0x35E    0110 1011 110
    TXB0SIDH = 0x0;
    TXB0SIDL = 0x0;
        TXB0DLC  = 3;
        TXB0D0   = 2;
        TXB0D1   = 0;
        TXB0D2   = 1;
        

        TXB0CONbits.TXREQ = 1; //Set the buffer to transmit		
        
    } 
    while(1)
    {
        if (RXB0CONbits.RXFUL != 0) //CheckRXB0
    {
       
    }
    if (RXB1CONbits.RXFUL != 0) //CheckRXB1
    {
        
    }
        LATCbits.LATC0 ^= 1; 
        __delay_ms(10);
    }
    return;
}