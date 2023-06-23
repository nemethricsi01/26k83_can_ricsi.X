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
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

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
#define _XTAL_FREQ 64000000
struct CAN_RXBUFF
   {
    uint8_t idh;
    uint8_t idl;
    uint8_t dl;
    uint8_t d0;
    uint8_t d1;
    uint8_t d2;
    uint8_t d3;
    uint8_t d4;
   };
struct CAN_RXBUFF can_rxbuff;

void can_init(void)
{
    RB5PPS = 0b110011;//RB5->ECAN:CANTX1;  
    CANRXPPS = 0x0A;   //RB2->ECAN:CANRX;  
    
    ANSELBbits.ANSELB2 = 0;
    ANSELBbits.ANSELB5 = 0;
    CANCONbits.REQOP = 0b100;//request config mode
    while (0x80 != (CANSTAT & 0xE0)); // wait until ECAN is in config mode
    ECANCON = 0x00;
    RXM0EIDH = 0x00;
    RXM0EIDL = 0x00;
    RXM0SIDH = 0x00;
    RXM0SIDL = 0x00;
    RXM1EIDH = 0x00;
    RXM1EIDL = 0x00;
    RXM1SIDH = 0x00;
    RXM1SIDL = 0x00;  
    RXF0EIDH = 0x00;
    RXF0EIDL = 0x00;
    RXF0SIDH = 0x00;
    RXF0SIDL = 0x00;
    RXF1EIDH = 0x00;
    RXF1EIDL = 0x00;
    RXF1SIDH = 0x00;
    RXF1SIDL = 0x00;
    BRGCON1 = 0x3F;
    BRGCON2 = 0xF1;
    BRGCON3 = 0x05;
    CIOCONbits.CLKSEL = 0;
    ECANCONbits.MDSEL = 0b00;
    CANCONbits.REQOP = 0;//request normal mode
    while (0x00 != (CANSTAT & 0xE0)); // wait until ECAN is in Normal mode
}
//void i2c_start(void){
//    PIR1bits.SSPIF = 0;
//    SSPCON2bits.SEN = 1;
//    while(PIR1bits.SSPIF != 1);
//    PIR1bits.SSPIF = 0;
//}
//void i2c_stop(void){
//    PIR1bits.SSPIF = 0;
//    SSPCON2bits.PEN = 1;
//    while(PIR1bits.SSPIF != 1);
//    PIR1bits.SSPIF = 0;
//}

void disp_write_command(uint8_t command)
{
    I2C1ADB1 = 0x78;                   // Load address with write = 0
    I2C1TXB = 0x0;                 
    I2C1CNT = 2;                       // Load with size of array to write
    I2C1CON0bits.S = 1;
    while(I2C1STAT1bits.TXBE != 0);
    while(I2C1CON0bits.MDR != 1);
    I2C1TXB = command;
}
void disp_write_data(uint8_t data)
{
    I2C1ADB1 = 0x78;                   // Load address with write = 0
    I2C1TXB = 0x40;                 
    I2C1CNT = 2;                       // Load with size of array to write
    I2C1CON0bits.S = 1;
    while(I2C1STAT1bits.TXBE != 0);
    while(I2C1CON0bits.MDR != 1);
    I2C1TXB = data;
}
void i2c_init(void)
{
    TRISCbits.TRISC3 = 0;//scl
    TRISCbits.TRISC2 = 0;//sda
    ANSELCbits.ANSELC2 = 0;
    ANSELCbits.ANSELC3 = 0;
    I2C1SDAPPS = 0b00010010;//C2
    I2C1SCLPPS = 0b00010011;//C3
    RC2PPS = 0b100010;//sda
    RC3PPS = 0b100001;//scl
    WPUCbits.WPUC2 = 1;
    WPUCbits.WPUC3 = 1;
    ODCONC = 0xc;//FONTOS
    
    I2C1CON0bits.MODE = 0b100;//master mode 7 bit address
    I2C1CLKbits.CLK = 0b0011;//mfintosc
//    I2C1CON2bits.ACNT = 1;
    I2C1CON0bits.EN = 1;
}
void main(void) {

    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB5 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC1 = 0;

    ANSELCbits.ANSELC4 = 0;
    __delay_ms(2000);
    PIE5bits.RXB0IE = 1;
    INTCON0bits.GIE = 1;
    INTCON0bits.GIEH = 1;
    
    can_init();
    i2c_init();
    
    LATCbits.LATC1 = 0;
    __delay_ms(50);
    LATCbits.LATC1 = 1;
    __delay_ms(50);
    LATCbits.LATC1 = 0;
    __delay_ms(50);
    LATCbits.LATC1 = 1;
    __delay_ms(50);
    disp_write_command(0b00111010);//function set
	  __delay_ms(1);
	  disp_write_command(0b00001001);//extended function set
	  __delay_ms(1);
	  disp_write_command(0b00000110);//entry mode set
	  __delay_ms(1);
	  disp_write_command(0b00011110);//bias setting
	  __delay_ms(1);
	  disp_write_command(0b00111001);//function set
	  __delay_ms(1);
	  disp_write_command(0b00011011);//internal osc
	  __delay_ms(1);
	  disp_write_command(0b01101110);//follower control
	  __delay_ms(1);
	  disp_write_command(0b01010111);//power control
	  __delay_ms(1);
	  disp_write_command(0b01110010);//contrast set
	  __delay_ms(1);
	  disp_write_command(0b00111000);//function set
	  __delay_ms(1);
	  disp_write_command(0b00001111);//display on
      __delay_ms(1);
      disp_write_command(0b00000001);//display clr
	  __delay_ms(200);
      disp_write_command(0b00000010);//display home
	  __delay_ms(200);
      
	  disp_write_data('A');
      __delay_ms(1);
	  disp_write_data('B');
      __delay_ms(1);
	  disp_write_data('C');
      __delay_ms(1);
	  disp_write_data('D');
      __delay_ms(1);
	  disp_write_data('E');
    
    
    if (TXB0CONbits.TXREQ != 1) 
    {
        TXB0SIDL = 1;
        TXB0SIDH = 0;
        TXB0DLC  = 5;
        TXB0D0   = 2;
        TXB0D1   = 0;
        TXB0D2   = 1;
        

        TXB0CONbits.TXREQ = 1; //Set the buffer to transmit		
        
    } 
    while(1)
    {
        LATBbits.LATB0 ^= 1;
        __delay_ms(1000);
    }
    return;
}
void __interrupt() myISR(void)
{
// *** CAN ***
	if (PIR5bits.RXB0IF && PIE5bits.RXB0IE)		// CAN controller interrupt
		{
		PIR5bits.RXB0IF = 0;

		can_rxbuff.d0	= RXB0D0;		// read data
		can_rxbuff.d1	= RXB0D1;		// read data
		can_rxbuff.d2	= RXB0D2;		// read data
		can_rxbuff.d3	= RXB0D3;		// read data
		can_rxbuff.dl	= RXB0DLC;
        can_rxbuff.idh  = RXB0SIDH;
        can_rxbuff.idl  = RXB0SIDL;
        RXB0CONbits.RXFUL = 0;
		}
}