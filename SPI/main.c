/*
 * File:   main.c
 * Author: nikit
 *
 * Created on 10 aprile 2024, 15.36
 */


#include "xc.h"
#define baud_rate 9600
#define ubrg_value ((72000000 / 16) / baud_rate) - 1

void spi_write (unsigned int addr) {
    
}
int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    TRISDbits.TRISD11 = 1;
    TRISDbits.TRISD0 = 0;
    RPINR18bits.U1RXR = 75; //connect UART1 rx to pin RD11
    RPOR0bits.RP64R = 1; //CONNECT UART 1 tx to pin RD0
    U1BRG = ubrg_value;
    //enable UART
    U1MODEbits.UARTEN = 1;
    //enable tx on uart1
    U1STAbits.UTXEN = 1;
    
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.UTXISEL0 = 0;
    
    
    TRISAbits.TRISA1 = 1;
    TRISFbits.TRISF12 = 0;
    TRISFbits.TRISF13 = 0;
    
    RPINR20BITS.SDI1R = 17;
    RPOR12bits.RP109R = 0b000101; 
    RPOR11bits.RP108R = 0b000110; 
    
    SPI1CON1bits.MSTEN = 1;
    SPI1CON1bits.MODE16 = 0;
    
    
    
    return 0;
}
