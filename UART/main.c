/*
 * File:   main.c
 * Author: nikit
 *
 * Created on 21 marzo 2024, 13.02
 */


#include "xc.h"
#include "timer.h"
#define baud_rate 9600
#define ubrg_value ((72000000 / 16) / baud_rate) - 1
//char rec;


void __attribute__ ((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
        //IFS0bits.U1RXIF = 0;/**/
        
        //if (U1STAbits.URXDA == 1) {
        //LATAbits.LATA0 = !LATAbits.LATA0;
    char rec;
    rec = U1RXREG;
    //U1STAbits.UTXEN = 1;
    U1TXREG = rec;
    //U1STAbits.UTXEN = 0;
    //tmr_setup_peiriod (TIMER2, 10);
    //U1TXREG = U1RXREG;
    IFS0bits.U1RXIF = 0;
}


int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISEbits.TRISE8 = 1;
    TRISEbits.TRISE9 = 1;
    /*configure UART1 to mikrobus 2*/
    TRISDbits.TRISD11 = 1;
    TRISDbits.TRISD0 = 0;
    RPINR18bits.U1RXR = 75; //connect UART1 rx to pin RD11
    //RPINR18bits.U1RXR = 64
    RPOR0bits.RP64R = 1; //CONNECT UART 1 tx to pin RD0
    //setup for 9600 bds
    
    //long ubrg_value = ((72000000 / 16) / baud_rate) - 1;
    U1BRG = ubrg_value;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
    //enable UART
    U1MODEbits.UARTEN = 1;
    //enable tx on uart1
    U1STAbits.UTXEN = 1;
    //set the flag for reception
    U1STAbits.URXISEL = 0b00;
    //char msg = "test";
    //U1TXREG = 'T';
    /*for (int i = 0; i < 5; i++) {
        U1TXREG = msg [i];
    }*/
    /*RPINR0bits.INT1R = ; //map interrupt INT1 to RPI75 (pin of rd11)
    INTCON2bits.GIE = 1;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;*/
    
    
    //U1TXREG = 'T';
    //U1STAbits.UTXEN = 0;
    while (1) {
    /*basic*/
        /**/
        //LATGbits.LATG9 = !U1STAbits.TRMT;
        /*if (IFS0bits.U1RXIF == 1) {
            IFS0bits.U1RXIF = 0;/**/
        //if (U1STAbits.URXDA == 1) {
        /*    LATAbits.LATA0 = !LATAbits.LATA0;
            /*rec = U1RXREG;
            U1TXREG = rec;*/
            //U1TXREG = U1RXREG;
        //}
        
        
    }
    return 0;
}
