/*
 * File:   main.c
 * Author: nikit
 *
 * Created on March 12, 2024, 12:39 PM
 */


#include "xc.h"
#include "timer.h"

void __attribute__((__interrupt__, __auto_pav__)) _T2Interrupt () {
    IFS0bits.T2IF = 0;
    LATGbits.LATG9 = !LATGbits.LATG9;    
}
void __attribute__((__interrupt__, __auto_pav__)) _INT1Interrupt () {
    IFS1bits.INT1IF = 0;
    LATGbits.LATG9 = !LATGbits.LATG9;    
} 

int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0X0000; //set ports to digital
    TRISAbits.TRISA0 = 0;//set port to output
    TRISGbits.TRISG9 = 0;
    TRISEbits.TRISE8 = 1;//set port to input
    tmr_setup_period (TIMER1, 200); //set period to 200 ms
    tmr_setup_period (TIMER2, 100); //set period to 100 ms
    //enable interrupt on TIMER 2
    IEC0bits.T2IE = 0;
    //map INT1 to pin RE8
    RPINR0bits.INT1R = 0x58; //map interrupt INT1 to RPI88 (pin of button T2)
    INTCON2bits.GIE = 1;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;
    
    
    
    
    
    while (1) {
        /*part1*/
        /*
        LATAbits.LATA0 = !LATAbits.LATA0;
        tmr_wait_period (TIMER1);
        /**/
        /*part 2*/
        /**/
        LATAbits.LATA0 = !LATAbits.LATA0;
        tmr_wait_period (TIMER1);
        /**/
    }
    return 0;
}
