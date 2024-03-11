/*
 * File:   main.c
 * Author: nikit
 *
 * Created on March 6, 2024, 1:59 PM
 */


#include "xc.h"

int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0X0000; //set ports to digital
    TRISAbits.TRISA0 = 0;//set port to output
    TRISEbits.TRISE8 = 1;//set port to input
    while (1) {
        /*//turn on led
        LATAbits.LATA0 = 1;
        */
        /*//turn on led if button pressed
        LATAbits.LATA0 = !PORTEbits.RE8;
        */
        //toggle led when button pressed
        if (PORTEbits.RE8 == 0) {
            while (PORTEbits.RE8 == 0) {}
            LATAbits.LATA0 = !LATAbits.LATA0;
        }
        
    }
    return 0;
}
